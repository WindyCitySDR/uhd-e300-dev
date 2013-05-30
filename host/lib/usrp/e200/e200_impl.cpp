//
// Copyright 2013 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include "e200_impl.hpp"
#include "e200_regs.hpp"
#include <uhd/utils/msg.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/images.hpp>
#include <uhd/usrp/dboard_eeprom.hpp>
#include <uhd/transport/udp_zero_copy.hpp>
#include <uhd/types/sensors.hpp>
#include <boost/filesystem.hpp>
#include <boost/functional/hash.hpp>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/thread/thread.hpp> //sleep
#include <boost/asio.hpp>
#include <fstream>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace fs = boost::filesystem;
namespace asio = boost::asio;

/***********************************************************************
 * Discovery
 **********************************************************************/
static device_addrs_t e200_find(const device_addr_t &hint)
{
    device_addrs_t e200_addrs;

    //return an empty list of addresses when type is set to non-e200
    if (hint.has_key("type") and hint["type"] != "e200") return e200_addrs;

    // need network discovery that takes addr
    if (hint.has_key("addr")) try
    {
        asio::io_service io_service;
        asio::ip::udp::resolver resolver(io_service);
        asio::ip::udp::resolver::query query(asio::ip::udp::v4(), hint["addr"], E200_SERVER_CODEC_PORT);
        asio::ip::udp::endpoint endpoint = *resolver.resolve(query);
        {
            boost::shared_ptr<asio::ip::udp::socket> socket;
            socket.reset(new asio::ip::udp::socket(io_service));
            socket->connect(endpoint);
            socket->close();
        }
        device_addr_t new_addr;
        new_addr["type"] = "e200";
        new_addr["addr"] = hint["addr"];
        e200_addrs.push_back(new_addr);
        return e200_addrs;
    }
    catch(...){}

    //device node not provided, assume its 0
    if (not hint.has_key("node"))
    {
        device_addr_t new_addr = hint;
        new_addr["node"] = "/dev/axi_fpga";
        return e200_find(new_addr);
    }

    //use the given device node name
    if (fs::exists(hint["node"]))
    {
        device_addr_t new_addr;
        new_addr["type"] = "e200";
        new_addr["node"] = fs::system_complete(fs::path(hint["node"])).string();
        //TODO read EEPROM!
        new_addr["name"] = "";
        new_addr["serial"] = "";
        if (
            (not hint.has_key("name")   or hint["name"]   == new_addr["name"]) and
            (not hint.has_key("serial") or hint["serial"] == new_addr["serial"])
        ){
            e200_addrs.push_back(new_addr);
        }
    }

    return e200_addrs;
}

/***********************************************************************
 * Make
 **********************************************************************/
static device::sptr e200_make(const device_addr_t &device_addr)
{
    return device::sptr(new e200_impl(device_addr));
}

UHD_STATIC_BLOCK(register_e200_device)
{
    device::register_device(&e200_find, &e200_make);
}

/***********************************************************************
 * Structors
 **********************************************************************/
e200_impl::e200_impl(const uhd::device_addr_t &device_addr)
{
    e200_impl_begin:
    ////////////////////////////////////////////////////////////////////
    // load the fpga image
    ////////////////////////////////////////////////////////////////////
    if (not device_addr.has_key("addr"))
    {
        //extract the FPGA path for the E200
        const std::string e200_fpga_image = find_image_path(
            device_addr.get("fpga", E200_FPGA_FILE_NAME)
        );

        //load fpga image - its super fast
        this->load_fpga_image(e200_fpga_image);
    }

    ////////////////////////////////////////////////////////////////////
    // setup fifo xports
    ////////////////////////////////////////////////////////////////////
    uhd::device_addr_t ctrl_xport_args;
    ctrl_xport_args["recv_frame_size"] = "64";
    ctrl_xport_args["num_recv_frames"] = "32";
    ctrl_xport_args["send_frame_size"] = "64";
    ctrl_xport_args["num_send_frames"] = "32";

    uhd::device_addr_t data_xport_args;
    data_xport_args["recv_frame_size"] = "2048";
    data_xport_args["num_recv_frames"] = "128";
    data_xport_args["send_frame_size"] = "2048";
    data_xport_args["num_send_frames"] = "128";

    _network_mode = device_addr.has_key("addr");

    if (_network_mode)
    {
        radio_perifs_t &perif = _radio_perifs[0];
        perif.send_ctrl_xport = udp_zero_copy::make(device_addr["addr"], E200_SERVER_CTRL_PORT);
        perif.recv_ctrl_xport = perif.send_ctrl_xport;
        perif.tx_data_xport = udp_zero_copy::make(device_addr["addr"], E200_SERVER_TX_PORT);
        perif.tx_flow_xport = perif.tx_data_xport;
        perif.rx_data_xport = udp_zero_copy::make(device_addr["addr"], E200_SERVER_RX_PORT);
        perif.rx_flow_xport = perif.rx_data_xport;
        zero_copy_if::sptr codec_xport;
        codec_xport = udp_zero_copy::make(device_addr["addr"], E200_SERVER_CODEC_PORT);
        ad9361_ctrl_iface_sptr codec_ctrl(new ad9361_ctrl_over_zc(codec_xport));
        _codec_ctrl = ad9361_ctrl::make(codec_ctrl);
    }
    else
    {
        radio_perifs_t &perif = _radio_perifs[0];
        _fifo_iface = e200_fifo_interface::make(e200_read_sysfs());
        perif.send_ctrl_xport = _fifo_iface->make_send_xport(1, ctrl_xport_args);
        perif.recv_ctrl_xport = _fifo_iface->make_recv_xport(1, ctrl_xport_args);
        perif.tx_data_xport = _fifo_iface->make_send_xport(0, data_xport_args);
        perif.tx_flow_xport = _fifo_iface->make_recv_xport(0, ctrl_xport_args);
        perif.rx_data_xport = _fifo_iface->make_recv_xport(2, data_xport_args);
        perif.rx_flow_xport = _fifo_iface->make_send_xport(2, ctrl_xport_args);
        zero_copy_if::sptr codec_xport;
        codec_xport = udp_zero_copy::make("localhost", E200_SERVER_CODEC_PORT);
        ad9361_ctrl_iface_sptr codec_ctrl(new ad9361_ctrl_over_zc(codec_xport));
        _codec_ctrl = ad9361_ctrl::make(codec_ctrl);
    }

    ////////////////////////////////////////////////////////////////////
    // optional udp server
    ////////////////////////////////////////////////////////////////////
    if (device_addr.has_key("server"))
    {
        boost::thread_group tg;
        tg.create_thread(boost::bind(&e200_impl::run_server, this, E200_SERVER_RX_PORT, "RX"));
        tg.create_thread(boost::bind(&e200_impl::run_server, this, E200_SERVER_TX_PORT, "TX"));
        tg.create_thread(boost::bind(&e200_impl::run_server, this, E200_SERVER_CTRL_PORT, "CTRL"));
        tg.join_all();
        goto e200_impl_begin;
    }

    ////////////////////////////////////////////////////////////////////
    // Initialize the properties tree
    ////////////////////////////////////////////////////////////////////
    _tree = property_tree::make();
    _tree->create<std::string>("/name").set("E-Series Device");
    const fs_path mb_path = "/mboards/0";
    _tree->create<std::string>(mb_path / "name").set("uSerp");
    _tree->create<std::string>(mb_path / "codename").set("");

    ////////////////////////////////////////////////////////////////////
    // and do the misc mboard sensors
    ////////////////////////////////////////////////////////////////////
    _tree->create<int>(mb_path / "sensors"); //empty TODO

    ////////////////////////////////////////////////////////////////////
    // setup the mboard eeprom
    ////////////////////////////////////////////////////////////////////
    // TODO
    mboard_eeprom_t mb_eeprom;
    mb_eeprom["name"] = "TODO"; //FIXME with real eeprom values
    mb_eeprom["serial"] = "TODO"; //FIXME with real eeprom values
    _tree->create<mboard_eeprom_t>(mb_path / "eeprom")
        .set(mb_eeprom);

    ////////////////////////////////////////////////////////////////////
    // clocking
    ////////////////////////////////////////////////////////////////////
    _tree->create<double>(mb_path / "tick_rate")
        .coerce(boost::bind(&e200_impl::set_tick_rate, this, _1))
        .publish(boost::bind(&e200_impl::get_tick_rate, this))
        .subscribe(boost::bind(&e200_impl::update_tick_rate, this, _1));

    //default some chains on -- needed for setup purposes
    _codec_ctrl->set_active_chains(true, false, true, false);
    _codec_ctrl->set_clock_rate(50e6);

    ////////////////////////////////////////////////////////////////////
    // setup radio goo
    ////////////////////////////////////////////////////////////////////
    this->setup_radio(0);
    //TODO this->setup_radio(1);


    _codec_ctrl->data_port_loopback(true);
    this->codec_loopback_self_test(_radio_perifs[0].ctrl);
    _codec_ctrl->data_port_loopback(false);

    ////////////////////////////////////////////////////////////////////
    // register the time keepers - only one can be the highlander
    ////////////////////////////////////////////////////////////////////
    _tree->create<time_spec_t>(mb_path / "time" / "now")
        .publish(boost::bind(&time_core_3000::get_time_now, _radio_perifs[0].time64))
        .subscribe(boost::bind(&time_core_3000::set_time_now, _radio_perifs[0].time64, _1))
        ;//.subscribe(boost::bind(&time_core_3000::set_time_now, _radio_perifs[1].time64, _1));
    _tree->create<time_spec_t>(mb_path / "time" / "pps")
        .publish(boost::bind(&time_core_3000::get_time_last_pps, _radio_perifs[0].time64))
        .subscribe(boost::bind(&time_core_3000::set_time_next_pps, _radio_perifs[0].time64, _1))
        ;//.subscribe(boost::bind(&time_core_3000::set_time_next_pps, _radio_perifs[1].time64, _1));
    //setup time source props
    _tree->create<std::string>(mb_path / "time_source" / "value")
        .subscribe(boost::bind(&e200_impl::update_time_source, this, _1));
    static const std::vector<std::string> time_sources = boost::assign::list_of("none")("external")("gpsdo");
    _tree->create<std::vector<std::string> >(mb_path / "time_source" / "options").set(time_sources);
    //setup reference source props
    _tree->create<std::string>(mb_path / "clock_source" / "value")
        .subscribe(boost::bind(&e200_impl::update_clock_source, this, _1));
    static const std::vector<std::string> clock_sources = boost::assign::list_of("internal")("external")("gpsdo");
    _tree->create<std::vector<std::string> >(mb_path / "clock_source" / "options").set(clock_sources);

    ////////////////////////////////////////////////////////////////////
    // dboard eeproms but not really
    ////////////////////////////////////////////////////////////////////
    dboard_eeprom_t db_eeprom;
    _tree->create<dboard_eeprom_t>(mb_path / "dboards" / "A" / "rx_eeprom").set(db_eeprom);
    _tree->create<dboard_eeprom_t>(mb_path / "dboards" / "A" / "tx_eeprom").set(db_eeprom);
    _tree->create<dboard_eeprom_t>(mb_path / "dboards" / "A" / "gdb_eeprom").set(db_eeprom);

    ////////////////////////////////////////////////////////////////////
    // create RF frontend interfacing
    ////////////////////////////////////////////////////////////////////
    static const std::vector<std::string> frontends = boost::assign::list_of
        ("TX1")("TX2")("RX1")("RX2");

    {
        const fs_path codec_path = mb_path / ("rx_codecs") / "A";
        _tree->create<std::string>(codec_path / "name").set("B200 RX dual ADC");
        _tree->create<int>(codec_path / "gains"); //empty cuz gains are in frontend
    }
    {
        const fs_path codec_path = mb_path / ("tx_codecs") / "A";
        _tree->create<std::string>(codec_path / "name").set("B200 TX dual DAC");
        _tree->create<int>(codec_path / "gains"); //empty cuz gains are in frontend
    }

    BOOST_FOREACH(const std::string &fe_name, frontends)
    {
        const std::string x = std::string(1, tolower(fe_name[0]));
        const fs_path rf_fe_path = mb_path / "dboards" / "A" / (x+"x_frontends") / fe_name;

        _tree->create<std::string>(rf_fe_path / "name").set(fe_name);
        _tree->create<int>(rf_fe_path / "sensors"); //empty TODO
        BOOST_FOREACH(const std::string &name, ad9361_ctrl::get_gain_names(fe_name))
        {
            _tree->create<meta_range_t>(rf_fe_path / "gains" / name / "range")
                .set(ad9361_ctrl::get_gain_range(fe_name));

            _tree->create<double>(rf_fe_path / "gains" / name / "value")
                .coerce(boost::bind(&ad9361_ctrl::set_gain, _codec_ctrl, fe_name, _1))
                .set(0.0);
        }
        _tree->create<std::string>(rf_fe_path / "connection").set("IQ");
        _tree->create<bool>(rf_fe_path / "enabled").set(true);
        _tree->create<bool>(rf_fe_path / "use_lo_offset").set(false);
        _tree->create<double>(rf_fe_path / "bandwidth" / "value")
            .coerce(boost::bind(&ad9361_ctrl::set_bw_filter, _codec_ctrl, fe_name, _1))
            .set(40e6);
        _tree->create<meta_range_t>(rf_fe_path / "bandwidth" / "range")
            .publish(boost::bind(&ad9361_ctrl::get_bw_filter_range, fe_name));
        _tree->create<double>(rf_fe_path / "freq" / "value")
            .set(0.0)
            .coerce(boost::bind(&ad9361_ctrl::tune, _codec_ctrl, fe_name, _1))
            .subscribe(boost::bind(&e200_impl::update_fe_lo_freq, this, fe_name, _1));
        _tree->create<meta_range_t>(rf_fe_path / "freq" / "range")
            .publish(boost::bind(&ad9361_ctrl::get_rf_freq_range));

        //setup antenna stuff
        if (fe_name[0] == 'R')
        {
            static const std::vector<std::string> ants = boost::assign::list_of("TX/RX")("RX2");
            _tree->create<std::vector<std::string> >(rf_fe_path / "antenna" / "options").set(ants);
            _tree->create<std::string>(rf_fe_path / "antenna" / "value")
                .subscribe(boost::bind(&e200_impl::update_antenna_sel, this, fe_name, _1))
                .set("RX2");
        }
        if (fe_name[0] == 'T')
        {
            static const std::vector<std::string> ants(1, "TX/RX");
            _tree->create<std::vector<std::string> >(rf_fe_path / "antenna" / "options").set(ants);
            _tree->create<std::string>(rf_fe_path / "antenna" / "value").set("TX/RX");
        }

    }

    ////////////////////////////////////////////////////////////////////
    // create frontend mapping
    ////////////////////////////////////////////////////////////////////
    _tree->create<subdev_spec_t>(mb_path / "rx_subdev_spec")
        .set(subdev_spec_t())
        .subscribe(boost::bind(&e200_impl::update_rx_subdev_spec, this, _1));
    _tree->create<subdev_spec_t>(mb_path / "tx_subdev_spec")
        .set(subdev_spec_t())
        .subscribe(boost::bind(&e200_impl::update_tx_subdev_spec, this, _1));

    ////////////////////////////////////////////////////////////////////
    // do some post-init tasks
    ////////////////////////////////////////////////////////////////////

    //init the clock rate to something, but only when we have active chains
    _tree->access<double>(mb_path / "tick_rate").set(61.44e6/2);
    //_codec_ctrl->set_active_chains(false, false, false, false);

    _tree->access<subdev_spec_t>(mb_path / "rx_subdev_spec").set(subdev_spec_t("A:RX2"));
    _tree->access<subdev_spec_t>(mb_path / "tx_subdev_spec").set(subdev_spec_t("A:TX2"));

    _tree->access<std::string>(mb_path / "clock_source" / "value").set("internal");
    _tree->access<std::string>(mb_path / "time_source" / "value").set("none");

}

e200_impl::~e200_impl(void)
{
    /* NOP */
}

double e200_impl::set_tick_rate(const double raw_rate)
{
    //clip rate (which can be doubled by factor) to possible bounds
    const double rate = ad9361_ctrl::get_samp_rate_range().clip(raw_rate);

    const size_t factor = 1.0;//((_fe_enb_map["RX1"] and _fe_enb_map["RX2"]) or (_fe_enb_map["TX1"] and _fe_enb_map["TX2"]))? 2:1;
    UHD_MSG(status) << "asking for clock rate " << rate/1e6 << " MHz\n";
    _tick_rate = _codec_ctrl->set_clock_rate(rate/factor)*factor;
    UHD_MSG(status) << "actually got clock rate " << _tick_rate/1e6 << " MHz\n";

    BOOST_FOREACH(radio_perifs_t &perif, _radio_perifs)
    {
        perif.time64->set_tick_rate(_tick_rate);
        perif.time64->self_test();
    }
    return _tick_rate;
}

void e200_impl::load_fpga_image(const std::string &path)
{
    if (not fs::exists("/dev/xdevcfg"))
    {
        ::system("mknod /dev/xdevcfg c 259 0");
        //throw uhd::runtime_error("no xdevcfg, please run: mknod /dev/xdevcfg c 259 0");
    }

    UHD_MSG(status) << "Loading FPGA image: " << path << "..." << std::flush;

    std::ifstream fpga_file(path.c_str(), std::ios_base::binary);
    UHD_ASSERT_THROW(fpga_file.good());

    std::FILE *wfile;
    wfile = std::fopen("/dev/xdevcfg", "wb");
    UHD_ASSERT_THROW(!(wfile == NULL));

    char buff[16384]; // devcfg driver can't handle huge writes
    do {
        fpga_file.read(buff, sizeof(buff));
        std::fwrite(buff, 1, fpga_file.gcount(), wfile);
    } while (fpga_file);

    fpga_file.close();
    std::fclose(wfile);

    UHD_MSG(status) << " done" << std::endl;
}

void e200_impl::register_loopback_self_test(wb_iface::sptr iface)
{
    bool test_fail = false;
    UHD_MSG(status) << "Performing register loopback test... " << std::flush;
    size_t hash = time(NULL);
    for (size_t i = 0; i < 100; i++)
    {
        boost::hash_combine(hash, i);
        iface->poke32(TOREG(SR_TEST), boost::uint32_t(hash));
        test_fail = iface->peek32(RB32_TEST) != boost::uint32_t(hash);
        if (test_fail) break; //exit loop on any failure
    }
    UHD_MSG(status) << ((test_fail)? " fail" : "pass") << std::endl;
}

void e200_impl::codec_loopback_self_test(wb_iface::sptr iface)
{
    bool test_fail = false;
    UHD_ASSERT_THROW(bool(iface));
    UHD_MSG(status) << "Performing CODEC loopback test... " << std::flush;
    size_t hash = time(NULL);
    for (size_t i = 0; i < 100; i++)
    {
        boost::hash_combine(hash, i);
        const boost::uint32_t word32 = boost::uint32_t(hash) & 0xfff0fff0;
        iface->poke32(TOREG(SR_CODEC_IDLE), word32);
        iface->peek64(RB64_CODEC_READBACK); //enough idleness for loopback to propagate
        const boost::uint64_t rb_word64 = iface->peek64(RB64_CODEC_READBACK);
        const boost::uint32_t rb_tx = boost::uint32_t(rb_word64 >> 32);
        const boost::uint32_t rb_rx = boost::uint32_t(rb_word64 & 0xffffffff);
        test_fail = word32 != rb_tx or word32 != rb_rx;
        if (test_fail) break; //exit loop on any failure
    }
    UHD_MSG(status) << ((test_fail)? " fail" : "pass") << std::endl;

    /* Zero out the idle data. */
    iface->poke32(TOREG(SR_CODEC_IDLE), 0);
}

void e200_impl::update_time_source(const std::string &)
{
    
}

void e200_impl::update_clock_source(const std::string &)
{
    
}

void e200_impl::update_antenna_sel(const std::string &fe, const std::string &ant)
{
    const size_t i = (fe == "RX1")? 0 : 1;
    _fe_control_settings[i].rx_ant = ant;
    this->update_atrs(i);
}

void e200_impl::update_fe_lo_freq(const std::string &fe, const double freq)
{
    for (size_t i = 0; i < 2; i++)
    {
        if (fe[0] == 'R') _fe_control_settings[i].rx_freq = freq;
        if (fe[0] == 'T') _fe_control_settings[i].tx_freq = freq;
        this->update_atrs(i);
    }
}

void e200_impl::update_active_frontends(void)
{
    _codec_ctrl->set_active_chains(
        _fe_control_settings[0].tx_enb,
        _fe_control_settings[1].tx_enb,
        _fe_control_settings[0].rx_enb,
        _fe_control_settings[1].rx_enb
    );
    this->update_atrs(0);
    this->update_atrs(1);
}

void e200_impl::setup_radio(const size_t dspno)
{
    radio_perifs_t &perif = _radio_perifs[dspno];
    const fs_path mb_path = "/mboards/0";

    ////////////////////////////////////////////////////////////////////
    // radio control
    ////////////////////////////////////////////////////////////////////
    perif.ctrl = radio_ctrl_core_3000::make(vrt::if_packet_info_t::LINK_TYPE_CHDR, perif.send_ctrl_xport, perif.recv_ctrl_xport, 1 | (1 << 16));
    this->register_loopback_self_test(perif.ctrl);
    perif.atr0 = gpio_core_200_32wo::make(perif.ctrl, TOREG(SR_GPIO));
    perif.atr1 = gpio_core_200_32wo::make(perif.ctrl, TOREG(SR_GPIO2));

    ////////////////////////////////////////////////////////////////////
    // create rx dsp control objects
    ////////////////////////////////////////////////////////////////////
    perif.framer = rx_vita_core_3000::make(perif.ctrl, TOREG(SR_RX_CTRL));
    perif.ddc = rx_dsp_core_3000::make(perif.ctrl, TOREG(SR_RX_DSP));
    perif.ddc->set_link_rate(10e9/8); //whatever
    _tree->access<double>(mb_path / "tick_rate")
        .subscribe(boost::bind(&rx_vita_core_3000::set_tick_rate, perif.framer, _1))
        .subscribe(boost::bind(&rx_dsp_core_3000::set_tick_rate, perif.ddc, _1));
    const fs_path rx_dsp_path = mb_path / "rx_dsps" / str(boost::format("%u") % dspno);
    _tree->create<meta_range_t>(rx_dsp_path / "rate" / "range")
        .publish(boost::bind(&rx_dsp_core_3000::get_host_rates, perif.ddc));
    _tree->create<double>(rx_dsp_path / "rate" / "value")
        .coerce(boost::bind(&rx_dsp_core_3000::set_host_rate, perif.ddc, _1))
        .subscribe(boost::bind(&e200_impl::update_rx_samp_rate, this, dspno, _1))
        .set(1e6);
    _tree->create<double>(rx_dsp_path / "freq" / "value")
        .coerce(boost::bind(&rx_dsp_core_3000::set_freq, perif.ddc, _1))
        .set(0.0);
    _tree->create<meta_range_t>(rx_dsp_path / "freq" / "range")
        .publish(boost::bind(&rx_dsp_core_3000::get_freq_range, perif.ddc));
    _tree->create<stream_cmd_t>(rx_dsp_path / "stream_cmd")
        .subscribe(boost::bind(&rx_vita_core_3000::issue_stream_command, perif.framer, _1));

    ////////////////////////////////////////////////////////////////////
    // create tx dsp control objects
    ////////////////////////////////////////////////////////////////////
    perif.deframer = tx_vita_core_3000::make(perif.ctrl, TOREG(SR_TX_CTRL));
    perif.duc = tx_dsp_core_3000::make(perif.ctrl, TOREG(SR_TX_DSP));
    perif.duc->set_link_rate(10e9/8); //whatever
    _tree->access<double>(mb_path / "tick_rate")
        .subscribe(boost::bind(&tx_vita_core_3000::set_tick_rate, perif.deframer, _1))
        .subscribe(boost::bind(&tx_dsp_core_3000::set_tick_rate, perif.duc, _1));
    const fs_path tx_dsp_path = mb_path / "tx_dsps" / str(boost::format("%u") % dspno);
    _tree->create<meta_range_t>(tx_dsp_path / "rate" / "range")
        .publish(boost::bind(&tx_dsp_core_3000::get_host_rates, perif.duc));
    _tree->create<double>(tx_dsp_path / "rate" / "value")
        .coerce(boost::bind(&tx_dsp_core_3000::set_host_rate, perif.duc, _1))
        .subscribe(boost::bind(&e200_impl::update_tx_samp_rate, this, dspno, _1))
        .set(1e6);
    _tree->create<double>(tx_dsp_path / "freq" / "value")
        .coerce(boost::bind(&tx_dsp_core_3000::set_freq, perif.duc, _1))
        .set(0.0);
    _tree->create<meta_range_t>(tx_dsp_path / "freq" / "range")
        .publish(boost::bind(&tx_dsp_core_3000::get_freq_range, perif.duc));

    ////////////////////////////////////////////////////////////////////
    // create time control objects
    ////////////////////////////////////////////////////////////////////
    time_core_3000::readback_bases_type time64_rb_bases;
    time64_rb_bases.rb_now = RB64_TIME_NOW;
    time64_rb_bases.rb_pps = RB64_TIME_PPS;
    perif.time64 = time_core_3000::make(perif.ctrl, TOREG(SR_TIME), time64_rb_bases);
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
//////////////// ATR SETUP FOR FRONTEND CONTROL VIA GPIO ///////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void e200_impl::update_atrs(const size_t &fe)
{
    const fe_control_settings_t &settings = _fe_control_settings[fe];

    //----------------- rx ant comprehension --------------------//
    const bool rx_ant_rx2 = settings.rx_ant == "RX2";
    const bool rx_ant_txrx = settings.rx_ant == "TX/RX";

    //----------------- high/low band decision --------------------//
    const bool rx_low_band = settings.rx_freq < 2.6e9;
    const bool rx_high_band = not rx_low_band;
    const bool tx_low_band = settings.tx_freq < 2940.0e6;
    const bool tx_high_band = not tx_low_band;

    //------------------- TX low band bandsel ----------------------//
    int tx_bandsels = 0;
    if      (settings.tx_freq < 117.7e6)   tx_bandsels = 0;
    else if (settings.tx_freq < 178.2e6)   tx_bandsels = 1;
    else if (settings.tx_freq < 284.3e6)   tx_bandsels = 2;
    else if (settings.tx_freq < 453.7e6)   tx_bandsels = 3;
    else if (settings.tx_freq < 723.8e6)   tx_bandsels = 4;
    else if (settings.tx_freq < 1154.9e6)  tx_bandsels = 5;
    else if (settings.tx_freq < 1842.6e6)  tx_bandsels = 6;
    else if (settings.tx_freq < 2940.0e6)  tx_bandsels = 7;
    else                                   tx_bandsels = 0;

    //------------------- RX low band bandsel ----------------------//
    int rx_bandsels = 0, rx_bandsel_b = 0, rx_bandsel_c = 0;
    if(fe == 0) {
        if      (settings.rx_freq < 450e6)   {rx_bandsels = 4; rx_bandsel_b = 0; rx_bandsel_c = 2;}
        else if (settings.rx_freq < 700e6)   {rx_bandsels = 2; rx_bandsel_b = 0; rx_bandsel_c = 3;}
        else if (settings.rx_freq < 1200e6)  {rx_bandsels = 0; rx_bandsel_b = 0; rx_bandsel_c = 1;}
        else if (settings.rx_freq < 1800e6)  {rx_bandsels = 1; rx_bandsel_b = 2; rx_bandsel_c = 0;}
        else if (settings.rx_freq < 2350e6)  {rx_bandsels = 3; rx_bandsel_b = 3; rx_bandsel_c = 0;}
        else if (settings.rx_freq < 2600e6)  {rx_bandsels = 5; rx_bandsel_b = 1; rx_bandsel_c = 0;}
        else                                 {rx_bandsels = 0; rx_bandsel_b = 0; rx_bandsel_c = 0;}
    } else {
        if      (settings.rx_freq < 450e6)   {rx_bandsels = 5; rx_bandsel_b = 0; rx_bandsel_c = 1;}
        else if (settings.rx_freq < 700e6)   {rx_bandsels = 3; rx_bandsel_b = 0; rx_bandsel_c = 3;}
        else if (settings.rx_freq < 1200e6)  {rx_bandsels = 1; rx_bandsel_b = 0; rx_bandsel_c = 2;}
        else if (settings.rx_freq < 1800e6)  {rx_bandsels = 0; rx_bandsel_b = 1; rx_bandsel_c = 0;}
        else if (settings.rx_freq < 2350e6)  {rx_bandsels = 2; rx_bandsel_b = 3; rx_bandsel_c = 0;}
        else if (settings.rx_freq < 2600e6)  {rx_bandsels = 4; rx_bandsel_b = 2; rx_bandsel_c = 0;}
        else                                 {rx_bandsels = 0; rx_bandsel_b = 0; rx_bandsel_c = 0;}
    }

    //-------------------- VCRX - rx mode -------------------------//
    int vcrx_1_rxing = 1, vcrx_2_rxing = 0;
    if ((rx_ant_rx2 and rx_low_band) or (rx_ant_txrx and rx_high_band))
    {
        vcrx_1_rxing = 0; vcrx_2_rxing = 1;
    }
    if ((rx_ant_rx2 and rx_high_band) or (rx_ant_txrx and rx_low_band))
    {
        vcrx_1_rxing = 1; vcrx_2_rxing = 0;
    }

    //-------------------- VCRX - tx mode -------------------------//
    int vcrx_1_txing = 1, vcrx_2_txing = 0;
    if (rx_low_band)
    {
        vcrx_1_txing = 0; vcrx_2_txing = 1;
    }
    if (rx_high_band)
    {
        vcrx_1_txing = 1; vcrx_2_txing = 0;
    }

    //-------------------- VCTX - rx mode -------------------------//
    int vctxrx_1_rxing = 0, vctxrx_2_rxing = 1;
    if (rx_ant_txrx)
    {
        vctxrx_1_rxing = 1; vctxrx_2_rxing = 0;
    }
    if (rx_ant_rx2 and tx_high_band)
    {
        vctxrx_1_rxing = 1; vctxrx_2_rxing = 1;
    }
    if (rx_ant_rx2 and tx_low_band)
    {
        vctxrx_1_rxing = 0; vctxrx_2_rxing = 1;
    }

    //-------------------- VCTX - tx mode -------------------------//
    int vctxrx_1_txing = 0, vctxrx_2_txing = 1;
    if (tx_high_band)
    {
        vctxrx_1_txing = 1; vctxrx_2_txing = 1;
    }
    if (tx_low_band)
    {
        vctxrx_1_txing = 0; vctxrx_2_txing = 1;
    }

    //swapped for routing reasons, reswap it here
    if (fe == 1) std::swap(vctxrx_1_txing, vctxrx_2_txing);

    //----------------- TX ENABLES ----------------------------//
    const int tx_enable_a = (tx_high_band and settings.tx_enb)? 1 : 0;
    const int tx_enable_b = (tx_low_band and settings.tx_enb)? 1 : 0;

    //----------------- LEDS ----------------------------//
    const int led_rx2 = (rx_ant_rx2)? 1 : 0;
    const int led_txrx = (rx_ant_txrx)? 1 : 0;
    const int led_tx = 1;

    const int rx_leds = (led_rx2 << LED_RX_RX) | (led_txrx << LED_TXRX_RX);
    const int tx_leds = (led_tx << LED_TXRX_TX);
    const int xx_leds = tx_leds | (1 << LED_RX_RX); //forced to rx2

    //----------------- ATR values ---------------------------//

    const int rx_selects = 0
        | (vcrx_1_rxing << VCRX_V1)
        | (vcrx_2_rxing << VCRX_V2)
        | (vctxrx_1_rxing << VCTXRX_V1)
        | (vctxrx_2_rxing << VCTXRX_V2)
    ;
    const int tx_selects = 0
        | (vcrx_1_txing << VCRX_V1)
        | (vcrx_2_txing << VCRX_V2)
        | (vctxrx_1_txing << VCTXRX_V1)
        | (vctxrx_2_txing << VCTXRX_V2)
    ;
    const int xx_selects = 0
        | (tx_bandsels << TX_BANDSEL)
        | (rx_bandsels << RX_BANDSEL)
        | (rx_bandsel_b << RXB_BANDSEL)
        | (rx_bandsel_c << RXC_BANDSEL)
    ;
    const int tx_enables = 0
        | (tx_enable_a << TX_ENABLEA)
        | (tx_enable_b << TX_ENABLEB)
    ;

    //default selects
    int oo_reg = xx_selects | rx_selects;
    int rx_reg = xx_selects | rx_selects;
    int tx_reg = xx_selects | tx_selects;
    int fd_reg = xx_selects | tx_selects; //tx selects dominate in fd mode

    //add in leds and tx enables based on fe enable
    if (settings.rx_enb) rx_reg |= rx_leds;
    if (settings.rx_enb) fd_reg |= xx_leds;
    if (settings.tx_enb) tx_reg |= tx_enables | tx_leds;
    if (settings.tx_enb) fd_reg |= tx_enables | xx_leds;

    /*
    UHD_VAR(fe);
    UHD_VAR(settings.rx_enb);
    UHD_VAR(settings.tx_enb);
    UHD_VAR(settings.rx_freq);
    UHD_VAR(settings.tx_freq);
    UHD_VAR(tx_bandsels);
    UHD_VAR(rx_bandsels);
    UHD_VAR(rx_bandsel_b);
    UHD_VAR(rx_bandsel_c);
    UHD_VAR(rx_ant_rx2);
    UHD_VAR(rx_ant_txrx);
    UHD_VAR(rx_low_band);
    UHD_VAR(rx_high_band);
    UHD_VAR(vcrx_1_rxing);
    UHD_VAR(vcrx_2_rxing);
    UHD_VAR(vcrx_1_txing);
    UHD_VAR(vcrx_2_txing);
    */

    //load actual values into atr registers
    gpio_core_200_32wo::sptr atr = (fe == 0)? _radio_perifs[0].atr0 : _radio_perifs[0].atr1;
    atr->set_atr_reg(dboard_iface::ATR_REG_IDLE, oo_reg);
    atr->set_atr_reg(dboard_iface::ATR_REG_RX_ONLY, rx_reg);
    atr->set_atr_reg(dboard_iface::ATR_REG_TX_ONLY, tx_reg);
    atr->set_atr_reg(dboard_iface::ATR_REG_FULL_DUPLEX, fd_reg);
}