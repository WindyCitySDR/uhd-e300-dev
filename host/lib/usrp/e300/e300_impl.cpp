//
// Copyright 2013-2014 Ettus Research LLC
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

#include "e300_impl.hpp"
#include "e300_spi.hpp"
#include "e300_regs.hpp"

#include <uhd/utils/msg.hpp>
#include <uhd/utils/log.hpp>
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
static device_addrs_t e300_find(const device_addr_t &hint_)
{
    //we only do single device discovery for now
    const device_addr_t hint = separate_device_addr(hint_).at(0);

    UHD_LOG << "e300_find with hint " << hint.to_pp_string() << std::endl;
    device_addrs_t e300_addrs;

    //return an empty list of addresses when type is set to non-e300
    if (hint.has_key("type") and hint["type"] != "e300") return e300_addrs;

    //Return an empty list of addresses when a resource is specified,
    //since a resource is intended for a different, non-USB, device.
    if (hint.has_key("resource")) return e300_addrs;

    // need network discovery that takes addr
    if (hint.has_key("addr")) try
    {
        UHD_LOG << "e300_find try network discovery..." << std::endl;
        asio::io_service io_service;
        asio::ip::udp::resolver resolver(io_service);
        asio::ip::udp::resolver::query query(asio::ip::udp::v4(), hint["addr"], E300_SERVER_CODEC_PORT);
        asio::ip::udp::endpoint endpoint = *resolver.resolve(query);
        {
            boost::shared_ptr<asio::ip::udp::socket> socket;
            socket.reset(new asio::ip::udp::socket(io_service));
            socket->connect(endpoint);
            socket->close();
        }
        device_addr_t new_addr;
        new_addr["type"] = "e300";
        new_addr["addr"] = hint["addr"];
        e300_addrs.push_back(new_addr);
        UHD_LOG << "e300_find network discovery good " << new_addr.to_pp_string() << std::endl;
        return e300_addrs;
    }
    catch(...)
    {
        UHD_LOG << "e300_find network discovery threw" << std::endl;
    }

    //device node not provided, assume its 0
    if (not hint.has_key("node"))
    {
        device_addr_t new_addr = hint;
        new_addr["node"] = "/dev/axi_fpga";
        return e300_find(new_addr);
    }

    //use the given device node name
    if (fs::exists(hint["node"]))
    {
        device_addr_t new_addr;
        new_addr["type"] = "e300";
        new_addr["node"] = fs::system_complete(fs::path(hint["node"])).string();
        //TODO read EEPROM!
        new_addr["name"] = "";
        new_addr["serial"] = "";
        if (
            (not hint.has_key("name")   or hint["name"]   == new_addr["name"]) and
            (not hint.has_key("serial") or hint["serial"] == new_addr["serial"])
        ){
            e300_addrs.push_back(new_addr);
        }
    }

    return e300_addrs;
}

/***********************************************************************
 * Make
 **********************************************************************/
static device::sptr e300_make(const device_addr_t &device_addr)
{
    UHD_LOG << "e300_make with args " << device_addr.to_pp_string() << std::endl;
    return device::sptr(new e300_impl(device_addr));
}

UHD_STATIC_BLOCK(register_e300_device)
{
    device::register_device(&e300_find, &e300_make);
}

/***********************************************************************
 * Structors
 **********************************************************************/
e300_impl::e300_impl(const uhd::device_addr_t &device_addr) : _sid_framer(0)
{
    _async_md.reset(new async_md_type(1000/*messages deep*/));

    e300_impl_begin:
    ////////////////////////////////////////////////////////////////////
    // load the fpga image
    ////////////////////////////////////////////////////////////////////
    if (not device_addr.has_key("addr"))
    {
        //extract the FPGA path for the e300
        const std::string e300_fpga_image = find_image_path(
            device_addr.get("fpga", E300_FPGA_FILE_NAME)
        );

        if (not device_addr.has_key("no_reload_fpga"))
            this->load_fpga_image(e300_fpga_image);
    }

    ////////////////////////////////////////////////////////////////////
    // setup fifo xports
    ////////////////////////////////////////////////////////////////////
    uhd::transport::zero_copy_xport_params ctrl_xport_params;
    ctrl_xport_params.recv_frame_size = 64;
    ctrl_xport_params.num_recv_frames = 32;
    ctrl_xport_params.send_frame_size = 64;
    ctrl_xport_params.num_send_frames = 32;

    uhd::transport::zero_copy_xport_params data_xport_params;
    data_xport_params.recv_frame_size = 2048;
    data_xport_params.num_recv_frames = 128;
    data_xport_params.send_frame_size = 2048;
    data_xport_params.num_send_frames = 128;

    uhd::device_addr_t ctrl_xport_args;
    ctrl_xport_args["recv_frame_size"] = str(boost::format("%d") % ctrl_xport_params.recv_frame_size);
    ctrl_xport_args["num_recv_frames"] = str(boost::format("%d") % ctrl_xport_params.num_recv_frames);
    ctrl_xport_args["send_frame_size"] = str(boost::format("%d") % ctrl_xport_params.send_frame_size);
    ctrl_xport_args["num_send_frames"] = str(boost::format("%d") % ctrl_xport_params.num_send_frames);

    uhd::device_addr_t data_xport_args;
    data_xport_args["recv_frame_size"] = str(boost::format("%d") % data_xport_params.recv_frame_size);
    data_xport_args["num_recv_frames"] = str(boost::format("%d") % data_xport_params.num_recv_frames);
    data_xport_args["send_frame_size"] = str(boost::format("%d") % data_xport_params.send_frame_size);
    data_xport_args["num_send_frames"] = str(boost::format("%d") % data_xport_params.num_send_frames);

    _network_mode = device_addr.has_key("addr");

    udp_zero_copy::buff_params dummy_buff_params_out;

    if (_network_mode)
    {
        radio_perifs_t &perif = _radio_perifs[0];
        perif.send_ctrl_xport = udp_zero_copy::make(device_addr["addr"], E300_SERVER_CTRL_PORT, ctrl_xport_params, dummy_buff_params_out, device_addr);
        perif.recv_ctrl_xport = perif.send_ctrl_xport;
        perif.tx_data_xport = udp_zero_copy::make(device_addr["addr"], E300_SERVER_TX_PORT, data_xport_params, dummy_buff_params_out, device_addr);
        perif.tx_flow_xport = perif.tx_data_xport;
        perif.rx_data_xport = udp_zero_copy::make(device_addr["addr"], E300_SERVER_RX_PORT, data_xport_params, dummy_buff_params_out, device_addr);
        perif.rx_flow_xport = perif.rx_data_xport;
        zero_copy_if::sptr codec_xport;
        codec_xport = udp_zero_copy::make(device_addr["addr"], E300_SERVER_CODEC_PORT, ctrl_xport_params, dummy_buff_params_out, device_addr);
        _codec_ctrl = ad9361_ctrl::make(ad9361_ctrl_transport::make_zero_copy(codec_xport));
        zero_copy_if::sptr gregs_xport;
        gregs_xport = udp_zero_copy::make(device_addr["addr"], E300_SERVER_GREGS_PORT, ctrl_xport_params, dummy_buff_params_out, device_addr);
        _global_regs = uhd::usrp::e300::global_regs::make(gregs_xport);
    }
    else
    {
        e300_fifo_config_t fifo_cfg;
        try {
            fifo_cfg = e300_read_sysfs();
        } catch (uhd::lookup_error &e) {
            throw uhd::runtime_error("Failed to get driver parameters from sysfs.");
        }
        _fifo_iface = e300_fifo_interface::make(fifo_cfg);
        _global_regs = uhd::usrp::e300::global_regs::make(_fifo_iface->get_global_regs_base());

        // static mapping, boooohhhhhh
        _radio_perifs[0].send_ctrl_xport = _fifo_iface->make_send_xport(E300_R0_CTRL_STREAM, ctrl_xport_args);
        _radio_perifs[0].recv_ctrl_xport = _fifo_iface->make_recv_xport(E300_R0_CTRL_STREAM, ctrl_xport_args);
        _radio_perifs[0].tx_data_xport   = _fifo_iface->make_send_xport(E300_R0_TX_DATA_STREAM, data_xport_args);
        _radio_perifs[0].tx_flow_xport   = _fifo_iface->make_recv_xport(E300_R0_TX_DATA_STREAM, ctrl_xport_args);
        _radio_perifs[0].rx_data_xport   = _fifo_iface->make_recv_xport(E300_R0_RX_DATA_STREAM, data_xport_args);
        _radio_perifs[0].rx_flow_xport   = _fifo_iface->make_send_xport(E300_R0_RX_DATA_STREAM, ctrl_xport_args);

        _radio_perifs[1].send_ctrl_xport = _fifo_iface->make_send_xport(E300_R1_CTRL_STREAM, ctrl_xport_args);
        _radio_perifs[1].recv_ctrl_xport = _fifo_iface->make_recv_xport(E300_R1_CTRL_STREAM, ctrl_xport_args);
        _radio_perifs[1].tx_data_xport   = _fifo_iface->make_send_xport(E300_R1_TX_DATA_STREAM, data_xport_args);
        _radio_perifs[1].tx_flow_xport   = _fifo_iface->make_recv_xport(E300_R1_TX_DATA_STREAM, ctrl_xport_args);
        _radio_perifs[1].rx_data_xport   = _fifo_iface->make_recv_xport(E300_R1_RX_DATA_STREAM, data_xport_args);
        _radio_perifs[1].rx_flow_xport   = _fifo_iface->make_send_xport(E300_R1_RX_DATA_STREAM, ctrl_xport_args);

        _codec_xport = ad9361_ctrl_transport::make_software_spi(AD9361_E300, uhd::usrp::e300::spi::make(E300_SPIDEV_DEVICE), 1);
        _codec_ctrl = ad9361_ctrl::make(_codec_xport);
        // This is horrible ... why do I have to sleep here?
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }

    // Verify we can talk to the e300 core control registers ...
    UHD_MSG(status) << "Initializing core control..." << std::endl;
    this->register_loopback_self_test(_global_regs);

    // TODO: Put this in the right place
    const boost::uint32_t git_hash = _global_regs->peek32(uhd::usrp::e300::global_regs::RB32_CORE_GITHASH);
    const boost::uint32_t compat = _global_regs->peek32(uhd::usrp::e300::global_regs::RB32_CORE_COMPAT);
    UHD_MSG(status) << "Getting version information... " << std::flush;
    UHD_MSG(status) << boost::format("%u.%02d (git %7x%s)")
        % (compat & 0xff) % ((compat & 0xff00) >> 8)
        % (git_hash & 0x0FFFFFFF)
        % ((git_hash & 0xF000000) ? "-dirty" : "") << std::endl;



    ////////////////////////////////////////////////////////////////////
    // optional udp server
    ////////////////////////////////////////////////////////////////////
    if (device_addr.has_key("server"))
    {
        boost::thread_group tg;
        tg.create_thread(boost::bind(&e300_impl::run_server, this, E300_SERVER_RX_PORT, "RX"));
        tg.create_thread(boost::bind(&e300_impl::run_server, this, E300_SERVER_TX_PORT, "TX"));
        tg.create_thread(boost::bind(&e300_impl::run_server, this, E300_SERVER_CTRL_PORT, "CTRL"));
        tg.create_thread(boost::bind(&e300_impl::run_server, this, E300_SERVER_CODEC_PORT, "CODEC"));
        tg.create_thread(boost::bind(&e300_impl::run_server, this, E300_SERVER_GREGS_PORT, "GREGS"));
        tg.join_all();
        goto e300_impl_begin;
    }



    ////////////////////////////////////////////////////////////////////
    // Initialize the properties tree
    ////////////////////////////////////////////////////////////////////
    _tree = property_tree::make();
    _tree->create<std::string>("/name").set("E-Series Device");
    const fs_path mb_path = "/mboards/0";
    _tree->create<std::string>(mb_path / "name").set("E300");
    _tree->create<std::string>(mb_path / "codename").set("Troll");

    ////////////////////////////////////////////////////////////////////
    // and do the misc mboard sensors
    ////////////////////////////////////////////////////////////////////
    _tree->create<int>(mb_path / "sensors"); //empty TODO


    if (!_network_mode) {
        const std::vector<std::string> xadc_sensors = boost::assign::list_of("temp");
        BOOST_FOREACH(const std::string &sensor, xadc_sensors)
        {
            _tree->create<sensor_value_t>(mb_path / "sensors" / sensor)
                .publish(boost::bind(&e300_impl::get_mb_temp, this));
        }
    }

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
        .coerce(boost::bind(&e300_impl::set_tick_rate, this, _1))
        .publish(boost::bind(&e300_impl::get_tick_rate, this))
        .subscribe(boost::bind(&e300_impl::update_tick_rate, this, _1));

    //default some chains on -- needed for setup purposes
    _codec_ctrl->set_active_chains(true, false, true, false);
    _codec_ctrl->set_clock_rate(50e6);

    ////////////////////////////////////////////////////////////////////
    // setup radios
    ////////////////////////////////////////////////////////////////////
    this->setup_radio(0);
    this->setup_radio(1);

    _codec_ctrl->data_port_loopback(true);
    BOOST_FOREACH(radio_perifs_t &_radio_perif, _radio_perifs)
    {
        this->codec_loopback_self_test(_radio_perif.ctrl);
    }
    _codec_ctrl->data_port_loopback(false);

    ////////////////////////////////////////////////////////////////////
    // internal gpios
    ////////////////////////////////////////////////////////////////////
    gpio_core_200::sptr fp_gpio = gpio_core_200::make(_radio_perifs[0].ctrl, TOREG(SR_FP_GPIO), RB32_FP_GPIO);
    const std::vector<std::string> gpio_attrs = boost::assign::list_of("CTRL")("DDR")("OUT")("ATR_0X")("ATR_RX")("ATR_TX")("ATR_XX");
    BOOST_FOREACH(const std::string &attr, gpio_attrs)
    {
        _tree->create<boost::uint32_t>(mb_path / "gpio" / "FP0" / attr)
            .subscribe(boost::bind(&e300_impl::set_internal_gpio, this, fp_gpio, attr, _1))
            .set(0);
    }
    _tree->create<boost::uint8_t>(mb_path / "gpio" / "FP0" / "READBACK")
        .publish(boost::bind(&e300_impl::get_internal_gpio, this, fp_gpio, "READBACK"));


    ////////////////////////////////////////////////////////////////////
    // register the time keepers - only one can be the highlander
    ////////////////////////////////////////////////////////////////////
    _tree->create<time_spec_t>(mb_path / "time" / "now")
        .publish(boost::bind(&time_core_3000::get_time_now, _radio_perifs[0].time64))
        .subscribe(boost::bind(&time_core_3000::set_time_now, _radio_perifs[0].time64, _1))
        .subscribe(boost::bind(&time_core_3000::set_time_now, _radio_perifs[1].time64, _1));
    _tree->create<time_spec_t>(mb_path / "time" / "pps")
        .publish(boost::bind(&time_core_3000::get_time_last_pps, _radio_perifs[0].time64))
        .subscribe(boost::bind(&time_core_3000::set_time_next_pps, _radio_perifs[0].time64, _1))
        .subscribe(boost::bind(&time_core_3000::set_time_next_pps, _radio_perifs[1].time64, _1));
    //setup time source props
    _tree->create<std::string>(mb_path / "time_source" / "value")
        .subscribe(boost::bind(&e300_impl::update_time_source, this, _1));
    static const std::vector<std::string> time_sources = boost::assign::list_of("none")("external")("gpsdo");
    _tree->create<std::vector<std::string> >(mb_path / "time_source" / "options").set(time_sources);
    //setup reference source props
    _tree->create<std::string>(mb_path / "clock_source" / "value")
        .subscribe(boost::bind(&e300_impl::update_clock_source, this, _1));
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
        _tree->create<std::string>(codec_path / "name").set("E300 RX dual ADC");
        _tree->create<int>(codec_path / "gains"); //empty cuz gains are in frontend
    }
    {
        const fs_path codec_path = mb_path / ("tx_codecs") / "A";
        _tree->create<std::string>(codec_path / "name").set("E300 TX dual DAC");
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
            .subscribe(boost::bind(&e300_impl::update_fe_lo_freq, this, fe_name, _1));
        _tree->create<meta_range_t>(rf_fe_path / "freq" / "range")
            .publish(boost::bind(&ad9361_ctrl::get_rf_freq_range));

        //setup antenna stuff
        if (fe_name[0] == 'R')
        {
            static const std::vector<std::string> ants = boost::assign::list_of("TX/RX")("RX2");
            _tree->create<std::vector<std::string> >(rf_fe_path / "antenna" / "options").set(ants);
            _tree->create<std::string>(rf_fe_path / "antenna" / "value")
                .subscribe(boost::bind(&e300_impl::update_antenna_sel, this, fe_name, _1))
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

     std::vector<size_t> default_map(2, 0);
     default_map[0] = 0; // set A->0
     default_map[1] = 1; // set B->1, even if there's only A

    _tree->create<std::vector<size_t> >(mb_path / "rx_chan_dsp_mapping").set(default_map);
    _tree->create<std::vector<size_t> >(mb_path / "tx_chan_dsp_mapping").set(default_map);

    _tree->create<subdev_spec_t>(mb_path / "rx_subdev_spec")
        .set(subdev_spec_t())
        .subscribe(boost::bind(&e300_impl::update_rx_subdev_spec, this, _1));
    _tree->create<subdev_spec_t>(mb_path / "tx_subdev_spec")
        .set(subdev_spec_t())
        .subscribe(boost::bind(&e300_impl::update_tx_subdev_spec, this, _1));

    ////////////////////////////////////////////////////////////////////
    // do some post-init tasks
    ////////////////////////////////////////////////////////////////////

    //init the clock rate to something, but only when we have active chains
    //init the clock rate to something reasonable
    _tree->access<double>(mb_path / "tick_rate").set(
        device_addr.cast<double>("master_clock_rate", E300_DEFAULT_TICK_RATE));
    //_codec_ctrl->set_active_chains(false, false, false, false);

    _tree->access<subdev_spec_t>(mb_path / "rx_subdev_spec").set(subdev_spec_t("A:RX1 A:RX2"));
    _tree->access<subdev_spec_t>(mb_path / "tx_subdev_spec").set(subdev_spec_t("A:TX1 A:TX2"));

    _tree->access<std::string>(mb_path / "clock_source" / "value").set("internal");
    _tree->access<std::string>(mb_path / "time_source" / "value").set("none");

}

boost::uint8_t e300_impl::get_internal_gpio(gpio_core_200::sptr gpio, const std::string &)
{
    return boost::uint32_t(gpio->read_gpio(dboard_iface::UNIT_RX));
}

void e300_impl::set_internal_gpio(gpio_core_200::sptr gpio, const std::string &attr, const boost::uint32_t value)
{
    if (attr == "CTRL")
        return gpio->set_pin_ctrl(dboard_iface::UNIT_RX, value);
    else if (attr == "DDR")
        return gpio->set_gpio_ddr(dboard_iface::UNIT_RX, value);
    else if (attr == "OUT")
        return gpio->set_gpio_out(dboard_iface::UNIT_RX, value);
    else if (attr == "ATR_0X")
        return gpio->set_atr_reg(dboard_iface::UNIT_RX, dboard_iface::ATR_REG_IDLE, value);
    else if (attr == "ATR_RX")
        return gpio->set_atr_reg(dboard_iface::UNIT_RX, dboard_iface::ATR_REG_RX_ONLY, value);
    else if (attr == "ATR_TX")
        return gpio->set_atr_reg(dboard_iface::UNIT_RX, dboard_iface::ATR_REG_TX_ONLY, value);
    else if (attr == "ATR_XX")
        return gpio->set_atr_reg(dboard_iface::UNIT_RX, dboard_iface::ATR_REG_FULL_DUPLEX, value);
}

e300_impl::~e300_impl(void)
{
    /* NOP */
}

double e300_impl::set_tick_rate(const double rate)
{
    const size_t factor = 2; // TODO: This breaks the SISO case, and needs to be reworked
    UHD_MSG(status) << "Asking for clock rate " << rate/1e6 << " MHz\n";
    _tick_rate = _codec_ctrl->set_clock_rate(rate/factor)*factor;
    UHD_MSG(status) << "Actually got clock rate " << _tick_rate/1e6 << " MHz\n";

    BOOST_FOREACH(radio_perifs_t &perif, _radio_perifs)
    {
        perif.time64->set_tick_rate(_tick_rate);
        perif.time64->self_test();
    }
    return _tick_rate;
}

void e300_impl::load_fpga_image(const std::string &path)
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

void e300_impl::register_loopback_self_test(wb_iface::sptr iface)
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

void e300_impl::codec_loopback_self_test(wb_iface::sptr iface)
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

boost::uint32_t e300_impl::allocate_sid(const sid_config_t &config)
{
    const boost::uint32_t stream = (config.dst_prefix | (config.router_dst_there << 2)) & 0xff;

    const boost::uint32_t sid = 0
        | (E300_DEVICE_HERE << 24)
        | (_sid_framer << 16)
        | (config.router_addr_there << 8)
        | (stream << 0)
    ;
    UHD_LOG << std::hex
        << " sid 0x" << sid
        << " framer 0x" << _sid_framer
        << " stream 0x" << stream
        << " router_dst_there 0x" << int(config.router_dst_there)
        << " router_addr_there 0x" << int(config.router_addr_there)
        << std::dec << std::endl;

    // Program the E300 to recognize it's own local address.
    _global_regs->poke32(uhd::usrp::e300::global_regs::SR_CORE_XB_LOCAL, config.router_addr_there);

    // Program CAM entry for outgoing packets matching a E300 resource (e.g. Radio).
    // This type of packet matches the XB_LOCAL address and is looked up in the upper
    // half of the CAM
    _global_regs->poke32(uhd::usrp::e300::XB_ADDR(256 + stream),
                         config.router_dst_there);

    // Program CAM entry for returning packets to us (for example GR host via zynq_fifo)
    // This type of packet does not match the XB_LOCAL address and is looked up in the lower half of the CAM
    _global_regs->poke32(uhd::usrp::e300::XB_ADDR(E300_DEVICE_HERE),
                         config.router_dst_here);

    UHD_LOG << std::hex
        << "done router config for sid 0x" << sid
        << std::dec << std::endl;

    //increment for next setup
    _sid_framer++;

    return sid;
}

void e300_impl::_setup_dest_mapping(const boost::uint32_t sid, const size_t which_stream)
{
    UHD_LOG << boost::format("Setting up dest map for 0x%lx to be stream %d")
                                     % (sid & 0xff) % which_stream << std::endl;
    _global_regs->poke32(uhd::usrp::e300::DST_ADDR(sid & 0xff), which_stream);
}

void e300_impl::update_time_source(const std::string &)
{
}

void e300_impl::update_clock_source(const std::string &)
{
}

void e300_impl::update_antenna_sel(const std::string &fe, const std::string &ant)
{
    const size_t i = (fe == "RX1")? 0 : 1;
    _fe_control_settings[i].rx_ant = ant;
    this->update_atrs(i);
}

void e300_impl::update_fe_lo_freq(const std::string &fe, const double freq)
{
    for (size_t i = 0; i < 2; i++)
    {
        if (fe[0] == 'R') _fe_control_settings[i].rx_freq = freq;
        if (fe[0] == 'T') _fe_control_settings[i].tx_freq = freq;
        this->update_atrs(i);
    }
}

void e300_impl::update_active_frontends(void)
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

void e300_impl::setup_radio(const size_t dspno)
{
    radio_perifs_t &perif = _radio_perifs[dspno];
    const fs_path mb_path = "/mboards/0";

    ////////////////////////////////////////////////////////////////////
    // crossbar config for ctrl xports
    ////////////////////////////////////////////////////////////////////
    sid_config_t config;
    config.router_addr_there = E300_DEVICE_THERE;
    config.dst_prefix        = E300_RADIO_DEST_PREFIX_CTRL;
    config.router_dst_there  = dspno ? E300_XB_DST_R1 : E300_XB_DST_R0;
    config.router_dst_here   = E300_XB_DST_AXI;
    boost::uint32_t ctrl_sid = this->allocate_sid(config);
    this->_setup_dest_mapping(ctrl_sid,
                              dspno ? E300_R1_CTRL_STREAM
                                    : E300_R0_CTRL_STREAM);


    ////////////////////////////////////////////////////////////////////
    // radio control
    ////////////////////////////////////////////////////////////////////
    perif.ctrl = radio_ctrl_core_3000::make(false/*lilE*/,
                                            perif.send_ctrl_xport,
                                            perif.recv_ctrl_xport,
                                            ctrl_sid,
                                            dspno ? "1" : "0"
                                            );
    this->register_loopback_self_test(perif.ctrl);
    perif.atr = gpio_core_200_32wo::make(perif.ctrl, TOREG(SR_GPIO));

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
        .subscribe(boost::bind(&e300_impl::update_rx_samp_rate, this, dspno, _1))
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
        .subscribe(boost::bind(&e300_impl::update_tx_samp_rate, this, dspno, _1))
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

uhd::sensor_value_t e300_impl::get_mb_temp(void)
{
    double scale = boost::lexical_cast<double>(e300_get_sysfs_attr(E300_TEMP_SYSFS, "in_temp0_scale"));
    unsigned long raw = boost::lexical_cast<unsigned long>(e300_get_sysfs_attr(E300_TEMP_SYSFS, "in_temp0_raw"));
    unsigned long offset = boost::lexical_cast<unsigned long>(e300_get_sysfs_attr(E300_TEMP_SYSFS, "in_temp0_offset"));
    return sensor_value_t("temp", (raw + offset) * scale / 1e3, "C");
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
//////////////// ATR SETUP FOR FRONTEND CONTROL VIA GPIO ///////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void e300_impl::update_atrs(const size_t &fe)
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
        | ((settings.rx_enb? 1 : 0) << ST_RX_ENABLE)
        | ((settings.tx_enb? 1 : 0) << ST_TX_ENABLE)
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
    gpio_core_200_32wo::sptr atr = _radio_perifs[fe].atr;
    atr->set_atr_reg(dboard_iface::ATR_REG_IDLE, oo_reg);
    atr->set_atr_reg(dboard_iface::ATR_REG_RX_ONLY, rx_reg);
    atr->set_atr_reg(dboard_iface::ATR_REG_TX_ONLY, tx_reg);
    atr->set_atr_reg(dboard_iface::ATR_REG_FULL_DUPLEX, fd_reg);
}
