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
#include <uhd/types/sensors.hpp>
#include <boost/filesystem.hpp>
#include <boost/functional/hash.hpp>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>
#include <fstream>

using namespace uhd;
using namespace uhd::usrp;
namespace fs = boost::filesystem;

/***********************************************************************
 * Discovery
 **********************************************************************/
static device_addrs_t e200_find(const device_addr_t &hint)
{
    device_addrs_t e200_addrs;

    //return an empty list of addresses when type is set to non-e200
    if (hint.has_key("type") and hint["type"] != "e200") return e200_addrs;

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
    const size_t dspno = 0; //FIXME just one for now

    ////////////////////////////////////////////////////////////////////
    // Initialize the properties tree
    ////////////////////////////////////////////////////////////////////
    _tree = property_tree::make();
    _tree->create<std::string>("/name").set("E-Series Device");
    const fs_path mb_path = "/mboards/0";
    _tree->create<std::string>(mb_path / "name").set("uSerp");
    _tree->create<std::string>(mb_path / "codename").set("");

    ////////////////////////////////////////////////////////////////////
    // clocking
    ////////////////////////////////////////////////////////////////////
    _tree->create<double>(mb_path / "tick_rate").set(E200_RADIO_CLOCK_RATE);

    ////////////////////////////////////////////////////////////////////
    // load the fpga image
    ////////////////////////////////////////////////////////////////////
    //extract the FPGA path for the E200
    const std::string e200_fpga_image = find_image_path(
        device_addr.get("fpga", E200_FPGA_FILE_NAME)
    );

    //load fpga image - its super fast
    this->load_fpga_image(e200_fpga_image);

    ////////////////////////////////////////////////////////////////////
    // create fifo interface
    ////////////////////////////////////////////////////////////////////
    _fifo_iface = e200_fifo_interface::make(e200_read_sysfs());

    ////////////////////////////////////////////////////////////////////
    // radio control
    ////////////////////////////////////////////////////////////////////
    uhd::device_addr_t xport_args;
    uhd::transport::zero_copy_if::sptr send_ctrl_xport = _fifo_iface->make_send_xport(1, xport_args);
    uhd::transport::zero_copy_if::sptr recv_ctrl_xport = _fifo_iface->make_recv_xport(1, xport_args);
    _radio_ctrl = e200_ctrl::make(send_ctrl_xport, recv_ctrl_xport, 1 | (1 << 16));
    this->register_loopback_self_test(_radio_ctrl);

    ////////////////////////////////////////////////////////////////////
    // radio data xports
    ////////////////////////////////////////////////////////////////////
    //_tx_data_xport = _fifo_iface->make_send_xport(0, xport_args);
    //_tx_flow_xport = _fifo_iface->make_recv_xport(0, xport_args);
    _rx_data_xport = _fifo_iface->make_recv_xport(2, xport_args);
    _rx_flow_xport = _fifo_iface->make_send_xport(2, xport_args);

    ////////////////////////////////////////////////////////////////////
    // create rx dsp control objects
    ////////////////////////////////////////////////////////////////////
    _rx_framer = rx_vita_core_3000::make(_radio_ctrl, TOREG(SR_RX_CTRL+4), TOREG(SR_RX_CTRL));
    _rx_framer->set_tick_rate(E200_RADIO_CLOCK_RATE);
    _rx_dsp = rx_dsp_core_3000::make(_radio_ctrl, TOREG(SR_RX_DSP));
    _rx_dsp->set_link_rate(10e9/8); //whatever
    _rx_dsp->set_tick_rate(E200_RADIO_CLOCK_RATE);
    const fs_path rx_dsp_path = mb_path / "rx_dsps" / str(boost::format("%u") % dspno);
    _tree->create<meta_range_t>(rx_dsp_path / "rate" / "range")
        .publish(boost::bind(&rx_dsp_core_3000::get_host_rates, _rx_dsp));
    _tree->create<double>(rx_dsp_path / "rate" / "value")
        .coerce(boost::bind(&rx_dsp_core_3000::set_host_rate, _rx_dsp, _1))
        //.subscribe(boost::bind(&e200_impl::update_rx_samp_rate, this, dspno, _1))
        .set(1e6);
    _tree->create<double>(rx_dsp_path / "freq" / "value")
        .coerce(boost::bind(&rx_dsp_core_3000::set_freq, _rx_dsp, _1))
        .set(0.0);
    _tree->create<meta_range_t>(rx_dsp_path / "freq" / "range")
        .publish(boost::bind(&rx_dsp_core_3000::get_freq_range, _rx_dsp));
    _tree->create<stream_cmd_t>(rx_dsp_path / "stream_cmd")
        .subscribe(boost::bind(&rx_vita_core_3000::issue_stream_command, _rx_framer, _1));

    ////////////////////////////////////////////////////////////////////
    // create tx dsp control objects
    ////////////////////////////////////////////////////////////////////
    _tx_deframer = tx_vita_core_3000::make(_radio_ctrl, TOREG(SR_TX_CTRL+2), TOREG(SR_TX_CTRL));
    _tx_deframer->set_tick_rate(E200_RADIO_CLOCK_RATE);
    _tx_dsp = tx_dsp_core_3000::make(_radio_ctrl, TOREG(SR_TX_DSP));
    _tx_dsp->set_link_rate(10e9/8); //whatever
    _tx_dsp->set_tick_rate(E200_RADIO_CLOCK_RATE);
    const fs_path tx_dsp_path = mb_path / "tx_dsps" / str(boost::format("%u") % dspno);
    _tree->create<meta_range_t>(tx_dsp_path / "rate" / "range")
        .publish(boost::bind(&tx_dsp_core_3000::get_host_rates, _tx_dsp));
    _tree->create<double>(tx_dsp_path / "rate" / "value")
        .coerce(boost::bind(&tx_dsp_core_3000::set_host_rate, _tx_dsp, _1))
        //.subscribe(boost::bind(&e200_impl::update_tx_samp_rate, this, dspno, _1))
        .set(1e6);
    _tree->create<double>(tx_dsp_path / "freq" / "value")
        .coerce(boost::bind(&tx_dsp_core_3000::set_freq, _tx_dsp, _1))
        .set(0.0);
    _tree->create<meta_range_t>(tx_dsp_path / "freq" / "range")
        .publish(boost::bind(&tx_dsp_core_3000::get_freq_range, _tx_dsp));

    ////////////////////////////////////////////////////////////////////
    // create time control objects
    ////////////////////////////////////////////////////////////////////
    time_core_3000::readback_bases_type time64_rb_bases;
    time64_rb_bases.rb_now = RB64_TIME_NOW;
    time64_rb_bases.rb_pps = RB64_TIME_PPS;
    _time64 = time_core_3000::make(_radio_ctrl, TOREG(SR_TIME), time64_rb_bases);
    _time64->set_tick_rate(E200_RADIO_CLOCK_RATE);
    _time64->self_test();
    _tree->create<time_spec_t>(mb_path / "time" / "now")
        .publish(boost::bind(&time_core_3000::get_time_now, _time64))
        .subscribe(boost::bind(&time_core_3000::set_time_now, _time64, _1));
    _tree->create<time_spec_t>(mb_path / "time" / "pps")
        .publish(boost::bind(&time_core_3000::get_time_last_pps, _time64))
        .subscribe(boost::bind(&time_core_3000::set_time_next_pps, _time64, _1));
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
    BOOST_FOREACH(const std::string &fe_name, frontends)
    {
        const std::string x = std::string(1, tolower(fe_name[0]));
        const fs_path rf_fe_path = mb_path / "dboards" / "A" / (x+"x_frontends") / fe_name;

        _tree->create<std::string>(rf_fe_path / "name").set(fe_name);
        _tree->create<int>(rf_fe_path / "sensors"); //empty TODO
        _tree->create<sensor_value_t>(rf_fe_path / "sensors" / "lo_locked");
        /*empty for temps*/ _tree->create<int>(rf_fe_path / "gains");
        /*
        BOOST_FOREACH(const std::string &name, _codec_ctrl->get_gain_names(fe_name))
        {
            _tree->create<meta_range_t>(rf_fe_path / "gains" / name / "range")
                    ;//.set(_codec_ctrl->get_gain_range(fe_name));

            _tree->create<double>(rf_fe_path / "gains" / name / "value")
                //.coerce(boost::bind(&b200_codec_ctrl::set_gain, _codec_ctrl, fe_name, _1))
                .set(0.0);
        }
        */
        _tree->create<std::string>(rf_fe_path / "connection").set("IQ");
        _tree->create<bool>(rf_fe_path / "enabled").set(true);
        _tree->create<bool>(rf_fe_path / "use_lo_offset").set(false);
        _tree->create<double>(rf_fe_path / "bandwidth" / "value")
            //.coerce(boost::bind(&b200_codec_ctrl::set_bw_filter, _codec_ctrl, fe_name, _1))
            .set(40e6);
        _tree->create<meta_range_t>(rf_fe_path / "bandwidth" / "range")
            ;//.publish(boost::bind(&b200_codec_ctrl::get_bw_filter_range, _codec_ctrl, fe_name));
        _tree->create<double>(rf_fe_path / "freq" / "value")
            //.coerce(boost::bind(&b200_codec_ctrl::tune, _codec_ctrl, fe_name, _1))
            ;//.subscribe(boost::bind(&b200_impl::update_bandsel, this, fe_name, _1));
        _tree->create<meta_range_t>(rf_fe_path / "freq" / "range")
            ;//.publish(boost::bind(&b200_codec_ctrl::get_rf_freq_range, _codec_ctrl, fe_name));

        //setup antenna stuff
        if (fe_name[0] == 'R')
        {
            static const std::vector<std::string> ants = boost::assign::list_of("TX/RX")("RX2");
            _tree->create<std::vector<std::string> >(rf_fe_path / "antenna" / "options").set(ants);
            _tree->create<std::string>(rf_fe_path / "antenna" / "value")
                //.subscribe(boost::bind(&b200_impl::update_antenna_sel, this, fe_name, _1))
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
    _tree->access<std::string>(mb_path / "clock_source" / "value").set("internal");
    _tree->access<std::string>(mb_path / "time_source" / "value").set("none");

}

e200_impl::~e200_impl(void)
{
    /* NOP */
}

void e200_impl::load_fpga_image(const std::string &path)
{
    if (not fs::exists("/dev/xdevcfg"))
    {
        ::system("mknod /dev/xdevcfg c 259 0");
        //throw uhd::runtime_error("no xdevcfg, please run: mknod /dev/xdevcfg c 259 0");
    }

    UHD_MSG(status) << "Loading FPGA image: " << path << "..." << std::flush;

    /*
    std::ifstream fpga_file(path.c_str(), std::ios_base::binary);
    UHD_ASSERT_THROW(fpga_file.good());

    std::ofstream xdev_file("/dev/xdevcfg", std::ios_base::binary);
    UHD_ASSERT_THROW(xdev_file.good());

    char buff[(1 << 22)]; //big enough for entire image
    fpga_file.read(buff, sizeof(buff));
    xdev_file.write(buff, fpga_file.gcount());

    fpga_file.close();
    xdev_file.close();
    //*/

    //cat works, fstream blows
    const std::string cmd = str(boost::format("cat \"%s\" > /dev/xdevcfg") % path);
    ::system(cmd.c_str());

    UHD_MSG(status) << " done" << std::endl;
}

//sshfs jblum@blarg:/home/jblum/src src
//utils/uhd_usrp_probe --tree --args="fpga=/home/root/b200_xport_t0.bin,type=e200"
//utils/uhd_usrp_probe --tree --args="fpga=/home/root/e200_xport_t1.bin,type=e200"

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

void e200_impl::update_time_source(const std::string &)
{
    
}

void e200_impl::update_clock_source(const std::string &)
{
    
}
