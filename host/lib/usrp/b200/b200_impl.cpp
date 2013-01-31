//
// Copyright 2012 Ettus Research LLC
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

#include "b200_impl.hpp"
#include "b200_regs.hpp"
#include <uhd/transport/usb_control.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/images.hpp>
#include <uhd/utils/safe_call.hpp>
#include <boost/format.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/functional/hash.hpp>
#include <cstdio>
#include <iomanip>
#include <ctime>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;

//const boost::uint16_t B200_VENDOR_ID  = 0x2500;
//const boost::uint16_t B200_PRODUCT_ID = 0x0003;

const boost::uint16_t B200_VENDOR_ID  = 0x04b4;
const boost::uint16_t FX3_PRODUCT_ID = 0x00f3;
const boost::uint16_t B200_PRODUCT_ID = 0x00f0;
static const boost::posix_time::milliseconds REENUMERATION_TIMEOUT_MS(3000);

/***********************************************************************
 * Discovery
 **********************************************************************/
static device_addrs_t b200_find(const device_addr_t &hint)
{
    device_addrs_t b200_addrs;

    //return an empty list of addresses when type is set to non-b200
    if (hint.has_key("type") and hint["type"] != "b200") return b200_addrs;

    //Return an empty list of addresses when an address is specified,
    //since an address is intended for a different, non-USB, device.
    if (hint.has_key("addr")) return b200_addrs;

    unsigned int vid, pid;

    if(hint.has_key("vid") && hint.has_key("pid") && hint.has_key("type") && hint["type"] == "b200") {
        sscanf(hint.get("vid").c_str(), "%x", &vid);
        sscanf(hint.get("pid").c_str(), "%x", &pid);
    } else {
        vid = B200_VENDOR_ID;
        pid = FX3_PRODUCT_ID;
    }

    // Important note:
    // The get device list calls are nested inside the for loop.
    // This allows the usb guts to decontruct when not in use,
    // so that re-enumeration after fw load can occur successfully.
    // This requirement is a courtesy of libusb1.0 on windows.

    //find the usrps and load firmware
    size_t found = 0;
    BOOST_FOREACH(usb_device_handle::sptr handle, usb_device_handle::get_device_list(vid, pid)) {
        //extract the firmware path for the b200
        std::string b200_fw_image;
        try{
            b200_fw_image = find_image_path(hint.get("fw", B200_FW_FILE_NAME));
        }
        catch(...){
            UHD_MSG(warning) << boost::format(
                "Could not locate B200 firmware.\n"
                "Please install the images package.\n"
            );
            return b200_addrs;
        }
        UHD_LOG << "the firmware image: " << b200_fw_image << std::endl;

        usb_control::sptr control;
        try{control = usb_control::make(handle, 0);}
        catch(const uhd::exception &){continue;} //ignore claimed

        b200_iface::make(control)->load_firmware(b200_fw_image);

        found++;
    }

    //get descriptors again with serial number, but using the initialized VID/PID now since we have firmware
    //TODO
    found = 1;
    vid = B200_VENDOR_ID;
    pid = B200_PRODUCT_ID;

    const boost::system_time timeout_time = boost::get_system_time() + REENUMERATION_TIMEOUT_MS;

    //search for the device until found or timeout
    while (boost::get_system_time() < timeout_time and b200_addrs.empty() and found != 0)
    {
        BOOST_FOREACH(usb_device_handle::sptr handle, usb_device_handle::get_device_list(vid, pid))
        {
            usb_control::sptr control;
            try{control = usb_control::make(handle, 0);}
            catch(const uhd::exception &){continue;} //ignore claimed

            b200_iface::sptr iface = b200_iface::make(control);
            //TODO
            //const mboard_eeprom_t mb_eeprom = mboard_eeprom_t(*iface, mboard_eeprom_t::MAP_B100);

            device_addr_t new_addr;
            new_addr["type"] = "b200";
            //TODO
            //new_addr["name"] = mb_eeprom["name"];
            new_addr["serial"] = handle->get_serial();
            //this is a found b200 when the hint serial and name match or blank
            if (
                (not hint.has_key("name")   or hint["name"]   == new_addr["name"]) and
                (not hint.has_key("serial") or hint["serial"] == new_addr["serial"])
            ){
                b200_addrs.push_back(new_addr);
            }
        }
    }

    return b200_addrs;
}

/***********************************************************************
 * Make
 **********************************************************************/
static device::sptr b200_make(const device_addr_t &device_addr)
{
    return device::sptr(new b200_impl(device_addr));
}

UHD_STATIC_BLOCK(register_b200_device)
{
    device::register_device(&b200_find, &b200_make);
}

/***********************************************************************
 * Structors
 **********************************************************************/
b200_impl::b200_impl(const device_addr_t &device_addr)
{
    //extract the FPGA path for the B200
    //TODO
    /*
    std::string b200_fpga_image = find_image_path(
        device_addr.has_key("fpga")? device_addr["fpga"] : B200_FPGA_FILE_NAME
    );
    */


    _tree = property_tree::make();

    //try to match the given device address with something on the USB bus
    std::vector<usb_device_handle::sptr> device_list =
        usb_device_handle::get_device_list(B200_VENDOR_ID, B200_PRODUCT_ID);

    //locate the matching handle in the device list
    usb_device_handle::sptr handle;
    BOOST_FOREACH(usb_device_handle::sptr dev_handle, device_list) {
        if (dev_handle->get_serial() == device_addr["serial"]){
            handle = dev_handle;
            break;
        }
    }
    UHD_ASSERT_THROW(handle.get() != NULL); //better be found

    //create control objects
    usb_control::sptr control = usb_control::make(handle, 0);
    _iface = b200_iface::make(control);
    this->check_fw_compat(); //check after making

    ////////////////////////////////////////////////////////////////////
    // Load the FPGA image - init GPIF - hold in reset
    ////////////////////////////////////////////////////////////////////
    _iface->reset_fpga(true);
    //TODO load the FPGA....
    _iface->reset_fx3();

    ////////////////////////////////////////////////////////////////////
    // Get the FPGA a clock from Catalina
    ////////////////////////////////////////////////////////////////////
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    //_iface->write_reg(0x00A, BOOST_BINARY( 00000010 ));//no clock
    _iface->write_reg(0x00A, BOOST_BINARY( 00010010 )); //yes clock
    _iface->write_reg(0x009, BOOST_BINARY( 00010111 ));

    _iface->reset_fpga(false); //bring it out of reset
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));

    ////////////////////////////////////////////////////////////////////
    // Create control transport
    ////////////////////////////////////////////////////////////////////
    boost::uint8_t usb_speed = _iface->get_usb_speed();
    UHD_MSG(status) << "Operating over USB " << (int) usb_speed << "." << std::endl;

    device_addr_t ctrl_xport_args;
    ctrl_xport_args["recv_frame_size"] = (usb_speed == 3) ? "1024" : "512";
    ctrl_xport_args["num_recv_frames"] = "16";
    ctrl_xport_args["send_frame_size"] = (usb_speed == 3) ? "1024" : "512";
    ctrl_xport_args["num_send_frames"] = "16";

    _ctrl_transport = usb_zero_copy::make(
        handle,
        4, 8, //interface, endpoint
        3, 4, //interface, endpoint
        ctrl_xport_args
    );
    while (_ctrl_transport->get_recv_buff(0.0)){} //flush ctrl xport

    ////////////////////////////////////////////////////////////////////
    // Initialize the properties tree
    ////////////////////////////////////////////////////////////////////
    _tree->create<std::string>("/name").set("B-Series Device");
    const fs_path mb_path = "/mboards/0";
    _tree->create<std::string>(mb_path / "name").set("B200");
    _tree->create<std::string>(mb_path / "codename").set("Sasquatch");

    ////////////////////////////////////////////////////////////////////
    // Initialize control (settings regs and async messages)
    ////////////////////////////////////////////////////////////////////
    _ctrl = b200_ctrl::make(_ctrl_transport);
    _tree->create<time_spec_t>(mb_path / "time/cmd")
        .subscribe(boost::bind(&b200_ctrl::set_time, _ctrl, _1));
    /*
    this->check_fpga_compat(); //check after making
    */

    //Perform readback tests, these tests also write the hash
    bool test_fail = false;
    UHD_MSG(status) << "Performing control readback test... " << std::flush;
    size_t hash = time(NULL);
    for (size_t i = 0; i < 100; i++)
    {
        boost::hash_combine(hash, i);
        _ctrl->poke32(TOREG(SR_TEST), boost::uint32_t(hash));
        test_fail = _ctrl->peek32(TOREG(0*2)) != boost::uint32_t(hash);
        if (test_fail) break; //exit loop on any failure
    }
    UHD_MSG(status) << ((test_fail)? " fail" : "pass") << std::endl;

    ////////////////////////////////////////////////////////////////////
    // Create data transport
    // This happens after FPGA ctrl instantiated so any junk that might
    // be in the FPGAs buffers doesn't get pulled into the transport
    // before being cleared.
    ////////////////////////////////////////////////////////////////////
    device_addr_t data_xport_args;
    data_xport_args["recv_frame_size"] = device_addr.get("recv_frame_size", "2048");
    data_xport_args["num_recv_frames"] = device_addr.get("num_recv_frames", "16");
    data_xport_args["send_frame_size"] = device_addr.get("send_frame_size", "2048");
    data_xport_args["num_send_frames"] = device_addr.get("num_send_frames", "16");

    _data_transport = usb_zero_copy::make(
        handle,        // identifier
        2, 6,          // IN interface, endpoint
        1, 2,          // OUT interface, endpoint
        data_xport_args    // param hints
    );

    ////////////////////////////////////////////////////////////////////
    // setup the mboard eeprom
    ////////////////////////////////////////////////////////////////////
    // TODO
//    const mboard_eeprom_t mb_eeprom(*_iface, mboard_eeprom_t::MAP_B100);
    const mboard_eeprom_t mb_eeprom;
    _tree->create<mboard_eeprom_t>(mb_path / "eeprom")
        .set(mb_eeprom)
        .subscribe(boost::bind(&b200_impl::set_mb_eeprom, this, _1));

    ////////////////////////////////////////////////////////////////////
    // create gpio and misc controls
    ////////////////////////////////////////////////////////////////////
    _atr0 = gpio_core_200_32wo::make(_ctrl, TOREG(SR_ATR0));
    _atr1 = gpio_core_200_32wo::make(_ctrl, TOREG(SR_ATR1));

    _gpio_state = gpio_state(); //clear all to zero

    //set bandsel switches to A band fixed for now
    //assign { /* 2'bX, */ ext_ref_enable, dac_shdn, pps_fpga_out_enable, pps_gps_out_enable, gps_out_enable, gps_ref_enable } = misc_outs[23:16];
    //assign { /* 3'bX, */ tx_bandsel_a, tx_bandsel_b, rx_bandsel_a, rx_bandsel_b, rx_bandsel_c } = misc_outs[15:8];
    //assign { /* 7'bX, */ mimo } = misc_outs[7:0];

    /* TODO lock to ext ref
    _gpio_state.gps_ref_enable = 1;
    _gpio_state.gps_out_enable = 1;
    _gpio_state.ext_ref_enable = 0;
    */
    update_gpio_state(); //first time init

    //set ATR for FDX operation fixed GPIO, amps enabled, LEDs all on
    //set all ATR to GPIO not ATR
    //assign {tx_enable1, SFDX1_RX, SFDX1_TX, SRX1_RX, SRX1_TX, LED_RX1, LED_TXRX1_RX, LED_TXRX1_TX} = atr0[23:16];
    //assign {tx_enable2, SFDX2_RX, SFDX2_TX, SRX2_RX, SRX2_TX, LED_RX2, LED_TXRX2_RX, LED_TXRX2_TX} = atr1[23:16];
    //assign {codec_txrx, codec_en_agc, codec_ctrl_in[3:0] } = atr0[5:0];


    //_atr1->set_atr_reg(dboard_iface::ATR_REG_IDLE, STATE_OFF);
    //_atr1->set_atr_reg(dboard_iface::ATR_REG_TX_ONLY, STATE_TX);
    //_atr1->set_atr_reg(dboard_iface::ATR_REG_FULL_DUPLEX, STATE_FDX);

    ////////////////////////////////////////////////////////////////////
    // create codec control objects
    ////////////////////////////////////////////////////////////////////
    _codec_ctrl = b200_codec_ctrl::make(_iface);
    static const std::vector<std::string> frontends = boost::assign::list_of
        ("TX_A")("TX_B")("RX_A")("RX_B");

    BOOST_FOREACH(const std::string &fe_name, frontends)
    {
        const std::string x = std::string(1, tolower(fe_name[0]));
        const std::string y = std::string(1, fe_name[3]);
        const fs_path codec_path = mb_path / (x+"x_codecs") / y;
        _tree->create<std::string>(codec_path / "name").set("B200 " + fe_name + " CODEC");
        _tree->create<int>(codec_path / "gains"); //empty cuz gains are in frontend
    }

    ////////////////////////////////////////////////////////////////////
    // create clock control objects
    ////////////////////////////////////////////////////////////////////
    //^^^ clock created up top, just reg props here... ^^^
    _tree->create<double>(mb_path / "tick_rate")
        .coerce(boost::bind(&b200_codec_ctrl::set_clock_rate, _codec_ctrl, _1))
        .subscribe(boost::bind(&b200_impl::update_tick_rate, this, _1));

    ////////////////////////////////////////////////////////////////////
    // and do the misc mboard sensors
    ////////////////////////////////////////////////////////////////////
    _tree->create<int>(mb_path / "sensors"); //empty but path exists TODO

    ////////////////////////////////////////////////////////////////////
    // create frontend mapping
    ////////////////////////////////////////////////////////////////////
    _tree->create<subdev_spec_t>(mb_path / "rx_subdev_spec")
        .subscribe(boost::bind(&b200_impl::update_rx_subdev_spec, this, _1));
    _tree->create<subdev_spec_t>(mb_path / "tx_subdev_spec")
        .subscribe(boost::bind(&b200_impl::update_tx_subdev_spec, this, _1));

    ////////////////////////////////////////////////////////////////////
    // create rx dsp control objects
    ////////////////////////////////////////////////////////////////////
    //_rx_dsps.resize(B200_NUM_RX_FE);
    for (size_t dspno = 0; dspno < B200_NUM_RX_FE; dspno++)
    {
        const fs_path rx_dsp_path = mb_path / str(boost::format("rx_dsps/%u") % dspno);
        //_rx_dsps[dspno] = rx_dsp_core_200::make(_ctrl, TOREG(SR_RX_DSP(dspno)), TOREG(SR_RX_CTRL(dspno)), B200_RX_SID_BASE + dspno);
        //_rx_dsps[dspno]->set_link_rate(B200_LINK_RATE_BPS);
        //_tree->access<double>(mb_path / "tick_rate")
        //    .subscribe(boost::bind(&rx_dsp_core_200::set_tick_rate, _rx_dsps[dspno], _1));
        _tree->create<meta_range_t>(rx_dsp_path / "rate/range");
        //TODO get rates from CAT
        //    .publish(boost::bind(&rx_dsp_core_200::get_host_rates, _rx_dsps[dspno]));
        _tree->create<double>(rx_dsp_path / "rate/value")
            .set(1e6) //some default
            //TODO get from CAT or CACHE
            //.coerce(boost::bind(&rx_dsp_core_200::set_host_rate, _rx_dsps[dspno], _1))
            .subscribe(boost::bind(&b200_impl::update_rx_samp_rate, this, dspno, _1));
        _tree->create<double>(rx_dsp_path / "freq/value");
            //TODO always readback 0
            //.coerce(boost::bind(&rx_dsp_core_200::set_freq, _rx_dsps[dspno], _1));
        _tree->create<meta_range_t>(rx_dsp_path / "freq/range");
            //TODO always readback 0, 0
            //.publish(boost::bind(&rx_dsp_core_200::get_freq_range, _rx_dsps[dspno]));
        _tree->create<stream_cmd_t>(rx_dsp_path / "stream_cmd");
            //TODO new stream command module...
            //.subscribe(boost::bind(&rx_dsp_core_200::issue_stream_command, _rx_dsps[dspno], _1));
    }

    ////////////////////////////////////////////////////////////////////
    // create tx dsp control objects
    ////////////////////////////////////////////////////////////////////
    //_tx_dsps.resize(B200_NUM_TX_FE);
    for (size_t dspno = 0; dspno < B200_NUM_TX_FE; dspno++)
    {
        const fs_path tx_dsp_path = mb_path / str(boost::format("tx_dsps/%u") % dspno);
        //_tx_dsps[dspno] = tx_dsp_core_200::make(_ctrl, TOREG(SR_TX_DSP(dspno)), TOREG(SR_TX_CTRL(dspno)), B200_ASYNC_SID_BASE + dspno);
        //_tx_dsps[dspno]->set_link_rate(B200_LINK_RATE_BPS);
        //_tree->access<double>(mb_path / "tick_rate")
        //    .subscribe(boost::bind(&tx_dsp_core_200::set_tick_rate, _tx_dsps[dspno], _1));
        _tree->create<meta_range_t>(tx_dsp_path / "rate/range");
        //TODO get rates from CAT
        //    .publish(boost::bind(&tx_dsp_core_200::get_host_rates, _tx_dsps[dspno]));
        _tree->create<double>(tx_dsp_path / "rate/value")
            .set(1e6) //some default
            //TODO get from CAT or CACHE
            //.coerce(boost::bind(&tx_dsp_core_200::set_host_rate, _tx_dsps[dspno], _1))
            .subscribe(boost::bind(&b200_impl::update_tx_samp_rate, this, 0, _1));
        _tree->create<double>(tx_dsp_path / "freq/value");
            //TODO always readback 0
            //.coerce(boost::bind(&tx_dsp_core_200::set_freq, _tx_dsps[dspno], _1));
        _tree->create<meta_range_t>(tx_dsp_path / "freq/range");
            //TODO always readback 0, 0
            //.publish(boost::bind(&tx_dsp_core_200::get_freq_range, _tx_dsps[dspno]));
    }

    ////////////////////////////////////////////////////////////////////
    // create time control objects
    ////////////////////////////////////////////////////////////////////
    time64_core_200::readback_bases_type time64_rb_bases;
    time64_rb_bases.rb_hi_now = 0;
    time64_rb_bases.rb_lo_now = 0;
    time64_rb_bases.rb_hi_pps = 0;
    time64_rb_bases.rb_lo_pps = 0;
    _time64 = time64_core_200::make(_ctrl, TOREG(0/*TODO*/), time64_rb_bases);
    _tree->access<double>(mb_path / "tick_rate")
        .subscribe(boost::bind(&time64_core_200::set_tick_rate, _time64, _1));
    _tree->create<time_spec_t>(mb_path / "time/now")
        .publish(boost::bind(&time64_core_200::get_time_now, _time64))
        .subscribe(boost::bind(&time64_core_200::set_time_now, _time64, _1));
    _tree->create<time_spec_t>(mb_path / "time/pps")
        .publish(boost::bind(&time64_core_200::get_time_last_pps, _time64))
        .subscribe(boost::bind(&time64_core_200::set_time_next_pps, _time64, _1));
    //setup time source props
    _tree->create<std::string>(mb_path / "time_source/value")
        .subscribe(boost::bind(&time64_core_200::set_time_source, _time64, _1));
    _tree->create<std::vector<std::string> >(mb_path / "time_source/options")
        .publish(boost::bind(&time64_core_200::get_time_sources, _time64));
    //setup reference source props
    _tree->create<std::string>(mb_path / "clock_source/value")
        .subscribe(boost::bind(&b200_impl::update_clock_source, this, _1));
    static const std::vector<std::string> clock_sources = boost::assign::list_of("internal")("external");
    _tree->create<std::vector<std::string> >(mb_path / "clock_source/options").set(clock_sources);

    ////////////////////////////////////////////////////////////////////
    // create RF frontend interfacing
    ////////////////////////////////////////////////////////////////////
    BOOST_FOREACH(const std::string &fe_name, frontends)
    {
        const std::string x = std::string(1, tolower(fe_name[0]));
        const std::string y = std::string(1, fe_name[3]);
        const fs_path rf_fe_path = mb_path / "dboards" / "A" / (x+"x_frontends") / y;

        _tree->create<std::string>(rf_fe_path / "name").set(fe_name);
        _tree->create<int>(rf_fe_path / "sensors"); //empty TODO
        BOOST_FOREACH(const std::string &name, _codec_ctrl->get_gain_names(fe_name))
        {
            _tree->create<meta_range_t>(rf_fe_path / "gains" / name / "range")
                    .set(_codec_ctrl->get_gain_range(fe_name, name));

            _tree->create<double>(rf_fe_path / "gains" / name / "value")
                .coerce(boost::bind(&b200_codec_ctrl::set_gain, _codec_ctrl, fe_name, name, _1))
                .set(0.0);
        }
        _tree->create<std::string>(rf_fe_path / "connection").set("IQ");
        _tree->create<bool>(rf_fe_path / "enabled").set(true);
        _tree->create<bool>(rf_fe_path / "use_lo_offset").set(false);
        _tree->create<double>(rf_fe_path / "bandwidth" / "value")
            .coerce(boost::bind(&b200_codec_ctrl::set_bw_filter, _codec_ctrl, fe_name, _1))
            .set(40e6);
        _tree->create<meta_range_t>(rf_fe_path / "bandwidth" / "range")
            .publish(boost::bind(&b200_codec_ctrl::get_bw_filter_range, _codec_ctrl, fe_name));
        _tree->create<double>(rf_fe_path / "freq" / "value")
            .coerce(boost::bind(&b200_codec_ctrl::tune, _codec_ctrl, fe_name, _1))
            .subscribe(boost::bind(&b200_impl::update_bandsel, this, fe_name, _1));
        _tree->create<meta_range_t>(rf_fe_path / "freq" / "range")
            .publish(boost::bind(&b200_codec_ctrl::get_rf_freq_range, _codec_ctrl, fe_name));

        //setup antenna stuff
        if (fe_name[0] == 'R')
        {
            static const std::vector<std::string> ants = boost::assign::list_of("TX/RX")("RX2");
            _tree->create<std::vector<std::string> >(rf_fe_path / "antenna" / "options").set(ants);
            _tree->create<std::string>(rf_fe_path / "antenna" / "value")
                .subscribe(boost::bind(&b200_impl::update_antenna_sel, this, fe_name, _1))
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
    // do some post-init tasks
    ////////////////////////////////////////////////////////////////////

    //allocate streamer weak ptrs containers
    _rx_streamers.resize(B200_NUM_RX_FE);
    _tx_streamers.resize(B200_NUM_TX_FE);

    _tree->access<double>(mb_path / "tick_rate") //now subscribe the clock rate setter
        .subscribe(boost::bind(&b200_ctrl::set_tick_rate, _ctrl, _1))
        .set(15.36e6);

    this->update_rates();

    //reset cordic rates and their properties to zero
    BOOST_FOREACH(const std::string &name, _tree->list(mb_path / "rx_dsps"))
    {
        _tree->access<double>(mb_path / "rx_dsps" / name / "freq" / "value").set(0.0);
    }
    BOOST_FOREACH(const std::string &name, _tree->list(mb_path / "tx_dsps"))
    {
        _tree->access<double>(mb_path / "tx_dsps" / name / "freq" / "value").set(0.0);
    }

    _tree->access<subdev_spec_t>(mb_path / "rx_subdev_spec").set(subdev_spec_t("A:" + _tree->list(mb_path / "dboards/A/rx_frontends").at(0)));
    _tree->access<subdev_spec_t>(mb_path / "tx_subdev_spec").set(subdev_spec_t("A:" + _tree->list(mb_path / "dboards/A/tx_frontends").at(0)));
    _tree->access<std::string>(mb_path / "clock_source/value").set("internal");
    _tree->access<std::string>(mb_path / "time_source/value").set("none");

    _server = task::make(boost::bind(&b200_impl::run_server, this));


    /* Attempting to lock to external reference. */
    /* TODO lock to ext ref
    _ctrl->transact_spi(1<<1, spi_config_t::EDGE_RISE, 0x1F8083, 24, false);
    _ctrl->transact_spi(1<<1, spi_config_t::EDGE_RISE, 0x4, 24, false);
    _ctrl->transact_spi(1<<1, spi_config_t::EDGE_RISE, 0x401, 24, false);
    */
}

b200_impl::~b200_impl(void)
{
    //TODO kill any threads here
    //_iface->reset_fpga(true);
}

void b200_impl::set_mb_eeprom(const uhd::usrp::mboard_eeprom_t &mb_eeprom)
{
    //TODO
    //mb_eeprom.commit(*_iface, mboard_eeprom_t::MAP_B100);
}

void b200_impl::check_fw_compat(void)
{
    //TODO
}

void b200_impl::check_fpga_compat(void)
{
    //TODO
    //_ctrl->peek32(REG_RB_COMPAT);....
}

void b200_impl::update_clock_source(const std::string &)
{
    //TODO
}

void b200_impl::update_bandsel(const std::string& which, double freq)
{
    if(which == "RX") {
        if(freq < 2.0e9) {
            _gpio_state.rx_bandsel_a = 0;
            _gpio_state.rx_bandsel_b = 0;
            _gpio_state.rx_bandsel_c = 1;
        } else if((freq >= 2.0e9) && (freq < 4e9)) {
            _gpio_state.rx_bandsel_a = 0;
            _gpio_state.rx_bandsel_b = 1;
            _gpio_state.rx_bandsel_c = 0;
        } else if((freq >= 4e9) && (freq <= 6e9)) {
            _gpio_state.rx_bandsel_a = 1;
            _gpio_state.rx_bandsel_b = 0;
            _gpio_state.rx_bandsel_c = 0;
        } else {
            UHD_THROW_INVALID_CODE_PATH();
        }
    } else if(which == "TX") {
        if(freq < 3e9) {
            _gpio_state.tx_bandsel_a = 0;
            _gpio_state.tx_bandsel_b = 1;
        } else if((freq >= 3e9) && (freq <= 6e9)) {
            _gpio_state.tx_bandsel_a = 1;
            _gpio_state.tx_bandsel_b = 0;
        } else {
            UHD_THROW_INVALID_CODE_PATH();
        }
    } else {
        UHD_THROW_INVALID_CODE_PATH();
    }

    update_gpio_state();
}

void b200_impl::update_gpio_state(void)
{
    UHD_HERE();
    const boost::uint32_t misc_word = 0
        | (_gpio_state.tx_bandsel_a << 10)
        | (_gpio_state.tx_bandsel_b << 9)
        | (_gpio_state.rx_bandsel_a << 8)
        | (_gpio_state.rx_bandsel_b << 7)
        | (_gpio_state.rx_bandsel_c << 6)
        | (_gpio_state.mimo_tx << 5)
        | (_gpio_state.mimo_rx << 4)
        | (_gpio_state.ext_ref_enable << 3)
        | (_gpio_state.pps_fpga_out_enable << 2)
        | (_gpio_state.gps_out_enable << 1)
        | (_gpio_state.gps_ref_enable << 0)
    ;

    _ctrl->poke32(TOREG(SR_MISC_OUTS), misc_word);
}

void b200_impl::update_antenna_sel(const std::string& which, const std::string &ant)
{
    if(ant == "TX/RX") {
        //TODO
    } else if(ant == "RX2") {
        //TODO
    } else {
        throw uhd::value_error("update_antenna_sel unknown antenna " + ant);
    }

    if(which[3] == 'A') {
        _atr0->set_atr_reg(dboard_iface::ATR_REG_IDLE, STATE_FDX_TXRX1);
        _atr0->set_atr_reg(dboard_iface::ATR_REG_RX_ONLY, STATE_FDX_TXRX1);
        _atr0->set_atr_reg(dboard_iface::ATR_REG_TX_ONLY, STATE_FDX_TXRX1);
        _atr0->set_atr_reg(dboard_iface::ATR_REG_FULL_DUPLEX, STATE_FDX_TXRX1);
    } else if(which[3] == 'B') {
        _atr0->set_atr_reg(dboard_iface::ATR_REG_IDLE, STATE_FDX_TXRX2);
        _atr0->set_atr_reg(dboard_iface::ATR_REG_RX_ONLY, STATE_FDX_TXRX2);
        _atr0->set_atr_reg(dboard_iface::ATR_REG_TX_ONLY, STATE_FDX_TXRX2);
        _atr0->set_atr_reg(dboard_iface::ATR_REG_FULL_DUPLEX, STATE_FDX_TXRX2);
    } else {
        throw uhd::value_error("update_antenna_sel unknown side " + which);
    }
}

static inline bool wait_for_recv_ready(int sock_fd, const size_t timeout_ms)
{
    //setup timeval for timeout
    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = timeout_ms*1000;

    //setup rset for timeout
    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(sock_fd, &rset);

    //call select with timeout on receive socket
    return ::select(sock_fd+1, &rset, NULL, NULL, &tv) > 0;
}

void b200_impl::run_server(void)
{
    while (true)
    {try{
        asio::io_service io_service;
        asio::ip::tcp::resolver resolver(io_service);
        asio::ip::tcp::resolver::query query(asio::ip::tcp::v4(), "0.0.0.0", "56789");
        asio::ip::tcp::endpoint endpoint = *resolver.resolve(query);

        boost::shared_ptr<asio::ip::tcp::acceptor> acceptor(new asio::ip::tcp::acceptor(io_service, endpoint));
        {
            boost::shared_ptr<asio::ip::tcp::socket> socket(new asio::ip::tcp::socket(io_service));
            while (not wait_for_recv_ready(acceptor->native(), 100)){
                if (boost::this_thread::interruption_requested()) return;
            }
            acceptor->accept(*socket);
            boost::uint32_t buff[512];
            while (true)
            {
                while (not wait_for_recv_ready(socket->native(), 100)){
                    if (boost::this_thread::interruption_requested()) return;
                }
                socket->receive(asio::buffer(buff, sizeof(buff)));
                const boost::uint32_t action = buff[0];
                const boost::uint32_t reg = buff[1];
                const boost::uint32_t val = buff[2];
                boost::uint32_t result = 0;
                if (action == 0){ //read spi
                    result = _iface->read_reg(reg);
                }
                if (action == 1){ //write spi
                    _iface->write_reg(reg, val);
                }
                if (action == 2){ //peek32
                    result = _ctrl->peek32(reg);
                }
                if (action == 3){ //poke32
                    _ctrl->poke32(reg, val);
                }
                socket->send(asio::buffer(&result, 4));
            }
        }
    }catch(...){}}
}
