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
#include <cstdio>
#include <iomanip>

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
        UHD_LOG << "the  firmware image: " << b200_fw_image << std::endl;

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

    //load the fpga
    //TODO
//    _iface->load_fpga(b200_fpga_image);
    _iface->load_fpga("");


    ////////////////////////////////////////////////////////////////////
    // Get the FPGA a clock from Catalina
    ////////////////////////////////////////////////////////////////////
    _iface->write_reg(0x00A, 0b00010010);
    _iface->write_reg(0x009, 0b00010111);


    ////////////////////////////////////////////////////////////////////
    // Create control transport
    ////////////////////////////////////////////////////////////////////
    device_addr_t ctrl_xport_args;
    ctrl_xport_args["recv_frame_size"] = "512";
    ctrl_xport_args["num_recv_frames"] = "16";
    ctrl_xport_args["send_frame_size"] = "512";
    ctrl_xport_args["num_send_frames"] = "16";

    _ctrl_transport = usb_zero_copy::make(
        handle,
        4, 8, //interface, endpoint
        3, 4, //interface, endpoint
        ctrl_xport_args
    );
    while (_ctrl_transport->get_recv_buff(0.0)){} //flush ctrl xport

    ////////////////////////////////////////////////////////////////////
    // Create data transport
    ////////////////////////////////////////////////////////////////////
    device_addr_t data_xport_args;
    data_xport_args["recv_frame_size"] = device_addr.get("recv_frame_size", "16384");
    data_xport_args["num_recv_frames"] = device_addr.get("num_recv_frames", "16");
    data_xport_args["send_frame_size"] = device_addr.get("send_frame_size", "16384");
    data_xport_args["num_send_frames"] = device_addr.get("num_send_frames", "16");

    _data_transport = usb_zero_copy::make_wrapper(
        usb_zero_copy::make(
            handle,        // identifier
            2, 6,          // IN interface, endpoint
            1, 2,          // OUT interface, endpoint
            data_xport_args    // param hints
        ),
        B200_MAX_PKT_BYTE_LIMIT
    );

    _rx_demux = recv_packet_demuxer::make(_data_transport, B200_NUM_RX_FE, B200_RX_SID_BASE);

    ////////////////////////////////////////////////////////////////////
    // Initialize control (settings regs and async messages)
    ////////////////////////////////////////////////////////////////////
    _ctrl = b200_ctrl::make(_ctrl_transport);
    /*
    this->check_fpga_compat(); //check after making
    */

    ////////////////////////////////////////////////////////////////////
    // Initialize the properties tree
    ////////////////////////////////////////////////////////////////////
    _tree->create<std::string>("/name").set("B-Series Device");
    const fs_path mb_path = "/mboards/0";
    _tree->create<std::string>(mb_path / "name").set("B200");
    _tree->create<std::string>(mb_path / "codename").set("Sasquatch");

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
    _atr0 = gpio_core_200::make(_ctrl, SR_GPIO0, 0/*unused*/);
    _atr1 = gpio_core_200::make(_ctrl, SR_GPIO1, 0/*unused*/);
    _gpio_state = gpio_state(); //clear all to zero

    //set bandsel switches to A band fixed for now
    //assign { /* 2'bX, */ ext_ref_enable, dac_shdn, pps_fpga_out_enable, pps_gps_out_enable, gps_out_enable, gps_ref_enable } = misc_outs[23:16];
    //assign { /* 3'bX, */ tx_bandsel_a, tx_bandsel_b, rx_bandsel_a, rx_bandsel_b, rx_bandsel_c } = misc_outs[15:8];
    //assign { /* 7'bX, */ mimo } = misc_outs[7:0];
    _gpio_state.tx_bandsel_a = 1;
    _gpio_state.rx_bandsel_a = 1;
    update_gpio_state(); //first time init

    //set ATR for FDX operation fixed GPIO, amps enabled, LEDs all on
    //set all ATR to GPIO not ATR
    //assign {tx_enable1, SFDX1_RX, SFDX1_TX, SRX1_RX, SRX1_TX, LED_RX1, LED_TXRX1_RX, LED_TXRX1_TX} = atr0[23:16];
    //assign {tx_enable2, SFDX2_RX, SFDX2_TX, SRX2_RX, SRX2_TX, LED_RX2, LED_TXRX2_RX, LED_TXRX2_TX} = atr1[23:16];
    //assign {codec_txrx, codec_en_agc, codec_ctrl_in[3:0] } = atr0[5:0];
    _atr0->set_pin_ctrl(dboard_iface::UNIT_TX, 0x0000);
    _atr0->set_pin_ctrl(dboard_iface::UNIT_RX, 0x0000);
    _atr1->set_pin_ctrl(dboard_iface::UNIT_TX, 0x0000);
    _atr1->set_pin_ctrl(dboard_iface::UNIT_RX, 0x0000);
    
    _atr0->set_gpio_ddr(dboard_iface::UNIT_TX, //upper 16
                       0xFFFF);
    _atr0->set_gpio_ddr(dboard_iface::UNIT_RX, //lower 16
                       0x30);
    _atr1->set_gpio_ddr(dboard_iface::UNIT_TX, //upper 16
                       0xFFFF);
    _atr1->set_gpio_ddr(dboard_iface::UNIT_RX, //lower 16
                       0xFFFF);

    _atr0->set_gpio_out(dboard_iface::UNIT_TX, //upper 16
                        STATE_TX);
    _atr0->set_gpio_out(dboard_iface::UNIT_RX, //lower 16
                        CODEC_TXRX);
    _atr1->set_gpio_out(dboard_iface::UNIT_TX, //upper 16
                        STATE_TX);
    _atr1->set_gpio_out(dboard_iface::UNIT_RX, //lower 16
                        0x00);    
                       

    ////////////////////////////////////////////////////////////////////
    // create codec control objects
    ////////////////////////////////////////////////////////////////////
    _codec_ctrl = b200_codec_ctrl::make(_iface);
    static const std::vector<std::string> frontends = boost::assign::list_of
        ("TX_A")("TX_B")("RX_A")("RX_B")
    ;
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
    // create rx frontend control objects
    ////////////////////////////////////////////////////////////////////
    _rx_fes.resize(B200_NUM_RX_FE);
    for (size_t i = 0; i < _rx_fes.size(); i++)
    {
        _rx_fes[i] = rx_frontend_core_200::make(_ctrl, SR_RX_FRONTEND(i));
        const std::string which = std::string(1, i+'A');
        const fs_path rx_fe_path = mb_path / "rx_frontends" / which;

        _tree->create<std::complex<double> >(rx_fe_path / "dc_offset" / "value")
            .coerce(boost::bind(&rx_frontend_core_200::set_dc_offset, _rx_fes[i], _1))
            .set(std::complex<double>(0.0, 0.0));
        _tree->create<bool>(rx_fe_path / "dc_offset" / "enable")
            .subscribe(boost::bind(&rx_frontend_core_200::set_dc_offset_auto, _rx_fes[i], _1))
            .set(true);
        _tree->create<std::complex<double> >(rx_fe_path / "iq_balance" / "value")
            .subscribe(boost::bind(&rx_frontend_core_200::set_iq_balance, _rx_fes[i], _1))
            .set(std::complex<double>(0.0, 0.0));
    }

    ////////////////////////////////////////////////////////////////////
    // create tx frontend control objects
    ////////////////////////////////////////////////////////////////////
    _tx_fes.resize(B200_NUM_TX_FE);
    for (size_t i = _tx_fes.size(); i < 2; i++)
    {
        _tx_fes[i] = tx_frontend_core_200::make(_ctrl, SR_TX_FRONTEND(i));
        const std::string which = std::string(1, i+'A');
        const fs_path tx_fe_path = mb_path / "tx_frontends" / which;

        _tree->create<std::complex<double> >(tx_fe_path / "dc_offset" / "value")
            .coerce(boost::bind(&tx_frontend_core_200::set_dc_offset, _tx_fes[i], _1))
            .set(std::complex<double>(0.0, 0.0));
        _tree->create<std::complex<double> >(tx_fe_path / "iq_balance" / "value")
            .subscribe(boost::bind(&tx_frontend_core_200::set_iq_balance, _tx_fes[i], _1))
            .set(std::complex<double>(0.0, 0.0));
    }

    ////////////////////////////////////////////////////////////////////
    // create rx dsp control objects
    ////////////////////////////////////////////////////////////////////
    _rx_dsps.resize(B200_NUM_RX_FE);
    for (size_t dspno = 0; dspno < _rx_dsps.size(); dspno++)
    {
        const fs_path rx_dsp_path = mb_path / str(boost::format("rx_dsps/%u") % dspno);
        _rx_dsps[dspno] = rx_dsp_core_200::make(_ctrl, SR_RX_DSP(dspno), SR_RX_CTRL(dspno), B200_RX_SID_BASE + dspno);
        _rx_dsps[dspno]->set_link_rate(B200_LINK_RATE_BPS);
        _tree->access<double>(mb_path / "tick_rate")
            .subscribe(boost::bind(&rx_dsp_core_200::set_tick_rate, _rx_dsps[dspno], _1));
        _tree->create<meta_range_t>(rx_dsp_path / "rate/range")
            .publish(boost::bind(&rx_dsp_core_200::get_host_rates, _rx_dsps[dspno]));
        _tree->create<double>(rx_dsp_path / "rate/value")
            .set(1e6) //some default
            .coerce(boost::bind(&rx_dsp_core_200::set_host_rate, _rx_dsps[dspno], _1))
            .subscribe(boost::bind(&b200_impl::update_rx_samp_rate, this, dspno, _1));
        _tree->create<double>(rx_dsp_path / "freq/value")
            .coerce(boost::bind(&rx_dsp_core_200::set_freq, _rx_dsps[dspno], _1));
        _tree->create<meta_range_t>(rx_dsp_path / "freq/range")
            .publish(boost::bind(&rx_dsp_core_200::get_freq_range, _rx_dsps[dspno]));
        _tree->create<stream_cmd_t>(rx_dsp_path / "stream_cmd")
            .subscribe(boost::bind(&rx_dsp_core_200::issue_stream_command, _rx_dsps[dspno], _1));
    }

    ////////////////////////////////////////////////////////////////////
    // create tx dsp control objects
    ////////////////////////////////////////////////////////////////////
    _tx_dsps.resize(B200_NUM_TX_FE);
    for (size_t dspno = 0; dspno < _tx_dsps.size(); dspno++)
    {
        const fs_path tx_dsp_path = mb_path / str(boost::format("tx_dsps/%u") % dspno);
        _tx_dsps[dspno] = tx_dsp_core_200::make(_ctrl, SR_TX_DSP(dspno), SR_TX_CTRL(dspno), B200_ASYNC_SID_BASE + dspno);
        _tx_dsps[dspno]->set_link_rate(B200_LINK_RATE_BPS);
        _tree->access<double>(mb_path / "tick_rate")
            .subscribe(boost::bind(&tx_dsp_core_200::set_tick_rate, _tx_dsps[dspno], _1));
        _tree->create<meta_range_t>(tx_dsp_path / "rate/range")
            .publish(boost::bind(&tx_dsp_core_200::get_host_rates, _tx_dsps[dspno]));
        _tree->create<double>(tx_dsp_path / "rate/value")
            .set(1e6) //some default
            .coerce(boost::bind(&tx_dsp_core_200::set_host_rate, _tx_dsps[dspno], _1))
            .subscribe(boost::bind(&b200_impl::update_tx_samp_rate, this, 0, _1));
        _tree->create<double>(tx_dsp_path / "freq/value")
            .coerce(boost::bind(&tx_dsp_core_200::set_freq, _tx_dsps[dspno], _1));
        _tree->create<meta_range_t>(tx_dsp_path / "freq/range")
            .publish(boost::bind(&tx_dsp_core_200::get_freq_range, _tx_dsps[dspno]));
    }

    ////////////////////////////////////////////////////////////////////
    // create time control objects
    ////////////////////////////////////////////////////////////////////
    time64_core_200::readback_bases_type time64_rb_bases;
    time64_rb_bases.rb_hi_now = REG_RB_TIME_NOW_HI;
    time64_rb_bases.rb_lo_now = REG_RB_TIME_NOW_LO;
    time64_rb_bases.rb_hi_pps = REG_RB_TIME_PPS_HI;
    time64_rb_bases.rb_lo_pps = REG_RB_TIME_PPS_LO;
    _time64 = time64_core_200::make(_ctrl, SR_TIME64*4, time64_rb_bases);
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
    // create user-defined control objects
    ////////////////////////////////////////////////////////////////////
    _user = user_settings_core_200::make(_ctrl, SR_USER_REGS*4);
    _tree->create<user_settings_core_200::user_reg_t>(mb_path / "user/regs")
        .subscribe(boost::bind(&user_settings_core_200::set_reg, _user, _1));

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
        _tree->create<int>(rf_fe_path / "gains"); //TODO empty so it exists (fixed when loop below has iterations)
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
        _tree->create<double>(rf_fe_path / "bandwidth" / "value").set(0.0); //TODO
        _tree->create<meta_range_t>(rf_fe_path / "bandwidth" / "range").set(meta_range_t(0.0, 0.0)); //TODO
        _tree->create<double>(rf_fe_path / "freq" / "value")
            .coerce(boost::bind(&b200_codec_ctrl::tune, _codec_ctrl, fe_name, _1))
            .set(1e9);
        _tree->create<meta_range_t>(rf_fe_path / "freq" / "range").set(meta_range_t(0.0, 0.0)); //TODO
        _tree->create<std::string>(rf_fe_path / "antenna" / "value").set(""); //TODO
        _tree->create<std::vector<std::string> >(rf_fe_path / "antenna" / "options").set(std::vector<std::string>(1, "")); //TODO

    }

    ////////////////////////////////////////////////////////////////////
    // do some post-init tasks
    ////////////////////////////////////////////////////////////////////
    //allocate streamer weak ptrs containers
    _rx_streamers.resize(_rx_dsps.size());
    _tx_streamers.resize(_tx_dsps.size());

    _ctrl->poke32((SR_MISC + 1)*4, 0xA);

    _tree->access<double>(mb_path / "tick_rate") //now subscribe the clock rate setter
        .subscribe(boost::bind(&b200_ctrl::set_tick_rate, _ctrl, _1))
        .set(40e6);

    this->update_rates();

    //reset cordic rates and their properties to zero
    BOOST_FOREACH(const std::string &name, _tree->list(mb_path / "rx_dsps")){
        _tree->access<double>(mb_path / "rx_dsps" / name / "freq" / "value").set(0.0);
    }
    BOOST_FOREACH(const std::string &name, _tree->list(mb_path / "tx_dsps")){
        _tree->access<double>(mb_path / "tx_dsps" / name / "freq" / "value").set(0.0);
    }

    _tree->access<subdev_spec_t>(mb_path / "rx_subdev_spec").set(subdev_spec_t("A:" + _tree->list(mb_path / "dboards/A/rx_frontends").at(0)));
    _tree->access<subdev_spec_t>(mb_path / "tx_subdev_spec").set(subdev_spec_t("A:" + _tree->list(mb_path / "dboards/A/tx_frontends").at(0)));
    _tree->access<std::string>(mb_path / "clock_source/value").set("internal");
    _tree->access<std::string>(mb_path / "time_source/value").set("none");

}

b200_impl::~b200_impl(void)
{
    //TODO kill any threads here
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

void b200_impl::update_gpio_state(void)
{
    //TODO a lot more TODO

    const boost::uint32_t misc_word = 0
        | (_gpio_state.ext_ref_enable << 21)
        | (_gpio_state.dac_shdn << 20)
        | (_gpio_state.pps_fpga_out_enable << 19)
        | (_gpio_state.pps_gps_out_enable << 18)
        | (_gpio_state.gps_out_enable << 17)
        | (_gpio_state.gps_ref_enable << 16)
        | (_gpio_state.tx_bandsel_a << 12)
        | (_gpio_state.tx_bandsel_b << 11)
        | (_gpio_state.rx_bandsel_a << 10)
        | (_gpio_state.rx_bandsel_b << 9)
        | (_gpio_state.rx_bandsel_c << 8)
        | (_gpio_state.mimo << 0)
    ;
    _ctrl->poke32(SR_MISC*4, misc_word);
}
