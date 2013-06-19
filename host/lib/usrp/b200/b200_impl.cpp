//
// Copyright 2012-2013 Ettus Research LLC
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
#include <uhd/usrp/dboard_eeprom.hpp>
#include <boost/format.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/functional/hash.hpp>
#include <cstdio>
#include <ctime>

#ifdef _MSC_VER

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#endif

#endif

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;

const boost::uint16_t B200_VENDOR_ID  = 0x2500;
const boost::uint16_t B200_PRODUCT_ID = 0x0020;
const boost::uint16_t INIT_PRODUCT_ID = 0x00f0;

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
        pid = B200_PRODUCT_ID;
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

        //check if fw was already loaded
        if (handle->get_manufacturer() != "Ettus Research LLC")
        {
            b200_iface::make(control)->load_firmware(b200_fw_image);
        }

        found++;
    }

    //get descriptors again with serial number, but using the initialized VID/PID now since we have firmware
    const bool init = hint.has_key("vid") and hint.has_key("pid");
    if (init) pid = INIT_PRODUCT_ID;
    //TODO sleep after fw load ?

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
            if (init)
            {
                UHD_HERE();
                byte_vector_t bytes(8);
                bytes[0] = 0x43;
                bytes[1] = 0x59;
                bytes[2] = 0x14;
                bytes[3] = 0xB2;
                bytes[4] = (B200_PRODUCT_ID & 0xff);
                bytes[5] = (B200_PRODUCT_ID >> 8);
                bytes[6] = (B200_VENDOR_ID & 0xff);
                bytes[7] = (B200_VENDOR_ID >> 8);
                iface->write_eeprom(0x0, 0x0, bytes);
                iface->reset_fx3();
                return b200_addrs;
            }
            const mboard_eeprom_t mb_eeprom = mboard_eeprom_t(*iface, "B200");

            device_addr_t new_addr;
            new_addr["type"] = "b200";
            new_addr["name"] = mb_eeprom["name"];
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
    _tree = property_tree::make();
    const fs_path mb_path = "/mboards/0";

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
    // setup the mboard eeprom
    ////////////////////////////////////////////////////////////////////
    const mboard_eeprom_t mb_eeprom(*_iface, "B200");
    _tree->create<mboard_eeprom_t>(mb_path / "eeprom")
        .set(mb_eeprom)
        .subscribe(boost::bind(&b200_impl::set_mb_eeprom, this, _1));

    ////////////////////////////////////////////////////////////////////
    // Load the FPGA image, then reset GPIF
    ////////////////////////////////////////////////////////////////////
    std::string default_file_name;
    if (not mb_eeprom["product"].empty())
    {
        switch (boost::lexical_cast<boost::uint16_t>(mb_eeprom["product"]))
        {
        case 0x0001: default_file_name = B200_FPGA_FILE_NAME; break;
        case 0x0002: default_file_name = B210_FPGA_FILE_NAME; break;
        default: throw uhd::runtime_error("b200 unknown product code: " + mb_eeprom["product"]);
        }
    }
    if (default_file_name.empty())
    {
        UHD_ASSERT_THROW(device_addr.has_key("fpga"));
    }

    //extract the FPGA path for the B200
    std::string b200_fpga_image = find_image_path(
        device_addr.has_key("fpga")? device_addr["fpga"] : default_file_name
    );

    _iface->load_fpga(b200_fpga_image);
    ad9361_ctrl::make(_iface); //radio clock on before global reset
    _iface->reset_gpif();

    ////////////////////////////////////////////////////////////////////
    // Init codec - turns on clocks
    ////////////////////////////////////////////////////////////////////
    _codec_ctrl = ad9361_ctrl::make(_iface);

    ////////////////////////////////////////////////////////////////////
    // Create control transport
    ////////////////////////////////////////////////////////////////////
    boost::uint8_t usb_speed = _iface->get_usb_speed();
    UHD_MSG(status) << "Operating over USB " << (int) usb_speed << "." << std::endl;
    const std::string min_frame_size = (usb_speed == 3) ? "1024" : "512";

    device_addr_t ctrl_xport_args;
    ctrl_xport_args["recv_frame_size"] = min_frame_size;
    ctrl_xport_args["num_recv_frames"] = "16";
    ctrl_xport_args["send_frame_size"] = min_frame_size;
    ctrl_xport_args["num_send_frames"] = "16";

    _ctrl_transport = usb_zero_copy::make(
        handle,
        4, 8, //interface, endpoint
        3, 4, //interface, endpoint
        ctrl_xport_args
    );
    while (_ctrl_transport->get_recv_buff(0.0)){} //flush ctrl xport
    _gpsdo_uart = b200_uart::make(_ctrl_transport, B200_TX_GPS_UART_SID);
    _gpsdo_uart->set_baud_divider(B200_BUS_CLOCK_RATE/115200);
    _async_md.reset(new async_md_type(1000/*messages deep*/));
    _async_task = uhd::task::make(boost::bind(&b200_impl::handle_async_task, this, _ctrl_transport, _async_md, _gpsdo_uart));

    ////////////////////////////////////////////////////////////////////
    // Local control endpoint
    ////////////////////////////////////////////////////////////////////
    _local_ctrl = radio_ctrl_core_3000::make(vrt::if_packet_info_t::LINK_TYPE_CHDR, _ctrl_transport, zero_copy_if::sptr()/*null*/, B200_LOCAL_CTRL_SID);
    _local_ctrl->hold_task(_async_task);
    this->check_fpga_compat();

    ////////////////////////////////////////////////////////////////////
    // Create the GPSDO control
    ////////////////////////////////////////////////////////////////////
    static const boost::uint32_t dont_look_for_gpsdo = 0x1234abcdul;

    //otherwise if not disabled, look for the internal GPSDO
    //TODO if (_iface->peekfw(U2_FW_REG_HAS_GPSDO) != dont_look_for_gpsdo)
    //if (false)
    {
        UHD_MSG(status) << "Detecting internal GPSDO.... " << std::flush;
        try
        {
            _gps = gps_ctrl::make(_gpsdo_uart);
        }
        catch(std::exception &e)
        {
            UHD_MSG(error) << "An error occurred making GPSDO control: " << e.what() << std::endl;
        }
        if (_gps and _gps->gps_detected())
        {
            UHD_MSG(status) << "found" << std::endl;
            BOOST_FOREACH(const std::string &name, _gps->get_sensors())
            {
                _tree->create<sensor_value_t>(mb_path / "sensors" / name)
                    .publish(boost::bind(&gps_ctrl::get_sensor, _gps, name));
            }
        }
        else
        {
            UHD_MSG(status) << "not found" << std::endl;
            //TODO _iface->pokefw(U2_FW_REG_HAS_GPSDO, dont_look_for_gpsdo);
        }
    }

    ////////////////////////////////////////////////////////////////////
    // Initialize the properties tree
    ////////////////////////////////////////////////////////////////////
    _tree->create<std::string>("/name").set("B-Series Device");
    _tree->create<std::string>(mb_path / "name").set("B200");
    _tree->create<std::string>(mb_path / "codename").set("Sasquatch");

    ////////////////////////////////////////////////////////////////////
    // Initialize control (settings regs and async messages)
    ////////////////////////////////////////////////////////////////////
    _ctrl = radio_ctrl_core_3000::make(vrt::if_packet_info_t::LINK_TYPE_CHDR, _ctrl_transport, zero_copy_if::sptr()/*null*/, B200_CTRL_MSG_SID);
    _ctrl->hold_task(_async_task);
    _tree->create<time_spec_t>(mb_path / "time" / "cmd")
        .subscribe(boost::bind(&radio_ctrl_core_3000::set_time, _ctrl, _1));
    this->register_loopback_self_test();

    ////////////////////////////////////////////////////////////////////
    // Create data transport
    // This happens after FPGA ctrl instantiated so any junk that might
    // be in the FPGAs buffers doesn't get pulled into the transport
    // before being cleared.
    ////////////////////////////////////////////////////////////////////
    device_addr_t data_xport_args;
    data_xport_args["recv_frame_size"] = device_addr.get("recv_frame_size", "8192");
    data_xport_args["num_recv_frames"] = device_addr.get("num_recv_frames", "16");
    data_xport_args["send_frame_size"] = device_addr.get("send_frame_size", "8192");
    data_xport_args["num_send_frames"] = device_addr.get("num_send_frames", "16");

    _data_transport = usb_zero_copy::make(
        handle,        // identifier
        2, 6,          // IN interface, endpoint
        1, 2,          // OUT interface, endpoint
        data_xport_args    // param hints
    );
    while (_data_transport->get_recv_buff(0.0)){} //flush ctrl xport

    ////////////////////////////////////////////////////////////////////
    // create gpio and misc controls
    ////////////////////////////////////////////////////////////////////
    _atr0 = gpio_core_200_32wo::make(_ctrl, TOREG(SR_ATR0));
    _atr1 = gpio_core_200_32wo::make(_ctrl, TOREG(SR_ATR1));

    /* Initialize the GPIOs, set the default bandsels to the lower range. Note
     * that calling update_bandsel calls update_gpio_state(). */
    _gpio_state = gpio_state();
    update_bandsel("RX", 800e6);
    update_bandsel("TX", 850e6);

    ////////////////////////////////////////////////////////////////////
    // create codec control objects
    ////////////////////////////////////////////////////////////////////
    static const std::vector<std::string> frontends = boost::assign::list_of
        ("TX1")("TX2")("RX1")("RX2");

    //default some chains on -- needed for setup purposes
    _codec_ctrl->set_active_chains(true, false, true, false);

    BOOST_FOREACH(const std::string &fe_name, frontends)
    {
        _fe_enb_map[fe_name] = false;
    }
    {
        _codec_ctrl->data_port_loopback(true);
        this->codec_loopback_self_test();
        _codec_ctrl->data_port_loopback(false);
    }
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

    ////////////////////////////////////////////////////////////////////
    // create clock control objects
    ////////////////////////////////////////////////////////////////////
    //^^^ clock created up top, just reg props here... ^^^
    _tree->create<double>(mb_path / "tick_rate")
        .publish(boost::bind(&b200_impl::get_tick_rate, this))
        .subscribe(boost::bind(&b200_impl::set_tick_rate, this, _1));

    ////////////////////////////////////////////////////////////////////
    // and do the misc mboard sensors
    ////////////////////////////////////////////////////////////////////
    _tree->create<int>(mb_path / "sensors"); //empty but path exists TODO
    _tree->create<sensor_value_t>(mb_path / "sensors" / "ref_locked")
        .publish(boost::bind(&b200_impl::get_ref_locked, this));

    ////////////////////////////////////////////////////////////////////
    // create frontend mapping
    ////////////////////////////////////////////////////////////////////
    _tree->create<subdev_spec_t>(mb_path / "rx_subdev_spec")
        .set(subdev_spec_t())
        .subscribe(boost::bind(&b200_impl::update_rx_subdev_spec, this, _1));
    _tree->create<subdev_spec_t>(mb_path / "tx_subdev_spec")
        .set(subdev_spec_t())
        .subscribe(boost::bind(&b200_impl::update_tx_subdev_spec, this, _1));

    ////////////////////////////////////////////////////////////////////
    // create rx dsp control objects
    ////////////////////////////////////////////////////////////////////
    _rx_framer = rx_vita_core_3000::make(_ctrl, TOREG(SR_RX_CTRL));
    for (size_t dspno = 0; dspno < B200_NUM_RX_FE; dspno++)
    {
        const fs_path rx_dsp_path = mb_path / "rx_dsps" / str(boost::format("%u") % dspno);
        _tree->create<meta_range_t>(rx_dsp_path / "rate" / "range")
            .publish(boost::bind(&ad9361_ctrl::get_samp_rate_range));
        _tree->create<double>(rx_dsp_path / "rate" / "value")
            .publish(boost::bind(&b200_impl::get_rx_sample_rate, this))
            .subscribe(boost::bind(&b200_impl::set_rx_sample_rate, this, _1));
        _tree->create<double>(rx_dsp_path / "freq" / "value")
            .publish(boost::bind(&b200_impl::get_dsp_freq, this));
        _tree->create<meta_range_t>(rx_dsp_path / "freq/range")
            .publish(boost::bind(&b200_impl::get_dsp_freq_range, this));
        _tree->create<stream_cmd_t>(rx_dsp_path / "stream_cmd")
            .subscribe(boost::bind(&b200_impl::issue_stream_cmd, this, dspno, _1));
    }

    ////////////////////////////////////////////////////////////////////
    // create tx dsp control objects
    ////////////////////////////////////////////////////////////////////
    _tx_deframer = tx_vita_core_3000::make(_ctrl, TOREG(SR_TX_CTRL));
    for (size_t dspno = 0; dspno < B200_NUM_TX_FE; dspno++)
    {
        const fs_path tx_dsp_path = mb_path / "tx_dsps" / str(boost::format("%u") % dspno);
        _tree->create<meta_range_t>(tx_dsp_path / "rate" / "range")
            .publish(boost::bind(&ad9361_ctrl::get_samp_rate_range));
        _tree->create<double>(tx_dsp_path / "rate" / "value")
            .publish(boost::bind(&b200_impl::get_tx_sample_rate, this))
            .subscribe(boost::bind(&b200_impl::set_tx_sample_rate, this, _1));
        _tree->create<double>(tx_dsp_path / "freq" / "value")
            .publish(boost::bind(&b200_impl::get_dsp_freq, this));
        _tree->create<meta_range_t>(tx_dsp_path / "freq" / "range")
            .publish(boost::bind(&b200_impl::get_dsp_freq_range, this));
    }

    ////////////////////////////////////////////////////////////////////
    // create time and clock control objects
    ////////////////////////////////////////////////////////////////////
    _spi_iface = spi_core_3000::make(_local_ctrl, TOREG(SR_CORE_SPI), RB32_CORE_SPI);
    _spi_iface->set_divider(B200_BUS_CLOCK_RATE/100);
    _adf4001_iface = boost::shared_ptr<adf4001_ctrl>(new adf4001_ctrl(_spi_iface, (1 << 1)/*slaveno*/));

    time_core_3000::readback_bases_type time64_rb_bases;
    time64_rb_bases.rb_now = RB64_TIME_NOW;
    time64_rb_bases.rb_pps = RB64_TIME_PPS;
    _time64 = time_core_3000::make(_ctrl, TOREG(SR_TIME), time64_rb_bases);
    _tree->create<time_spec_t>(mb_path / "time" / "now")
        .publish(boost::bind(&time_core_3000::get_time_now, _time64))
        .subscribe(boost::bind(&time_core_3000::set_time_now, _time64, _1));
    _tree->create<time_spec_t>(mb_path / "time" / "pps")
        .publish(boost::bind(&time_core_3000::get_time_last_pps, _time64))
        .subscribe(boost::bind(&time_core_3000::set_time_next_pps, _time64, _1));
    //setup time source props
    _tree->create<std::string>(mb_path / "time_source" / "value")
        .subscribe(boost::bind(&b200_impl::update_time_source, this, _1));
    static const std::vector<std::string> time_sources = boost::assign::list_of("none")("external")("gpsdo");
    _tree->create<std::vector<std::string> >(mb_path / "time_source" / "options").set(time_sources);
    //setup reference source props
    _tree->create<std::string>(mb_path / "clock_source" / "value")
        .subscribe(boost::bind(&b200_impl::update_clock_source, this, _1));
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
            .subscribe(boost::bind(&b200_impl::update_bandsel, this, fe_name, _1));
        _tree->create<meta_range_t>(rf_fe_path / "freq" / "range")
            .publish(boost::bind(&ad9361_ctrl::get_rf_freq_range));

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

    //init the clock rate to something, but only when we have active chains
    _tree->access<double>(mb_path / "tick_rate").set(61.44e6/8);
    _codec_ctrl->set_active_chains(false, false, false, false);

    //-- this block commented out because we want all chains off by default --//
    //_tree->access<subdev_spec_t>(mb_path / "rx_subdev_spec").set(subdev_spec_t("A:RX2"));
    //_tree->access<subdev_spec_t>(mb_path / "tx_subdev_spec").set(subdev_spec_t("A:TX2"));
    //-- this block commented out because we want all chains off by default --//

    _tree->access<std::string>(mb_path / "clock_source/value").set("internal");
    _tree->access<std::string>(mb_path / "time_source/value").set("none");

    //GPS installed: use external ref, time, and init time spec
    if (false and _gps and _gps->gps_detected())
    {
        UHD_MSG(status) << "Setting references to the internal GPSDO" << std::endl;
        _tree->access<std::string>(mb_path / "time_source" / "value").set("gpsdo");
        _tree->access<std::string>(mb_path / "clock_source" / "value").set("gpsdo");
        UHD_MSG(status) << "Initializing time to the internal GPSDO" << std::endl;
        const time_t tp = time_t(_gps->get_sensor("gps_time").to_int()+1);
        _tree->access<time_spec_t>(mb_path / "time" / "pps").set(time_spec_t(tp));
    }

}

b200_impl::~b200_impl(void)
{
    UHD_SAFE_CALL
    (
        _async_task.reset();
        _codec_ctrl->set_active_chains(false, false, false, false);
    )
}


/***********************************************************************
 * loopback tests
 **********************************************************************/
 
void b200_impl::register_loopback_self_test(void)
{
    bool test_fail = false;
    UHD_MSG(status) << "Performing register loopback test... " << std::flush;
    size_t hash = size_t(time(NULL));
    for (size_t i = 0; i < 100; i++)
    {
        boost::hash_combine(hash, i);
        _ctrl->poke32(TOREG(SR_TEST), boost::uint32_t(hash));
        test_fail = _ctrl->peek32(RB32_TEST) != boost::uint32_t(hash);
        if (test_fail) break; //exit loop on any failure
    }
    UHD_MSG(status) << ((test_fail)? " fail" : "pass") << std::endl;
}

void b200_impl::codec_loopback_self_test(void)
{
    bool test_fail = false;
    UHD_MSG(status) << "Performing CODEC loopback test... " << std::flush;
    size_t hash = size_t(time(NULL));
    for (size_t i = 0; i < 100; i++)
    {
        boost::hash_combine(hash, i);
        const boost::uint32_t word32 = boost::uint32_t(hash) & 0xfff0fff0;
        _ctrl->poke32(TOREG(SR_CODEC_IDLE), word32);
        _ctrl->peek64(RB64_CODEC_READBACK); //enough idleness for loopback to propagate
        const boost::uint64_t rb_word64 = _ctrl->peek64(RB64_CODEC_READBACK);
        const boost::uint32_t rb_tx = boost::uint32_t(rb_word64 >> 32);
        const boost::uint32_t rb_rx = boost::uint32_t(rb_word64 & 0xffffffff);
        test_fail = word32 != rb_tx or word32 != rb_rx;
        if (test_fail) break; //exit loop on any failure
    }
    UHD_MSG(status) << ((test_fail)? " fail" : "pass") << std::endl;

    /* Zero out the idle data. */
    _ctrl->poke32(TOREG(SR_CODEC_IDLE), 0);
}

/***********************************************************************
 * Sample and tick rate comprehension below
 **********************************************************************/
void b200_impl::set_tick_rate(const double raw_rate)
{
    //clip rate (which can be doubled by factor) to possible bounds
    const double rate = ad9361_ctrl::get_samp_rate_range().clip(raw_rate);

    const size_t factor = ((_fe_enb_map["RX1"] and _fe_enb_map["RX2"]) or (_fe_enb_map["TX1"] and _fe_enb_map["TX2"]))? 2:1;
    //UHD_MSG(status) << "asking for clock rate " << rate/1e6 << " MHz\n";
    _tick_rate = _codec_ctrl->set_clock_rate(rate/factor)*factor;
    //UHD_MSG(status) << "actually got clock rate " << _tick_rate/1e6 << " MHz\n";
    this->update_streamer_rates();
    _time64->set_tick_rate(_tick_rate);
    _time64->self_test();
    _rx_framer->set_tick_rate(_tick_rate);
    _tx_deframer->set_tick_rate(_tick_rate);
}

void b200_impl::set_rx_sample_rate(const double rate)
{
    const size_t factor = (_fe_enb_map["RX1"] and _fe_enb_map["RX2"])? 2:1;
    this->set_tick_rate(rate*factor);
}

void b200_impl::set_tx_sample_rate(const double rate)
{
    const size_t factor = (_fe_enb_map["TX1"] and _fe_enb_map["TX2"])? 2:1;
    this->set_tick_rate(rate*factor);
}

double b200_impl::get_rx_sample_rate(void)
{
    const size_t factor = (_fe_enb_map["RX1"] and _fe_enb_map["RX2"])? 2:1;
    return _tick_rate/factor;
}

double b200_impl::get_tx_sample_rate(void)
{
    const size_t factor = (_fe_enb_map["TX1"] and _fe_enb_map["TX2"])? 2:1;
    return _tick_rate/factor;
}

/***********************************************************************
 * compat checks
 **********************************************************************/

void b200_impl::check_fw_compat(void)
{
    boost::uint16_t compat_num = _iface->get_compat_num();
    boost::uint32_t compat_major = (boost::uint32_t) (compat_num >> 8);
    boost::uint32_t compat_minor = (boost::uint32_t) (compat_num & 0xFF);

    if (compat_major != B200_FW_COMPAT_NUM_MAJOR){
        throw uhd::runtime_error(str(boost::format(
            "Expected firmware compatibility number 0x%x, but got 0x%x.%x:\n"
            "The firmware build is not compatible with the host code build.\n"
            "%s"
        ) % int(B200_FW_COMPAT_NUM_MAJOR) % compat_major % compat_minor
          % print_images_error()));
    }
    _tree->create<std::string>("/mboards/0/fw_version").set(str(boost::format("%u.%u")
                % compat_major % compat_minor));
}

void b200_impl::check_fpga_compat(void)
{
    const boost::uint64_t compat = _local_ctrl->peek64(0);
    const boost::uint32_t signature = boost::uint32_t(compat >> 32);
    const boost::uint16_t compat_major = boost::uint16_t(compat >> 16);
    const boost::uint16_t compat_minor = boost::uint16_t(compat & 0xffff);
    if (signature != 0xACE0BA5E) throw uhd::runtime_error(
        "b200::check_fpga_compat signature register readback failed");

    if (compat_major != B200_FPGA_COMPAT_NUM){
        throw uhd::runtime_error(str(boost::format(
            "Expected FPGA compatibility number 0x%x, but got 0x%x.%x:\n"
            "The FPGA build is not compatible with the host code build.\n"
            "%s"
        ) % int(B200_FPGA_COMPAT_NUM) % compat_major % compat_minor
          % print_images_error()));
    }
    _tree->create<std::string>("/mboards/0/fpga_version").set(str(boost::format("%u.%u")
                % compat_major % compat_minor));
}

void b200_impl::set_mb_eeprom(const uhd::usrp::mboard_eeprom_t &mb_eeprom)
{
    mb_eeprom.commit(*_iface, "B200");
}


/***********************************************************************
 * Reference time and clock
 **********************************************************************/

void b200_impl::update_clock_source(const std::string &source)
{
    if (source == "internal"){
        _adf4001_iface->set_lock_to_ext_ref(false);
    }
    else if ((source == "external")
              or (source == "gpsdo")){

        _adf4001_iface->set_lock_to_ext_ref(true);
    } else {
        throw uhd::key_error("update_clock_source: unknown source: " + source);
    }

    _gpio_state.ref_sel = (source == "gpsdo")? 1 : 0;
    this->update_gpio_state();
}

void b200_impl::update_time_source(const std::string &source)
{
    if (source == "none"){}
    else if (source == "external"){}
    else if (source == "gpsdo"){}
    else throw uhd::key_error("update_time_source: unknown source: " + source);
    _time64->set_time_source((source == "external")? "external" : "internal");
    this->update_gpio_state();
}

/***********************************************************************
 * GPIO setup
 **********************************************************************/

void b200_impl::update_bandsel(const std::string& which, double freq)
{
    if(which[0] == 'R') {
        if(freq < 2.2e9) {
            _gpio_state.rx_bandsel_a = 0;
            _gpio_state.rx_bandsel_b = 0;
            _gpio_state.rx_bandsel_c = 1;
        } else if((freq >= 2.2e9) && (freq < 4e9)) {
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
    } else if(which[0] == 'T') {
        if(freq < 2.5e9) {
            _gpio_state.tx_bandsel_a = 0;
            _gpio_state.tx_bandsel_b = 1;
        } else if((freq >= 2.5e9) && (freq <= 6e9)) {
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
    const boost::uint32_t misc_word = 0
        | (_gpio_state.tx_bandsel_a << 10)
        | (_gpio_state.tx_bandsel_b << 9)
        | (_gpio_state.rx_bandsel_a << 8)
        | (_gpio_state.rx_bandsel_b << 7)
        | (_gpio_state.rx_bandsel_c << 6)
        | (_gpio_state.mimo_tx << 2)
        | (_gpio_state.mimo_rx << 1)
        | (_gpio_state.ref_sel << 0)
    ;

    _ctrl->poke32(TOREG(SR_MISC_OUTS), misc_word);
}

void b200_impl::update_atrs(void)
{
    {
        const bool is_rx2 = _fe_ant_map.get("RX1", "RX2") == "RX2";
        const size_t rxonly = (_fe_enb_map["RX1"])? ((is_rx2)? STATE_RX1_RX2 : STATE_RX1_TXRX) : STATE_OFF;
        const size_t txonly = (_fe_enb_map["TX1"])? (STATE_TX1_TXRX) : STATE_OFF;
        size_t fd = STATE_OFF;
        if (_fe_enb_map["RX1"] and _fe_enb_map["TX1"]) fd = STATE_FDX1_TXRX;
        if (_fe_enb_map["RX1"] and not _fe_enb_map["TX1"]) fd = rxonly;
        if (not _fe_enb_map["RX1"] and _fe_enb_map["TX1"]) fd = txonly;
        _atr0->set_atr_reg(dboard_iface::ATR_REG_IDLE, STATE_OFF);
        _atr0->set_atr_reg(dboard_iface::ATR_REG_RX_ONLY, rxonly);
        _atr0->set_atr_reg(dboard_iface::ATR_REG_TX_ONLY, txonly);
        _atr0->set_atr_reg(dboard_iface::ATR_REG_FULL_DUPLEX, fd);
    }
    {
        const bool is_rx2 = _fe_ant_map.get("RX2", "RX2") == "RX2";
        const size_t rxonly = (_fe_enb_map["RX2"])? ((is_rx2)? STATE_RX2_RX2 : STATE_RX2_TXRX) : STATE_OFF;
        const size_t txonly = (_fe_enb_map["TX2"])? (STATE_TX2_TXRX) : STATE_OFF;
        size_t fd = STATE_OFF;
        if (_fe_enb_map["RX2"] and _fe_enb_map["TX2"]) fd = STATE_FDX2_TXRX;
        if (_fe_enb_map["RX2"] and not _fe_enb_map["TX2"]) fd = rxonly;
        if (not _fe_enb_map["RX2"] and _fe_enb_map["TX2"]) fd = txonly;
        _atr1->set_atr_reg(dboard_iface::ATR_REG_IDLE, STATE_OFF);
        _atr1->set_atr_reg(dboard_iface::ATR_REG_RX_ONLY, rxonly);
        _atr1->set_atr_reg(dboard_iface::ATR_REG_TX_ONLY, txonly);
        _atr1->set_atr_reg(dboard_iface::ATR_REG_FULL_DUPLEX, fd);
    }
}

void b200_impl::update_antenna_sel(const std::string& which, const std::string &ant)
{
    _fe_ant_map[which] = ant;
    this->update_atrs();
}

void b200_impl::update_enables(void)
{
    _codec_ctrl->set_active_chains(_fe_enb_map["TX1"], _fe_enb_map["TX2"], _fe_enb_map["RX1"], _fe_enb_map["RX2"]);
    this->update_atrs();
}

sensor_value_t b200_impl::get_ref_locked(void)
{
    const bool lock = (_local_ctrl->peek32(RB32_CORE_MISC) & 0x1) == 0x1;
    return sensor_value_t("Ref", lock, "locked", "unlocked");
}
