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

#include "b250_impl.hpp"
#include "b250_regs.hpp"
#include <uhd/utils/static.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/utils/images.hpp>
#include <uhd/utils/safe_call.hpp>
#include <uhd/usrp/subdev_spec.hpp>
#include <uhd/transport/if_addrs.hpp>
#include <boost/foreach.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/functional/hash.hpp>
#include <boost/assign/list_of.hpp>
#include <fstream>
#include <uhd/transport/udp_zero_copy.hpp>
#include <uhd/transport/nirio_zero_copy.hpp>
#include <uhd/transport/nirio/nifpga_interface.h>
#include <uhd/transport/nirio/nifpga_image.h>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
using namespace nifpga_interface;
using namespace nirio_interface;
namespace asio = boost::asio;

/***********************************************************************
 * Discovery over the udp transport
 **********************************************************************/
static device_addrs_t b250_find_with_addr(const device_addr_t &hint)
{
    udp_simple::sptr comm = udp_simple::make_broadcast(
        hint["addr"], BOOST_STRINGIZE(X300_FW_COMMS_UDP_PORT));

    //load request struct
    x300_fw_comms_t request = x300_fw_comms_t();
    request.flags = uhd::htonx<boost::uint32_t>(X300_FW_COMMS_FLAGS_ACK);
    request.sequence = uhd::htonx<boost::uint32_t>(std::rand());

    //send request
    comm->send(asio::buffer(&request, sizeof(request)));

    //loop for replies until timeout
    device_addrs_t addrs;
    while (true)
    {
        char buff[X300_FW_COMMS_MTU] = {};
        const size_t nbytes = comm->recv(asio::buffer(buff), 0.050);
        if (nbytes == 0) break;
        const x300_fw_comms_t *reply = (const x300_fw_comms_t *)buff;
        if (request.flags != reply->flags) break;
        if (request.sequence != reply->sequence) break;
        device_addr_t new_addr;
        new_addr["type"] = "x300";
        new_addr["addr"] = comm->get_recv_addr();

        //Attempt to read the name from the EEPROM and perform filtering.
        //This operation can throw due to compatibility mismatch.
        try
        {
            wb_iface::sptr zpu_ctrl(new b250_ctrl_iface_enet(udp_simple::make_connected(new_addr["addr"], BOOST_STRINGIZE(X300_FW_COMMS_UDP_PORT))));
            i2c_core_100_wb32::sptr zpu_i2c = i2c_core_100_wb32::make(zpu_ctrl, I2C1_BASE);
            i2c_iface::sptr eeprom16 = zpu_i2c->eeprom16();
            const mboard_eeprom_t mb_eeprom(*eeprom16, "X300");
            new_addr["name"] = mb_eeprom["name"];
            new_addr["serial"] = mb_eeprom["serial"];
        }
        catch(const std::exception &)
        {
            //set these values as empty string so the device may still be found
            //and the filter's below can still operate on the discovered device
            new_addr["name"] = "";
            new_addr["serial"] = "";
        }
        //filter the discovered device below by matching optional keys
        if (
            (not hint.has_key("name")   or hint["name"]   == new_addr["name"]) and
            (not hint.has_key("serial") or hint["serial"] == new_addr["serial"])
        ){
            addrs.push_back(new_addr);
        }
    }

    return addrs;
}

static device_addrs_t b250_find(const device_addr_t &hint_)
{
    //we only do single device discovery for now
    const device_addr_t hint = separate_device_addr(hint_).at(0);

    device_addrs_t addrs;
    if (hint.has_key("type") and hint["type"] != "x300") return addrs;

    if (hint.has_key("transport") && hint["transport"] == "pcie")
    {
        niriok_proxy_vtr dev_proxy_vtr;
        niriok_proxy_factory::get_by_device_id(X300_PCIE_SSID, dev_proxy_vtr);
        BOOST_FOREACH(niriok_proxy &dev_proxy, dev_proxy_vtr)
        {
            device_addr_t new_addr;
            new_addr["type"] = "x300";
            new_addr["addr"] = boost::str(boost::format("RIO%u") % dev_proxy.get_interface_num());
            addrs.push_back(new_addr);
            dev_proxy.close();
        }
        return addrs;
    }
    else
    {
        //use the address given
        if (hint.has_key("addr"))
        {
            device_addrs_t reply_addrs = b250_find_with_addr(hint);
            BOOST_FOREACH(const device_addr_t &reply_addr, reply_addrs)
            {
                device_addrs_t new_addrs = b250_find_with_addr(reply_addr);
                addrs.insert(addrs.begin(), new_addrs.begin(), new_addrs.end());
            }
            return addrs;
        }

        //otherwise, no address was specified, send a broadcast on each interface
        BOOST_FOREACH(const if_addrs_t &if_addrs, get_if_addrs())
        {
            //avoid the loopback device
            if (if_addrs.inet == asio::ip::address_v4::loopback().to_string()) continue;

            //create a new hint with this broadcast address
            device_addr_t new_hint = hint;
            new_hint["addr"] = if_addrs.bcast;

            //call discover with the new hint and append results
            device_addrs_t new_addrs = b250_find(new_hint);
            addrs.insert(addrs.begin(), new_addrs.begin(), new_addrs.end());
        }
    }

    return addrs;
}

/***********************************************************************
 * Make
 **********************************************************************/
static device::sptr b250_make(const device_addr_t &device_addr)
{
    return device::sptr(new b250_impl(device_addr));
}

UHD_STATIC_BLOCK(register_b250_device)
{
    device::register_device(&b250_find, &b250_make);
}

static void b250_load_fw(const std::string &addr, const std::string &file_name)
{
    udp_simple::sptr comm = udp_simple::make_connected(
        addr, BOOST_STRINGIZE(X300_FW_COMMS_UDP_PORT));

    UHD_MSG(status) << "Loading firmware " << file_name << std::flush;

    //load file into memory
    std::ifstream fw_file(file_name.c_str());
    boost::uint32_t fw_file_buff[X300_FW_NUM_BYTES/sizeof(boost::uint32_t)];
    fw_file.read((char *)fw_file_buff, sizeof(fw_file_buff));
    fw_file.close();

    //poke the fw words into the upper bootram half
    size_t seq = 0;
    x300_fw_comms_t request = x300_fw_comms_t();
    char buff[X300_FW_COMMS_MTU] = {};

    request.flags = uhd::htonx<boost::uint32_t>(X300_FW_COMMS_FLAGS_POKE32 | X300_FW_COMMS_FLAGS_ACK);
    request.sequence = uhd::htonx<boost::uint32_t>(seq++);
    request.addr = uhd::htonx<boost::uint32_t>(SR_ADDR(BOOT_LDR_BASE, BL_ADDRESS));
    request.data = 0;
    comm->send(asio::buffer(&request, sizeof(request)));
    comm->recv(asio::buffer(buff));

    for (size_t i = 0; i < X300_FW_NUM_BYTES; i+=sizeof(boost::uint32_t))
    {
        //do ack for occasional backpressure
        const bool ack = (i & 0xf) == 0;

        //load request struct
        request.flags = uhd::htonx<boost::uint32_t>(X300_FW_COMMS_FLAGS_POKE32 | (ack?X300_FW_COMMS_FLAGS_ACK : 0));
        request.sequence = uhd::htonx<boost::uint32_t>(seq++);
        request.addr = uhd::htonx<boost::uint32_t>(SR_ADDR(BOOT_LDR_BASE, BL_DATA));
        request.data = uhd::htonx(uhd::byteswap(fw_file_buff[i/sizeof(boost::uint32_t)]));

        //send request
        comm->send(asio::buffer(&request, sizeof(request)));

        //do ack for occasional backpressure
        if (ack) comm->recv(asio::buffer(buff));

        if ((i & 0x1fff) == 0) UHD_MSG(status) << "." << std::flush;
    }

    UHD_MSG(status) << " done!" << std::endl;
}

b250_impl::b250_impl(const uhd::device_addr_t &dev_addr)
{
    UHD_MSG(status) << "X300 initialization sequence..." << std::endl;
    _async_md.reset(new async_md_type(1000/*messages deep*/));
    _tree = uhd::property_tree::make();
    _sid_framer = 0;
    _addr = dev_addr["addr"];
    _xport_path = dev_addr.has_key("transport") ? dev_addr["transport"] : "eth";
    _if_pkt_link_type = (_xport_path == "pcie") ? vrt::if_packet_info_t::LINK_TYPE_CHDR :
                                                  vrt::if_packet_info_t::LINK_TYPE_VRLP;

    if (_xport_path == "pcie") {
        UHD_MSG(status) << "Loading NiFpga lib...\n";
        nirio_status_not_fatal(nifpga_session::load_lib());

        UHD_MSG(status) << boost::format("Loading bitfile %s...\n") % nifpga_image::SIGNATURE;
        _rio_fpga_interface.reset(new nifpga_session("RIO0"));
        nirio_status status = 0;
        nirio_status_chain(_rio_fpga_interface->open(nifpga_image::BITFILE, nifpga_image::SIGNATURE), status);

        UHD_ASSERT_THROW(nirio_status_not_fatal(status));
    }

    BOOST_FOREACH(const std::string &key, dev_addr.keys())
    {
        if (key[0] == 'r') _recv_args[key] = dev_addr[key];
        if (key[0] == 's') _send_args[key] = dev_addr[key];
    }

    //extract the FW path for the B250
    //and live load fw over ethernet link
    if (dev_addr.has_key("fw"))
    {
        const std::string b250_fw_image = find_image_path(
            dev_addr.has_key("fw")? dev_addr["fw"] : B250_FW_FILE_NAME
        );
        b250_load_fw(_addr, b250_fw_image);
    }

    const std::vector<std::string> DB_NAMES = boost::assign::list_of("A")("B");

    //create basic communication
    UHD_MSG(status) << "Setup basic communication..." << std::endl;
    if (_xport_path == "pcie")
        _zpu_ctrl.reset(new b250_ctrl_iface_pcie(_rio_fpga_interface->get_kernel_proxy()));
    else
        _zpu_ctrl.reset(new b250_ctrl_iface_enet(udp_simple::make_connected(_addr, BOOST_STRINGIZE(X300_FW_COMMS_UDP_PORT))));

    _zpu_spi = spi_core_3000::make(_zpu_ctrl, SR_ADDR(SET0_BASE, ZPU_SR_SPI), SR_ADDR(SET0_BASE, ZPU_RB_SPI));
    _zpu_i2c = i2c_core_100_wb32::make(_zpu_ctrl, I2C1_BASE);
    _zpu_i2c->set_clock_rate(B250_BUS_CLOCK_RATE);

    ////////////////////////////////////////////////////////////////////
    // Initialize the properties tree
    ////////////////////////////////////////////////////////////////////
    _tree->create<std::string>("/name").set("X-Series Device");
    const fs_path mb_path = "/mboards/0";
    _tree->create<std::string>(mb_path / "name").set("X300");
    _tree->create<std::string>(mb_path / "codename").set("Yetti");
    _tree->create<std::string>(mb_path / "fpga_version").set("1.0");
    _tree->create<std::string>(mb_path / "fw_version").set("1.0");

    ////////////////////////////////////////////////////////////////////
    // setup the mboard eeprom
    ////////////////////////////////////////////////////////////////////
    UHD_MSG(status) << "Loading values from EEPROM..." << std::endl;
    i2c_iface::sptr eeprom16 = _zpu_i2c->eeprom16();
    const mboard_eeprom_t mb_eeprom(*eeprom16, "X300");
    _tree->create<mboard_eeprom_t>(mb_path / "eeprom")
        .set(mb_eeprom)
        .subscribe(boost::bind(&b250_impl::set_mb_eeprom, this, _1));

    ////////////////////////////////////////////////////////////////////
    // determine routing based on address match
    ////////////////////////////////////////////////////////////////////
    _router_dst_here = B250_XB_DST_E0; //some default if eeprom not match
    if (_addr == mb_eeprom["ip-addr0"]) _router_dst_here = B250_XB_DST_E0;
    if (_addr == mb_eeprom["ip-addr1"]) _router_dst_here = B250_XB_DST_E1;
    if (_addr == mb_eeprom["ip-addr2"]) _router_dst_here = B250_XB_DST_E0;
    if (_addr == mb_eeprom["ip-addr3"]) _router_dst_here = B250_XB_DST_E1;
    if (_xport_path == "pcie")          _router_dst_here = B250_XB_DST_PCI;

    ////////////////////////////////////////////////////////////////////
    // read dboard eeproms
    ////////////////////////////////////////////////////////////////////
    for (size_t i = 0; i < 8; i++)
    {
        if (i == 0 or i == 2) continue; //not used
        _db_eeproms[i].load(*_zpu_i2c, 0x50 | i);
    }

    ////////////////////////////////////////////////////////////////////
    // create clock control objects
    ////////////////////////////////////////////////////////////////////
    UHD_MSG(status) << "Setup RF frontend clocking..." << std::endl;

    //init shadow and clock source
    std::memset(&clock_control_regs, 0, sizeof(clock_control_regs));
    this->update_clock_source("internal");
    this->update_clock_control();

    _clock = b250_clock_ctrl::make(_zpu_spi, 1/*slaveno*/,
        dev_addr.cast<double>("master_clock_rate", B250_DEFAULT_TICK_RATE));
    _tree->create<double>(mb_path / "tick_rate")
        .publish(boost::bind(&b250_clock_ctrl::get_master_clock_rate, _clock));

    UHD_MSG(status) << "Radio 1x clock set to " << (_clock->get_master_clock_rate()/1e6) << std::dec << " MHz" << std::endl;

    ////////////////////////////////////////////////////////////////////
    // Create the GPSDO control
    ////////////////////////////////////////////////////////////////////
    static const boost::uint32_t dont_look_for_gpsdo = 0x1234abcdul;

    //otherwise if not disabled, look for the internal GPSDO
    if (_zpu_ctrl->peek32(SR_ADDR(X300_FW_SHMEM_BASE, X300_FW_SHMEM_GPSDO_STATUS)) != dont_look_for_gpsdo)
    {
        UHD_MSG(status) << "Detecting internal GPSDO.... " << std::flush;
        try
        {
            _gps = gps_ctrl::make(b250_make_uart_iface(_zpu_ctrl));
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
            _zpu_ctrl->poke32(SR_ADDR(X300_FW_SHMEM_BASE, X300_FW_SHMEM_GPSDO_STATUS), dont_look_for_gpsdo);
        }
    }

    ////////////////////////////////////////////////////////////////////
    //clear router?
    ////////////////////////////////////////////////////////////////////
    for (size_t i = 0; i < 512; i++) _zpu_ctrl->poke32(SR_ADDR(SETXB_BASE, i), 0);
  
    ////////////////////////////////////////////////////////////////////
    // setup radios
    ////////////////////////////////////////////////////////////////////
    UHD_MSG(status) << "Initialize Radio control..." << std::endl;
    this->setup_radio(0, DB_NAMES[0]);
    this->setup_radio(1, DB_NAMES[1]);

    ////////////////////////////////////////////////////////////////////
    // front panel gpio
    ////////////////////////////////////////////////////////////////////
    _fp_gpio = gpio_core_200::make(_radio_perifs[0].ctrl, TOREG(SR_FP_GPIO), RB32_FP_GPIO);
    const std::vector<std::string> GPIO_ATTRS = boost::assign::list_of("CTRL")("DDR")("OUT")("ATR_0X")("ATR_RX")("ATR_TX")("ATR_XX");
    BOOST_FOREACH(const std::string &attr, GPIO_ATTRS)
    {
        _tree->create<boost::uint64_t>(mb_path / "gpio" / "FP0" / attr)
            .set(0)
            .subscribe(boost::bind(&b250_impl::set_fp_gpio, this, attr, _1));
    }
    _tree->create<boost::uint64_t>(mb_path / "gpio" / "FP0" / "READBACK")
        .publish(boost::bind(&b250_impl::get_fp_gpio, this, "READBACK"));

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
        .subscribe(boost::bind(&b250_impl::update_time_source, this, _1));
    _tree->create<bool>(mb_path / "time_source" / "output")
        .subscribe(boost::bind(&b250_impl::set_time_source_out, this, _1))
        .set(true);
    static const std::vector<std::string> time_sources = boost::assign::list_of("internal")("external")("gpsdo");
    _tree->create<std::vector<std::string> >(mb_path / "time_source" / "options").set(time_sources);
    //setup reference source props
    _tree->create<std::string>(mb_path / "clock_source" / "value")
        .subscribe(boost::bind(&b250_impl::update_clock_source, this, _1));
    _tree->create<bool>(mb_path / "clock_source" / "output")
        .subscribe(boost::bind(&b250_clock_ctrl::set_ref_out, _clock, _1))
        .set(true);
    static const std::vector<std::string> clock_sources = boost::assign::list_of("internal")("external")("gpsdo");
    _tree->create<std::vector<std::string> >(mb_path / "clock_source" / "options").set(clock_sources);

    ////////////////////////////////////////////////////////////////////
    // create frontend mapping
    ////////////////////////////////////////////////////////////////////
    _tree->create<subdev_spec_t>(mb_path / "rx_subdev_spec")
        .subscribe(boost::bind(&b250_impl::update_rx_subdev_spec, this, _1));
    _tree->create<subdev_spec_t>(mb_path / "tx_subdev_spec")
        .subscribe(boost::bind(&b250_impl::update_tx_subdev_spec, this, _1));

    ////////////////////////////////////////////////////////////////////
    // and do the misc mboard sensors
    ////////////////////////////////////////////////////////////////////
    _tree->create<sensor_value_t>(mb_path / "sensors" / "ref_locked")
        .publish(boost::bind(&b250_impl::get_ref_locked, this));

    ////////////////////////////////////////////////////////////////////
    // do some post-init tasks
    ////////////////////////////////////////////////////////////////////
    _tree->access<double>(mb_path / "tick_rate") //now subscribe the clock rate setter
        .subscribe(boost::bind(&b250_impl::set_tick_rate, this, _1))
        .subscribe(boost::bind(&b250_impl::update_tick_rate, this, _1))
        .set(_clock->get_master_clock_rate());

    subdev_spec_t rx_fe_spec, tx_fe_spec;
    rx_fe_spec.push_back(subdev_spec_pair_t("A", _tree->list(mb_path / "dboards" / "A" / "rx_frontends").at(0)));
    rx_fe_spec.push_back(subdev_spec_pair_t("B", _tree->list(mb_path / "dboards" / "B" / "rx_frontends").at(0)));
    tx_fe_spec.push_back(subdev_spec_pair_t("A", _tree->list(mb_path / "dboards" / "A" / "tx_frontends").at(0)));
    tx_fe_spec.push_back(subdev_spec_pair_t("B", _tree->list(mb_path / "dboards" / "B" / "tx_frontends").at(0)));

    _tree->access<subdev_spec_t>(mb_path / "rx_subdev_spec").set(rx_fe_spec);
    _tree->access<subdev_spec_t>(mb_path / "tx_subdev_spec").set(tx_fe_spec);

    //GPS installed: use external ref, time, and init time spec
    if (_gps and _gps->gps_detected())
    {
        UHD_MSG(status) << "Setting references to the internal GPSDO" << std::endl;
        _tree->access<std::string>(mb_path / "time_source" / "value").set("gpsdo");
        _tree->access<std::string>(mb_path / "clock_source" / "value").set("gpsdo");
        UHD_MSG(status) << "Initializing time to the internal GPSDO" << std::endl;
        const time_t tp = time_t(_gps->get_sensor("gps_time").to_int()+1);
        _tree->access<time_spec_t>(mb_path / "time" / "pps").set(time_spec_t(tp));
    }
    else
    {
        _tree->access<std::string>(mb_path / "time_source" / "value").set("external");
        _tree->access<std::string>(mb_path / "clock_source" / "value").set("external");
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        if (this->get_ref_locked().to_bool())
        {
            UHD_MSG(status) << "Setting references to external sources" << std::endl;
        }
        else
        {
            UHD_MSG(status) << "Setting references to internal sources" << std::endl;
            _tree->access<std::string>(mb_path / "time_source" / "value").set("internal");
            _tree->access<std::string>(mb_path / "clock_source" / "value").set("internal");
        }
    }
}

b250_impl::~b250_impl(void)
{
    UHD_SAFE_CALL
    (
        _radio_perifs[0].ctrl->poke32(TOREG(SR_MISC_OUTS), (1 << 2)); //disable/reset ADC/DAC
        _radio_perifs[1].ctrl->poke32(TOREG(SR_MISC_OUTS), (1 << 2)); //disable/reset ADC/DAC

//        if (_xport_path == "pcie") {
//            _rio_fpga_interface->close(true);
//            nifpga_session::unload_lib();
//        }
    )
}

static void check_adc(wb_iface::sptr iface, const boost::uint32_t val)
{
    boost::uint32_t adc_rb = iface->peek32(RB32_RX);
    adc_rb ^= 0xfffc0000; //adapt for I inversion in FPGA
    //UHD_MSG(status) << "adc_rb " << std::hex << adc_rb << "  val " << std::hex << val << std::endl;
    UHD_ASSERT_THROW(adc_rb == val);
}

void b250_impl::setup_radio(const size_t i, const std::string &db_name)
{
    const fs_path mb_path = "/mboards/0";
    radio_perifs_t &perif = _radio_perifs[i];
    const size_t dspno = i;

    ////////////////////////////////////////////////////////////////////
    // radio control
    ////////////////////////////////////////////////////////////////////
    UHD_HERE();
    uint8_t dest = (i == 0)? B250_XB_DST_R0 : B250_XB_DST_R1;
    boost::uint32_t ctrl_sid;
    zero_copy_if::sptr ctrl_xport = this->make_transport(_addr, _xport_path, dest, B250_RADIO_DEST_PREFIX_CTRL, device_addr_t(), ctrl_sid);
    perif.ctrl = radio_ctrl_core_3000::make(_if_pkt_link_type, ctrl_xport, ctrl_xport, ctrl_sid, db_name);
    perif.ctrl->poke32(TOREG(SR_MISC_OUTS), (1 << 2)); //reset adc + dac
    perif.ctrl->poke32(TOREG(SR_MISC_OUTS),  (1 << 1) | (1 << 0)); //out of reset + dac enable

    this->register_loopback_self_test(perif.ctrl);

    perif.spi = spi_core_3000::make(perif.ctrl, TOREG(SR_SPI), RB32_SPI);
    perif.adc = b250_adc_ctrl::make(perif.spi, DB_ADC_SEN);
    perif.dac = b250_dac_ctrl::make(perif.spi, DB_DAC_SEN, _clock->get_master_clock_rate());
    perif.leds = gpio_core_200_32wo::make(perif.ctrl, TOREG(SR_LEDS));

    ////////////////////////////////////////////////////////////////
    // ADC self test
    ////////////////////////////////////////////////////////////////
    perif.adc->set_test_word("ones", "ones"); check_adc(perif.ctrl, 0xfffcfffc);
    perif.adc->set_test_word("zeros", "zeros"); check_adc(perif.ctrl, 0x00000000);
    perif.adc->set_test_word("ones", "zeros"); check_adc(perif.ctrl, 0xfffc0000);
    perif.adc->set_test_word("zeros", "ones"); check_adc(perif.ctrl, 0x0000fffc);
    for (size_t k = 0; k < 14; k++)
    {
        perif.adc->set_test_word("zeros", "custom", 1 << k);
        check_adc(perif.ctrl, 1 << (k+2));
    }
    for (size_t k = 0; k < 14; k++)
    {
        perif.adc->set_test_word("custom", "zeros", 1 << k);
        check_adc(perif.ctrl, 1 << (k+18));
    }
    perif.adc->set_test_word("normal", "normal");

    ////////////////////////////////////////////////////////////////
    // create codec control objects
    ////////////////////////////////////////////////////////////////
    _tree->create<int>(mb_path / "rx_codecs" / db_name / "gains"); //phony property so this dir exists
    _tree->create<int>(mb_path / "tx_codecs" / db_name / "gains"); //phony property so this dir exists
    _tree->create<std::string>(mb_path / "rx_codecs" / db_name / "name").set("ads62p48");
    _tree->create<std::string>(mb_path / "tx_codecs" / db_name / "name").set("ad9146");

    _tree->create<meta_range_t>(mb_path / "rx_codecs" / db_name / "gains" / "digital" / "range").set(meta_range_t(0, 6.0, 0.5));
    _tree->create<double>(mb_path / "rx_codecs" / db_name / "gains" / "digital" / "value")
        .subscribe(boost::bind(&b250_adc_ctrl::set_gain, perif.adc, _1)).set(0);

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
        .subscribe(boost::bind(&b250_impl::update_rx_samp_rate, this, dspno, _1))
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
        .subscribe(boost::bind(&b250_impl::update_tx_samp_rate, this, dspno, _1))
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

    ////////////////////////////////////////////////////////////////////
    // create RF frontend interfacing
    ////////////////////////////////////////////////////////////////////
    const size_t j = (db_name == "B")? 0x2 : 0x0;
    _tree->create<dboard_eeprom_t>(mb_path / "dboards" / db_name / "rx_eeprom")
        .set(_db_eeproms[B250_DB0_RX_EEPROM | j])
        .subscribe(boost::bind(&b250_impl::set_db_eeprom, this, (0x50 | B250_DB0_RX_EEPROM | j), _1));
    _tree->create<dboard_eeprom_t>(mb_path / "dboards" / db_name / "tx_eeprom")
        .set(_db_eeproms[B250_DB0_TX_EEPROM | j])
        .subscribe(boost::bind(&b250_impl::set_db_eeprom, this, (0x50 | B250_DB0_TX_EEPROM | j), _1));
    _tree->create<dboard_eeprom_t>(mb_path / "dboards" / db_name / "gdb_eeprom")
        .set(_db_eeproms[B250_DB0_GDB_EEPROM | j])
        .subscribe(boost::bind(&b250_impl::set_db_eeprom, this, (0x50 | B250_DB0_GDB_EEPROM | j), _1));

    //create a new dboard interface
    b250_dboard_iface_config_t db_config;
    db_config.gpio = gpio_core_200::make(perif.ctrl, TOREG(SR_GPIO), RB32_GPIO);
    db_config.spi = perif.spi;
    db_config.rx_spi_slaveno = DB_RX_SEN;
    db_config.tx_spi_slaveno = DB_TX_SEN;
    db_config.i2c = _zpu_i2c;
    db_config.clock = _clock;
    db_config.which_rx_clk = (db_name == "A")? B250_CLOCK_WHICH_DB0_RX : B250_CLOCK_WHICH_DB1_RX;
    db_config.which_tx_clk = (db_name == "A")? B250_CLOCK_WHICH_DB0_TX : B250_CLOCK_WHICH_DB1_TX;
    _dboard_ifaces[db_name] = b250_make_dboard_iface(db_config);

    //create a new dboard manager
    _tree->create<dboard_iface::sptr>(mb_path / "dboards" / db_name / "iface").set(_dboard_ifaces[db_name]);
    _dboard_managers[db_name] = dboard_manager::make(
        _db_eeproms[B250_DB0_RX_EEPROM | j].id,
        _db_eeproms[B250_DB0_TX_EEPROM | j].id,
        _db_eeproms[B250_DB0_GDB_EEPROM | j].id,
        _dboard_ifaces[db_name],
        _tree->subtree(mb_path / "dboards" / db_name)
    );

    //now that dboard is created -- register into rx antenna event
    const std::string fe_name = _tree->list(mb_path / "dboards" / db_name / "rx_frontends").front();
    _tree->access<std::string>(mb_path / "dboards" / db_name / "rx_frontends" / fe_name / "antenna" / "value")
        .subscribe(boost::bind(&b250_impl::update_atr_leds, this, i, _1));
    this->update_atr_leds(i, ""); //init anyway, even if never called
}


uhd::transport::zero_copy_if::sptr b250_impl::make_transport(
    const std::string& addr,
    const std::string& xport_path,
    const uint8_t& destination,
    const uint8_t& prefix,
    const uhd::device_addr_t& args,
    boost::uint32_t& sid
)
{
    zero_copy_if::sptr xport;

    sid_config_t config;
    config.router_addr_there    = B250_DEVICE_THERE;
    config.dst_prefix           = prefix;
    config.router_dst_there     = destination;
    config.router_dst_here      = _router_dst_here;
    sid = this->allocate_sid(config, xport_path);

    if (xport_path == "pcie") {
        uint32_t chan = (destination == B250_XB_DST_R0) ? 0 : 1;
        uint32_t instance = prefix + (chan * 3);
        xport = nirio_zero_copy::make(_rio_fpga_interface, instance, args);
    } else {
        //make a new transport - fpga has no idea how to talk to use on this yet
        xport = udp_zero_copy::make(addr, BOOST_STRINGIZE(X300_VITA_UDP_PORT), args);

        //clear the ethernet dispatcher's udp port
        //NOT clearing this, the dispatcher is now intelligent
        //_zpu_ctrl->poke32(SR_ADDR(SET0_BASE, (ZPU_SR_ETHINT0+8+3)), 0);

        //send a mini packet with SID into the ZPU
        //ZPU will reprogram the ethernet framer
        UHD_LOG << "programming packet for new xport on "
            << addr << std::hex << "sid 0x" << sid << std::dec << std::endl;
        managed_send_buffer::sptr mb = xport->get_send_buff();
        mb->cast<boost::uint32_t *>()[0] = uhd::htonx(sid);
        mb->commit(4);
        mb.reset();

        //reprogram the ethernet dispatcher's udp port (should be safe to always set)
        UHD_LOG << "reprogram the ethernet dispatcher's udp port" << std::endl;
        _zpu_ctrl->poke32(SR_ADDR(SET0_BASE, (ZPU_SR_ETHINT0+8+3)), X300_VITA_UDP_PORT);
        _zpu_ctrl->poke32(SR_ADDR(SET0_BASE, (ZPU_SR_ETHINT1+8+3)), X300_VITA_UDP_PORT);

        //Do a peek to an arbitrary address to guarantee that the
        //ethernet framer has been programmed before we return.
        _zpu_ctrl->peek32(0);
    }
    return xport;
}


boost::uint32_t b250_impl::allocate_sid(const sid_config_t &config, std::string xport_path)
{
    const boost::uint32_t stream = (config.dst_prefix | (config.router_dst_there << 2)) & 0xff;

    const boost::uint32_t sid = 0
        | (B250_DEVICE_HERE << 24)
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

    // Program the B250 to recognise it's own local address.
    _zpu_ctrl->poke32(SR_ADDR(SET0_BASE, ZPU_SR_XB_LOCAL), config.router_addr_there);
    // Program CAM entry for outgoing packets matching a B250 resource (for example a Radio)
    // This type of packet does matches the XB_LOCAL address and is looked up in the upper half of the CAM
    _zpu_ctrl->poke32(SR_ADDR(SETXB_BASE, 256 + (stream)), config.router_dst_there);
    // Program CAM entry for returning packets to us (for example GR host via Eth0)
    // This type of packet does not match the XB_LOCAL address and is looked up in the lower half of the CAM
    _zpu_ctrl->poke32(SR_ADDR(SETXB_BASE, 0 + (B250_DEVICE_HERE)), config.router_dst_here);

    if (xport_path == "pcie") {
        uint32_t chan = config.router_dst_there == B250_XB_DST_R0 ? 0 : 1;
        uint32_t router_config_word = ((_sid_framer & 0xff) << 16) | (config.dst_prefix + (chan * 3));
        _rio_fpga_interface->get_kernel_proxy().poke(PCIE_ROUTER_REG(0), router_config_word);
    }

    UHD_LOG << std::hex
        << "done router config for sid 0x" << sid
        << std::dec << std::endl;

    //increment for next setup
    _sid_framer++;

    return sid;
}

void b250_impl::update_atr_leds(const size_t fe, const std::string &rx_ant)
{
    const bool is_txrx = (rx_ant == "TX/RX");
    const int rx_led = (1 << 2);
    const int txrx_led = (1 << 1);
    const int tx_led = (1 << 0);
    _radio_perifs[fe].leds->set_atr_reg(dboard_iface::ATR_REG_IDLE, 0);
    _radio_perifs[fe].leds->set_atr_reg(dboard_iface::ATR_REG_RX_ONLY, is_txrx? txrx_led : rx_led);
    _radio_perifs[fe].leds->set_atr_reg(dboard_iface::ATR_REG_TX_ONLY, tx_led);
    _radio_perifs[fe].leds->set_atr_reg(dboard_iface::ATR_REG_FULL_DUPLEX, rx_led | tx_led);
}

void b250_impl::set_tick_rate(const double rate)
{
    BOOST_FOREACH(radio_perifs_t &perif, _radio_perifs)
    {
        perif.time64->set_tick_rate(rate);
        perif.time64->self_test();
    }
}

void b250_impl::register_loopback_self_test(wb_iface::sptr iface)
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

void b250_impl::set_time_source_out(const bool enb)
{
    clock_control_regs.pps_out_enb = enb? 1 : 0;
    this->update_clock_control();
}

void b250_impl::update_clock_control(void)
{
    const size_t reg = clock_control_regs.clock_source
        | (clock_control_regs.pps_select << 2)
        | (clock_control_regs.pps_out_enb << 3)
    ;
    _zpu_ctrl->poke32(SR_ADDR(SET0_BASE, ZPU_SR_CLOCK_CTRL), reg);
}

void b250_impl::update_clock_source(const std::string &source)
{
    clock_control_regs.clock_source = 0;
    if (source == "internal") clock_control_regs.clock_source = 0x2;
    else if (source == "external") clock_control_regs.clock_source = 0x0;
    else if (source == "gpsdo") clock_control_regs.clock_source = 0x3;
    else throw uhd::key_error("update_clock_source: unknown source: " + source);
    this->update_clock_control();
}

void b250_impl::update_time_source(const std::string &source)
{
    if (source == "internal"){}
    else if (source == "external"){}
    else if (source == "gpsdo"){}
    else throw uhd::key_error("update_time_source: unknown source: " + source);
    clock_control_regs.pps_select = (source == "external")? 1 : 0;
    this->update_clock_control();
}

sensor_value_t b250_impl::get_ref_locked(void)
{
    const bool lock = (_zpu_ctrl->peek32(SR_ADDR(SET0_BASE, ZPU_RB_CLK_STATUS)) & (1 << 2)) != 0;
    return sensor_value_t("Ref", lock, "locked", "unlocked");
}

void b250_impl::set_db_eeprom(const size_t addr, const uhd::usrp::dboard_eeprom_t &db_eeprom)
{
    db_eeprom.store(*_zpu_i2c, addr);
}

void b250_impl::set_mb_eeprom(const mboard_eeprom_t &mb_eeprom)
{
    i2c_iface::sptr eeprom16 = _zpu_i2c->eeprom16();
    mb_eeprom.commit(*eeprom16, "X300");
}

boost::uint64_t b250_impl::get_fp_gpio(const std::string &)
{
    return boost::uint64_t(_fp_gpio->read_gpio(dboard_iface::UNIT_RX));
}

template <typename T>
static T shadow_it(const T &shadow, const T &value, const T &mask)
{
    return (shadow & ~mask) | (value & mask);
}

void b250_impl::set_fp_gpio(const std::string &attr, const boost::uint64_t setting)
{
    const boost::uint32_t value = boost::uint32_t(setting >> 0);
    const boost::uint32_t mask = boost::uint32_t(setting >> 32);
    const boost::uint32_t shadow = boost::uint32_t(_tree->access<boost::uint64_t>("/mboards/0/gpio/FP0/"+attr).get());
    const boost::uint32_t new_value = shadow_it(shadow, value, mask);
    if (attr == "CTRL") return _fp_gpio->set_pin_ctrl(dboard_iface::UNIT_RX, new_value);
    if (attr == "DDR") return _fp_gpio->set_gpio_ddr(dboard_iface::UNIT_RX, new_value);
    if (attr == "OUT") return _fp_gpio->set_gpio_out(dboard_iface::UNIT_RX, new_value);
    if (attr == "ATR_0X") return _fp_gpio->set_atr_reg(dboard_iface::UNIT_RX, dboard_iface::ATR_REG_IDLE, new_value);
    if (attr == "ATR_RX") return _fp_gpio->set_atr_reg(dboard_iface::UNIT_RX, dboard_iface::ATR_REG_RX_ONLY, new_value);
    if (attr == "ATR_TX") return _fp_gpio->set_atr_reg(dboard_iface::UNIT_RX, dboard_iface::ATR_REG_TX_ONLY, new_value);
    if (attr == "ATR_XX") return _fp_gpio->set_atr_reg(dboard_iface::UNIT_RX, dboard_iface::ATR_REG_FULL_DUPLEX, new_value);
}
