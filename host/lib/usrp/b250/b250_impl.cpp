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
#include <uhd/usrp/subdev_spec.hpp>
#include <uhd/usrp/mboard_eeprom.hpp>
#include <uhd/transport/if_addrs.hpp>
#include <boost/foreach.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/functional/hash.hpp>
#include <boost/assign/list_of.hpp>
#include <fstream>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace asio = boost::asio;

/***********************************************************************
 * Discovery over the udp transport
 **********************************************************************/
static device_addrs_t b250_find_with_addr(const device_addr_t &dev_addr)
{
    udp_simple::sptr comm = udp_simple::make_broadcast(
        dev_addr["addr"], BOOST_STRINGIZE(B250_FW_COMMS_UDP_PORT));

    //load request struct
    b250_fw_comms_t request = b250_fw_comms_t();
    request.flags = uhd::htonx<boost::uint32_t>(B250_FW_COMMS_FLAGS_ACK);
    request.sequence = 0;

    //send request
    comm->send(asio::buffer(&request, sizeof(request)));

    //loop for replies until timeout
    device_addrs_t addrs;
    while (true)
    {
        char buff[B250_FW_COMMS_MTU] = {};
        const size_t nbytes = comm->recv(asio::buffer(buff));
        if (nbytes == 0) break;
        device_addr_t new_addr;
        new_addr["type"] = "b250";
        new_addr["addr"] = comm->get_recv_addr();
        addrs.push_back(new_addr);
    }

    return addrs;
}

static device_addrs_t b250_find(const device_addr_t &hint)
{
    device_addrs_t addrs;
    if (hint.has_key("type") and hint["type"] != "b250") return addrs;

    //use the address given
    if (hint.has_key("addr"))
    {
        device_addrs_t reply_addrs = b250_find_with_addr(hint);
        BOOST_FOREACH(const device_addr_t &reply_addr, reply_addrs)
        {
            if (b250_find_with_addr(reply_addr).empty()) continue;
            addrs.push_back(reply_addr);
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
        addr, BOOST_STRINGIZE(B250_FW_COMMS_UDP_PORT));

    UHD_MSG(status) << "Loading firmware " << file_name << std::flush;

    //load file into memory
    std::ifstream fw_file(file_name.c_str());
    boost::uint32_t fw_file_buff[B250_FW_NUM_BYTES/sizeof(boost::uint32_t)];
    fw_file.read((char *)fw_file_buff, sizeof(fw_file_buff));
    fw_file.close();

    //poke the fw words into the upper bootram half
    size_t seq = 0;
    for (size_t i = 0; i < B250_FW_NUM_BYTES; i+=sizeof(boost::uint32_t))
    {
        //do ack for occasional backpressure
        const bool ack = (i & 0xf) == 0;

        //load request struct
        b250_fw_comms_t request = b250_fw_comms_t();
        request.flags = uhd::htonx<boost::uint32_t>(B250_FW_COMMS_FLAGS_POKE32 | (ack?B250_FW_COMMS_FLAGS_ACK : 0));
        request.sequence = uhd::htonx<boost::uint32_t>(seq++);
        request.addr = uhd::htonx<boost::uint32_t>(B250_FW_NUM_BYTES+i);
        request.data = uhd::htonx(uhd::byteswap(fw_file_buff[i/sizeof(boost::uint32_t)]));

        //send request
        comm->send(asio::buffer(&request, sizeof(request)));

        //do ack for occasional backpressure
        char buff[B250_FW_COMMS_MTU] = {};
        if (ack) comm->recv(asio::buffer(buff));

        if ((i & 0xfff) == 0) UHD_MSG(status) << "." << std::flush;
    }

    UHD_MSG(status) << " done!" << std::endl;
}

b250_impl::b250_impl(const uhd::device_addr_t &dev_addr)
{
    _tree = uhd::property_tree::make();
    _last_sid = 0;
    _addr = dev_addr["addr"];

    //extract the FW path for the B250
    //and live load fw over ethernet link
    const std::string b250_fw_image = find_image_path(
        dev_addr.has_key("fw")? dev_addr["fw"] : B250_FW_FILE_NAME
    );
    b250_load_fw(dev_addr["addr"], b250_fw_image);

    const std::vector<std::string> DB_NAMES = boost::assign::list_of("A")("B");

    //create basic communication
    _zpu_ctrl.reset(new b250_ctrl_iface(udp_simple::make_connected(dev_addr["addr"], BOOST_STRINGIZE(B250_FW_COMMS_UDP_PORT))));
    _zpu_spi = spi_core_3000::make(_zpu_ctrl, SR_ADDR(SET0_BASE, ZPU_SR_SPI), SR_ADDR(SET0_BASE, ZPU_RB_SPI));
    _zpu_i2c = i2c_core_100_wb32::make(_zpu_ctrl, I2C1_BASE);
    _zpu_i2c->set_clock_rate(B250_BUS_CLOCK_RATE);

    ////////////////////////////////////////////////////////////////////
    // Initialize the properties tree
    ////////////////////////////////////////////////////////////////////
    _tree->create<std::string>("/name").set("B-Series Device");
    const fs_path mb_path = "/mboards/0";
    _tree->create<std::string>(mb_path / "name").set("B250");
    _tree->create<std::string>(mb_path / "codename").set("Yetti");

    ////////////////////////////////////////////////////////////////////
    // setup the mboard eeprom
    ////////////////////////////////////////////////////////////////////
    // TODO
//    const mboard_eeprom_t mb_eeprom(*_iface, mboard_eeprom_t::MAP_B100);
    mboard_eeprom_t mb_eeprom;
    mb_eeprom["name"] = "TODO"; //FIXME with real eeprom values
    mb_eeprom["serial"] = "TODO"; //FIXME with real eeprom values
    _tree->create<mboard_eeprom_t>(mb_path / "eeprom")
        .set(mb_eeprom);

    ////////////////////////////////////////////////////////////////////
    // create clock control objects
    ////////////////////////////////////////////////////////////////////
    UHD_HERE();
    _tree->create<double>(mb_path / "tick_rate");
    this->setup_ad9510_clock(_zpu_spi);

    //clear router?
    for (size_t i = 0; i < 512; i++) _zpu_ctrl->poke32(SR_ADDR(SETXB_BASE, i), 0);

    ////////////////////////////////////////////////////////////////////
    // radio control 0
    ////////////////////////////////////////////////////////////////////
    UHD_HERE();
    sid_config_t ctrl0_config;
    ctrl0_config.router_addr_there = B250_DEVICE_THERE;
    ctrl0_config.dst_prefix = B250_RADIO_DEST_PREFIX_CTRL;
    ctrl0_config.router_dst_there = B250_XB_DST_R0;
    ctrl0_config.router_dst_here = B250_XB_DST_E0;
    const boost::uint32_t ctrl0_sid = this->allocate_sid(ctrl0_config);
    udp_zero_copy::sptr r0_ctrl_xport = this->make_transport(dev_addr["addr"], ctrl0_sid);
    _radio_ctrl0 = b250_ctrl::make(r0_ctrl_xport, ctrl0_sid);
    _radio_ctrl0->poke32(TOREG(SR_MISC_OUTS), (1 << 2)); //reset adc + dac
    _radio_ctrl0->poke32(TOREG(SR_MISC_OUTS), (1 << 1) | (1 << 0)); //out of reset + dac enable

    this->register_loopback_self_test(_radio_ctrl0);

    _radio_spi0 = spi_core_3000::make(_radio_ctrl0, TOREG(SR_SPI), RB32_SPI);
    _adc_ctrl0 = b250_adc_ctrl::make(_radio_spi0, DB_ADC_SEN);
    this->set_ad9146_dac(_radio_spi0);

    ////////////////////////////////////////////////////////////////////
    // radio control 1
    ////////////////////////////////////////////////////////////////////
    UHD_HERE();
    /*
    sid_config_t ctrl1_config;
    ctrl1_config.router_addr_there = B250_DEVICE_THERE;
    ctrl1_config.dst_prefix = B250_RADIO_DEST_PREFIX_CTRL;
    ctrl1_config.router_dst_there = B250_XB_DST_R1;
    ctrl1_config.router_dst_here = B250_XB_DST_E0;
    const boost::uint32_t ctrl1_sid = this->allocate_sid(ctrl1_config);
    udp_zero_copy::sptr r1_ctrl_xport = this->make_transport(dev_addr["addr"], ctrl1_sid);
    _radio_ctrl1 = b250_ctrl::make(r1_ctrl_xport, ctrl1_sid);
    this->register_loopback_self_test(_radio_ctrl1);
    */

    ////////////////////////////////////////////////////////////////
    // create codec control objects
    ////////////////////////////////////////////////////////////////
    BOOST_FOREACH (const std::string &db_name, DB_NAMES)
    {
        _tree->create<int>(mb_path / "rx_codecs" / db_name / "gains"); //phony property so this dir exists
        _tree->create<int>(mb_path / "tx_codecs" / db_name / "gains"); //phony property so this dir exists
        _tree->create<std::string>(mb_path / "rx_codecs" / db_name / "name").set("ads62p44");
        _tree->create<std::string>(mb_path / "tx_codecs" / db_name / "name").set("ad9146");
    }

    ////////////////////////////////////////////////////////////////////
    // create rx dsp control objects
    ////////////////////////////////////////////////////////////////////
    _rx_framer = rx_vita_core_3000::make(_radio_ctrl0, TOREG(SR_RX_CTRL+4), TOREG(SR_RX_CTRL));
    _rx_framer->set_tick_rate(B250_RADIO_CLOCK_RATE);
    for (size_t dspno = 0; dspno < 1; dspno++)
    {
        const fs_path rx_dsp_path = mb_path / "rx_dsps" / str(boost::format("%u") % dspno);
        _tree->create<meta_range_t>(rx_dsp_path / "rate" / "range")
            .set(meta_range_t(120e6, 120e6));
        _tree->create<double>(rx_dsp_path / "rate" / "value")
            .set(120e6);
        _tree->create<double>(rx_dsp_path / "freq" / "value")
            .set(0.0);
        _tree->create<meta_range_t>(rx_dsp_path / "freq/range")
            .set(meta_range_t(0.0, 0.0));
        _tree->create<stream_cmd_t>(rx_dsp_path / "stream_cmd")
            .subscribe(boost::bind(&rx_vita_core_3000::issue_stream_command, _rx_framer, _1));
    }

    ////////////////////////////////////////////////////////////////////
    // create tx dsp control objects
    ////////////////////////////////////////////////////////////////////
    _tx_deframer = tx_vita_core_3000::make(_radio_ctrl0, TOREG(SR_TX_CTRL+2), TOREG(SR_TX_CTRL));
    _tx_deframer->set_tick_rate(B250_RADIO_CLOCK_RATE);
    for (size_t dspno = 0; dspno < 1; dspno++)
    {
        const fs_path tx_dsp_path = mb_path / "tx_dsps" / str(boost::format("%u") % dspno);
        _tree->create<meta_range_t>(tx_dsp_path / "rate" / "range")
            .set(meta_range_t(120e6, 120e6));
        _tree->create<double>(tx_dsp_path / "rate" / "value")
            .set(120e6);
        _tree->create<double>(tx_dsp_path / "freq" / "value")
            .set(0.0);
        _tree->create<meta_range_t>(tx_dsp_path / "freq" / "range")
            .set(meta_range_t(0.0, 0.0));
    }

    ////////////////////////////////////////////////////////////////////
    // create time control objects
    ////////////////////////////////////////////////////////////////////
    time_core_3000::readback_bases_type time64_rb_bases;
    time64_rb_bases.rb_now = RB64_TIME_NOW;
    time64_rb_bases.rb_pps = RB64_TIME_PPS;
    _time64 = time_core_3000::make(_radio_ctrl0, TOREG(SR_TIME), time64_rb_bases);
    _time64->set_tick_rate(B250_RADIO_CLOCK_RATE);
    _time64->self_test();
    _tree->create<time_spec_t>(mb_path / "time" / "now")
        .publish(boost::bind(&time_core_3000::get_time_now, _time64))
        .subscribe(boost::bind(&time_core_3000::set_time_now, _time64, _1));
    _tree->create<time_spec_t>(mb_path / "time" / "pps")
        .publish(boost::bind(&time_core_3000::get_time_last_pps, _time64))
        .subscribe(boost::bind(&time_core_3000::set_time_next_pps, _time64, _1));
    //setup time source props
    _tree->create<std::string>(mb_path / "time_source" / "value")
        .subscribe(boost::bind(&time_core_3000::set_time_source, _time64, _1));
    _tree->create<std::vector<std::string> >(mb_path / "time_source" / "options")
        .publish(boost::bind(&time_core_3000::get_time_sources, _time64));
    //setup reference source props
    _tree->create<std::string>(mb_path / "clock_source" / "value")
        .subscribe(boost::bind(&b250_impl::update_clock_source, this, _1));
    static const std::vector<std::string> clock_sources = boost::assign::list_of("internal")("external");
    _tree->create<std::vector<std::string> >(mb_path / "clock_source" / "options").set(clock_sources);

    ////////////////////////////////////////////////////////////////////
    // create frontend mapping
    ////////////////////////////////////////////////////////////////////
    _tree->create<subdev_spec_t>(mb_path / "rx_subdev_spec");
    _tree->create<subdev_spec_t>(mb_path / "tx_subdev_spec");

    ////////////////////////////////////////////////////////////////////
    // create RF frontend interfacing
    ////////////////////////////////////////////////////////////////////

    //read all the eeproms, some may not be present
    dboard_eeprom_t db_eeproms[8];
    for (size_t i = 0; i < 8; i++)
    {
        db_eeproms[i].load(*_zpu_i2c, 0x50 | i);
    }

    BOOST_FOREACH (const std::string &db_name, DB_NAMES)
    {
        const size_t j = (db_name == "B")? 0x2 : 0x0;
        _tree->create<dboard_eeprom_t>(mb_path / "dboards" / db_name / "rx_eeprom")
            .set(db_eeproms[B250_DB0_RX_EEPROM | j])
            .subscribe(boost::bind(&b250_impl::set_db_eeprom, this, (0x50 | B250_DB0_RX_EEPROM | j), _1));
        _tree->create<dboard_eeprom_t>(mb_path / "dboards" / db_name / "tx_eeprom")
            .set(db_eeproms[B250_DB0_TX_EEPROM | j])
            .subscribe(boost::bind(&b250_impl::set_db_eeprom, this, (0x50 | B250_DB0_TX_EEPROM | j), _1));
        _tree->create<dboard_eeprom_t>(mb_path / "dboards" / db_name / "gdb_eeprom")
            .set(db_eeproms[B250_DB0_GDB_EEPROM | j])
            .subscribe(boost::bind(&b250_impl::set_db_eeprom, this, (0x50 | B250_DB0_GDB_EEPROM | j), _1));

        //create a new dboard interface and manager
        _dboard_ifaces[db_name] = b250_make_dboard_iface();
        _tree->create<dboard_iface::sptr>(mb_path / "dboards" / db_name / "iface").set(_dboard_ifaces[db_name]);
        _dboard_managers[db_name] = dboard_manager::make(
            db_eeproms[B250_DB0_RX_EEPROM | j].id,
            db_eeproms[B250_DB0_TX_EEPROM | j].id,
            db_eeproms[B250_DB0_GDB_EEPROM | j].id,
            _dboard_ifaces[db_name],
            _tree->subtree(mb_path / "dboards" / db_name)
        );
    }

    ////////////////////////////////////////////////////////////////////
    // and do the misc mboard sensors
    ////////////////////////////////////////////////////////////////////
    //none for now...
    _tree->create<int>(mb_path / "sensors"); //phony property so this dir exists

    ////////////////////////////////////////////////////////////////////
    // do some post-init tasks
    ////////////////////////////////////////////////////////////////////

    _tree->access<double>(mb_path / "tick_rate") //now subscribe the clock rate setter
        .set(B250_RADIO_CLOCK_RATE);

    _tree->access<subdev_spec_t>(mb_path / "rx_subdev_spec").set(subdev_spec_t("A:" + _tree->list(mb_path / "dboards" / "A" / "rx_frontends").at(0)));
    _tree->access<subdev_spec_t>(mb_path / "tx_subdev_spec").set(subdev_spec_t("A:" + _tree->list(mb_path / "dboards" / "A" / "tx_frontends").at(0)));
    _tree->access<std::string>(mb_path / "clock_source" / "value").set("internal");
    _tree->access<std::string>(mb_path / "time_source" / "value").set("internal");

}

b250_impl::~b250_impl(void)
{
    //NOP
}

uhd::transport::udp_zero_copy::sptr b250_impl::make_transport(const std::string &addr, const boost::uint32_t sid)
{
    //make a new transport - fpga has no idea how to talk to use on this yet
    udp_zero_copy::sptr xport = udp_zero_copy::make(addr, BOOST_STRINGIZE(B250_VITA_UDP_PORT));

    //clear the ethernet dispatcher's udp port
    _zpu_ctrl->poke32(SR_ADDR(SET0_BASE, (ZPU_SR_ETHINT0+8+3)), 0);

    //send a mini packet with SID into the ZPU
    //ZPU will reprogram the ethernet framer
    managed_send_buffer::sptr mb = xport->get_send_buff();
    mb->cast<boost::uint32_t *>()[0] = uhd::htonx(sid);
    mb->commit(4);
    mb.reset();

    //reprogram the ethernet dispatcher's udp port
    _zpu_ctrl->poke32(SR_ADDR(SET0_BASE, (ZPU_SR_ETHINT0+8+3)), B250_VITA_UDP_PORT);

    return xport;
}

boost::uint32_t b250_impl::allocate_sid(const sid_config_t &config)
{
    _last_sid++;

    const boost::uint32_t stream = (config.dst_prefix | (_last_sid << 2)) & 0xff;
    UHD_VAR(stream);

    const boost::uint32_t sid = 0
        | (4 << 24)
        | (stream << 16)
        | (config.router_addr_there << 8)
        | (stream << 0)
    ;

    _zpu_ctrl->poke32(SR_ADDR(SET0_BASE, ZPU_SR_XB_LOCAL), config.router_addr_there);
    _zpu_ctrl->poke32(SR_ADDR(SETXB_BASE, 256 + (stream)), config.router_dst_there);
    _zpu_ctrl->poke32(SR_ADDR(SETXB_BASE, 0   + (stream)), config.router_dst_here);

    return sid;
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

void b250_impl::setup_ad9510_clock(spi_core_3000::sptr iface)
{
    #define write_ad9510_reg(addr, data) \
        iface->write_spi(1, spi_config_t::EDGE_RISE, ((addr) << 8) | (data), 24)
    write_ad9510_reg(0x3C, 0x08); // TEST_CLK on
    write_ad9510_reg(0x3D, 0x02); // NC off
    write_ad9510_reg(0x3E, 0x08); // RX_CLK on
    write_ad9510_reg(0x3F, 0x08); // TX_CLK on
    write_ad9510_reg(0x40, 0x02); // FPGA_CLK on
    write_ad9510_reg(0x41, 0x01); // NC off
    write_ad9510_reg(0x42, 0x01); // MIMO off
    write_ad9510_reg(0x43, 0x01); // NC off
    write_ad9510_reg(0x49, 0x80); // TEST_CLK bypass div
    write_ad9510_reg(0x4B, 0x80); // NC bypass div
    write_ad9510_reg(0x4D, 0x80); // RX_CLK bypass div
    write_ad9510_reg(0x4F, 0x80); // TX_CLK bypass div
    write_ad9510_reg(0x51, 0x80); // FPGA_CLK bypass div
    write_ad9510_reg(0x53, 0x80); // NC bypass div
    write_ad9510_reg(0x55, 0x80); // MIMO bypass div
    write_ad9510_reg(0x57, 0x80); // NC bypass div
    write_ad9510_reg(0x5a, 0x01); // Apply settings
}

void b250_impl::set_ad9146_dac(spi_core_3000::sptr iface)
{
    spi_config_t spi_config(spi_config_t::EDGE_FALL);
    spi_config.mosi_edge = spi_config_t::EDGE_RISE;
    spi_config.miso_edge = spi_config_t::EDGE_RISE;

    #define write_ad9146_reg(addr, data) \
        iface->write_spi(DB_DAC_SEN, spi_config, ((addr) << 8) | (data), 16)
    #define read_ad9146_reg(addr) \
        (iface->read_spi(DB_DAC_SEN, spi_config, ((addr) << 8) | (1 << 15), 16) & 0xffff)
    //
    write_ad9146_reg(0x0, 1 << 5); //reset
    write_ad9146_reg(0x0, 1 << 7); //config + out of reset
    write_ad9146_reg(0x1, 0x0); //out of power down
    UHD_MSG(status) << std::hex << read_ad9146_reg(0x7f) << std::endl;
    UHD_MSG(status) << std::hex << read_ad9146_reg(0x49) << std::endl;
    UHD_MSG(status) << std::hex << read_ad9146_reg(0x4A) << std::endl;
    UHD_MSG(status) << std::hex << read_ad9146_reg(0xE) << std::endl;
    UHD_MSG(status) << std::hex << read_ad9146_reg(0xF) << std::endl;
    UHD_MSG(status) << std::hex << read_ad9146_reg(0x0) << std::endl;
    write_ad9146_reg(0x8, 0x3f);
    UHD_MSG(status) << std::hex << read_ad9146_reg(0x8) << std::endl;
}

void b250_impl::update_clock_source(const std::string &)
{
    
}

void b250_impl::set_db_eeprom(const size_t addr, const uhd::usrp::dboard_eeprom_t &db_eeprom)
{
    db_eeprom.store(*_zpu_i2c, addr);
}
