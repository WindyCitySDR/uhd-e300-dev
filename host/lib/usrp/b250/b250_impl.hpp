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

#ifndef INCLUDED_B250_IMPL_HPP
#define INCLUDED_B250_IMPL_HPP

#include <uhd/property_tree.hpp>
#include <uhd/device.hpp>
#include <uhd/usrp/dboard_manager.hpp>
#include <uhd/usrp/dboard_eeprom.hpp>
#include <uhd/usrp/subdev_spec.hpp>
#include <uhd/types/sensors.hpp>
#include "wb_iface.hpp"
#include "b250_clock_ctrl.hpp"
#include "b250_fw_common.h"
#include "b250_ctrl.hpp"
#include "b250_fw_ctrl.hpp"
#include <uhd/transport/udp_zero_copy.hpp>
#include "spi_core_3000.hpp"
#include "b250_adc_ctrl.hpp"
#include "b250_dac_ctrl.hpp"
#include "rx_vita_core_3000.hpp"
#include "tx_vita_core_3000.hpp"
#include "time_core_3000.hpp"
#include "rx_dsp_core_3000.hpp"
#include "tx_dsp_core_3000.hpp"
#include "i2c_core_100_wb32.hpp"
#include "gpio_core_200.hpp"
#include <boost/weak_ptr.hpp>

static const size_t B250_TX_FC_PKT_WINDOW = 100;
static const std::string B250_FW_FILE_NAME = "/home/jblum/src/ettus/b250_dev/uhd/firmware/b250/build/b250/b250_main.bin";
static const double B250_RADIO_CLOCK_RATE = 120e6;
static const double B250_BUS_CLOCK_RATE = 200e6;

#define B250_RADIO_DEST_PREFIX_TX 0
#define B250_RADIO_DEST_PREFIX_CTRL 1
#define B250_RADIO_DEST_PREFIX_RX 2

#define B250_XB_DST_E0 0
#define B250_XB_DST_E1 1
#define B250_XB_DST_R0 2
#define B250_XB_DST_R1 3

#define B250_DEVICE_THERE 2
#define B250_DEVICE_HERE 0

//eeprom addrs for various boards
enum
{
    B250_DB0_RX_EEPROM = 0x5,
    B250_DB0_TX_EEPROM = 0x4,
    B250_DB0_GDB_EEPROM = 0x1,
    B250_DB1_RX_EEPROM = 0x7,
    B250_DB1_TX_EEPROM = 0x6,
    B250_DB1_GDB_EEPROM = 0x3,
};

struct b250_dboard_iface_config_t
{
    gpio_core_200::sptr gpio;
    spi_core_3000::sptr spi;
    size_t rx_spi_slaveno;
    size_t tx_spi_slaveno;
    i2c_core_100_wb32::sptr i2c;
    b250_clock_ctrl::sptr clock;
    b250_clock_which_t which_rx_clk;
    b250_clock_which_t which_tx_clk;
};

uhd::usrp::dboard_iface::sptr b250_make_dboard_iface(const b250_dboard_iface_config_t &);

class b250_impl : public uhd::device
{
public:
    b250_impl(const uhd::device_addr_t &);
    ~b250_impl(void);

    //the io interface
    uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &);
    uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &);
    bool recv_async_msg(uhd::async_metadata_t &, double);

private:
    uhd::property_tree::sptr _tree;
    //device properties interface
    uhd::property_tree::sptr get_tree(void) const
    {
        return _tree;
    }

    uhd::transport::udp_zero_copy::sptr make_transport(const std::string &addr, const boost::uint32_t sid);
    void register_loopback_self_test(wb_iface::sptr iface);

    std::string _addr;

    //perifs in the zpu
    wb_iface::sptr _zpu_ctrl;
    spi_core_3000::sptr _zpu_spi;
    i2c_core_100_wb32::sptr _zpu_i2c;

    //perifs in the radio core
    b250_ctrl::sptr _radio_ctrl0;
    b250_ctrl::sptr _radio_ctrl1;
    spi_core_3000::sptr _radio_spi0;
    spi_core_3000::sptr _radio_spi1;
    b250_adc_ctrl::sptr _adc_ctrl0;
    b250_adc_ctrl::sptr _adc_ctrl1;
    b250_dac_ctrl::sptr _dac_ctrl0;
    b250_dac_ctrl::sptr _dac_ctrl1;
    time_core_3000::sptr _time64;
    b250_clock_ctrl::sptr _clock;

    size_t _last_sid;
    struct sid_config_t
    {
        boost::uint8_t router_addr_there;
        boost::uint8_t dst_prefix; //2bits
        boost::uint8_t router_dst_there;
        boost::uint8_t router_dst_here;
    };
    boost::uint32_t allocate_sid(const sid_config_t &config);

    rx_vita_core_3000::sptr _rx_framer;
    rx_dsp_core_3000::sptr _rx_dsp;
    tx_vita_core_3000::sptr _tx_deframer;
    tx_dsp_core_3000::sptr _tx_dsp;

    uhd::dict<size_t, boost::weak_ptr<uhd::rx_streamer> > _rx_streamers;
    uhd::dict<size_t, boost::weak_ptr<uhd::tx_streamer> > _tx_streamers;

    uhd::dict<std::string, uhd::usrp::dboard_manager::sptr> _dboard_managers;
    uhd::dict<std::string, uhd::usrp::dboard_iface::sptr> _dboard_ifaces;

    void update_rx_subdev_spec(const uhd::usrp::subdev_spec_t &spec);
    void update_tx_subdev_spec(const uhd::usrp::subdev_spec_t &spec);

    void update_tick_rate(const double);
    void update_rx_samp_rate(const size_t, const double);
    void update_tx_samp_rate(const size_t, const double);

    void update_clock_source(const std::string &);
    uhd::sensor_value_t get_ref_locked(void);
    void set_db_eeprom(const size_t, const uhd::usrp::dboard_eeprom_t &);
};

#endif /* INCLUDED_B250_IMPL_HPP */
