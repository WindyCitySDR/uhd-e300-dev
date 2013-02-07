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

#ifndef INCLUDED_B200_IMPL_HPP
#define INCLUDED_B200_IMPL_HPP

#include "b200_iface.hpp"
#include "b200_ctrl.hpp"
#include "b200_codec_ctrl.hpp"
#include "rx_vita_core_3000.hpp"
#include "tx_vita_core_3000.hpp"
#include "time_core_3000.hpp"
#include "gpio_core_200.hpp"
#include <uhd/device.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/utils/pimpl.hpp>
#include <uhd/utils/tasks.hpp>
#include <uhd/types/dict.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/clock_config.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/usrp/mboard_eeprom.hpp>
#include <uhd/usrp/subdev_spec.hpp>
#include <uhd/transport/usb_zero_copy.hpp>
#include <uhd/transport/bounded_buffer.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/asio.hpp>
#include <uhd/utils/tasks.hpp>

namespace asio = boost::asio;

static const std::string     B200_FW_FILE_NAME = "/home/jblum/Desktop/b200_rev2_5656.hex";
static const std::string     B200_FPGA_FILE_NAME = "usrp_b200_fpga.bin";
static const boost::uint16_t B200_FW_COMPAT_NUM = 0x03;
static const boost::uint16_t B200_FPGA_COMPAT_NUM = 0x09;
static const size_t          B200_MAX_PKT_BYTE_LIMIT = 2048*4;
static const double          B200_LINK_RATE_BPS = (5e9)/8; //practical link rate (5 Gbps)
static const boost::uint32_t B200_CTRL_MSG_SID = 0x00010000;
static const boost::uint32_t B200_RESP_MSG_SID = 0x00000001;
static const boost::uint32_t B200_TX_DATA_SID_BASE = 0x00020000;
static const boost::uint32_t B200_TX_MSG_SID_BASE = 0x00000002;
static const boost::uint32_t B200_RX_DATA_SID_BASE = 0x00040000;
static const size_t          B200_NUM_RX_FE = 2;
static const size_t          B200_NUM_TX_FE = 2;


/***********************************************************************
 * The B200 Capability Constants
 **********************************************************************/
static const uhd::meta_range_t b200_samp_range(200e3, 56e6);


//! Implementation guts
class b200_impl : public uhd::device
{
public:
    //structors
    b200_impl(const uhd::device_addr_t &);
    ~b200_impl(void);

    //the io interface
    uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &args);
    uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &args);
    bool recv_async_msg(uhd::async_metadata_t &, double);

private:
    uhd::property_tree::sptr _tree;

    //controllers
    b200_iface::sptr _iface;
    b200_ctrl::sptr _ctrl;
    b200_codec_ctrl::sptr _codec_ctrl;
    rx_vita_core_3000::sptr _rx_framer;
    tx_vita_core_3000::sptr _tx_deframer;
    time_core_3000::sptr _time64;
    gpio_core_200_32wo::sptr _atr0;
    gpio_core_200_32wo::sptr _atr1;
    double _tick_rate;
    double get_clock_rate(void){return _tick_rate;}

    //transports
    uhd::transport::zero_copy_if::sptr _data_transport;
    uhd::transport::zero_copy_if::sptr _ctrl_transport;
    uhd::task::sptr _async_task;
    void handle_async_task(void);

    //device properties interface
    uhd::property_tree::sptr get_tree(void) const
    {
        return _tree;
    }

    boost::weak_ptr<uhd::rx_streamer> _rx_streamer;
    boost::weak_ptr<uhd::tx_streamer> _tx_streamer;
    uhd::transport::bounded_buffer<uhd::async_metadata_t> _async_md;

    void issue_stream_cmd(const size_t dspno, const uhd::stream_cmd_t &);

    void register_loopback_self_test(void);
    void codec_loopback_self_test(void);
    void time_loopback_self_test(void);
    void set_mb_eeprom(const uhd::usrp::mboard_eeprom_t &);
    void check_fw_compat(void);
    void check_fpga_compat(void);
    void update_rx_subdev_spec(const uhd::usrp::subdev_spec_t &);
    void update_tx_subdev_spec(const uhd::usrp::subdev_spec_t &);
    void update_clock_source(const std::string &);
    void update_streamer_rates(const double rate);
    void update_bandsel(const std::string& which, double freq);
    void update_antenna_sel(const std::string& which, const std::string &ant);

    struct gpio_state {
        boost::uint32_t  tx_bandsel_a, tx_bandsel_b, rx_bandsel_a, rx_bandsel_b, rx_bandsel_c;
        boost::uint32_t  mimo_tx, mimo_rx, ext_ref_enable, pps_fpga_out_enable, gps_out_enable, gps_ref_enable;

        gpio_state() {
            tx_bandsel_a = 0;
            tx_bandsel_b = 0;
            rx_bandsel_a = 0;
            rx_bandsel_b = 0;
            rx_bandsel_c = 0;
            mimo_tx = 0;
            mimo_rx = 0;
            ext_ref_enable = 0;
            pps_fpga_out_enable = 0;
            gps_out_enable = 0;
            gps_ref_enable = 0;
        }
    } _gpio_state;

    void update_gpio_state(void);

    bool _enable_tx1, _enable_tx2, _enable_rx1, _enable_rx2;
    void update_enables(void);

    //no dsp in fpga
    double get_dsp_freq(void){return 0.0;}

    uhd::meta_range_t get_dsp_freq_range(void){return uhd::meta_range_t(0.0, 0.0);};
    uhd::meta_range_t get_possible_rates(void){return b200_samp_range;};

    double set_sample_rate(const double rate);

    uhd::task::sptr _server;
    void run_server(void);
};

#endif /* INCLUDED_B200_IMPL_HPP */
