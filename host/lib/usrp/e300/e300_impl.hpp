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

#ifndef INCLUDED_E300_IMPL_HPP
#define INCLUDED_E300_IMPL_HPP

#include <uhd/device.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/usrp/subdev_spec.hpp>
#include <uhd/usrp/mboard_eeprom.hpp>
#include <uhd/usrp/dboard_eeprom.hpp>
#include <uhd/types/serial.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include "e300_fifo_config.hpp"
#include "radio_ctrl_core_3000.hpp"
#include "rx_vita_core_3000.hpp"
#include "tx_vita_core_3000.hpp"
#include "time_core_3000.hpp"
#include "rx_dsp_core_3000.hpp"
#include "tx_dsp_core_3000.hpp"
#include "ad9361_ctrl.hpp"
#include "gpio_core_200.hpp"

static const std::string E300_FPGA_FILE_NAME = "usrp_e300_fpga.bin";

static std::string E300_SERVER_RX_PORT = "321756";
static std::string E300_SERVER_TX_PORT = "321757";
static std::string E300_SERVER_CTRL_PORT = "321758";
static std::string E300_SERVER_CODEC_PORT = "321759";

static const double E300_DEFAULT_TICK_RATE = 32e6;

static const double E300_RX_SW_BUFF_FULLNESS = 0.5;        //Buffer should be half full

/*!
 * USRP-E300 implementation guts:
 * The implementation details are encapsulated here.
 * Handles properties on the mboard, dboard, dsps...
 */
class e300_impl : public uhd::device
{
public:
    //structors
    e300_impl(const uhd::device_addr_t &);
    virtual ~e300_impl(void);

    //the io interface
    boost::mutex _stream_spawn_mutex;
    uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &);
    uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &);
    bool recv_async_msg(uhd::async_metadata_t &, double);

private:
    bool _network_mode;

    void load_fpga_image(const std::string &path);

    e300_fifo_interface::sptr _fifo_iface;

    void register_loopback_self_test(uhd::wb_iface::sptr iface);

    //perifs in the radio core
    struct radio_perifs_t
    {
        radio_ctrl_core_3000::sptr ctrl;
        gpio_core_200_32wo::sptr atr0;
        gpio_core_200_32wo::sptr atr1;
        time_core_3000::sptr time64;
        rx_vita_core_3000::sptr framer;
        rx_dsp_core_3000::sptr ddc;
        tx_vita_core_3000::sptr deframer;
        tx_dsp_core_3000::sptr duc;

        uhd::transport::zero_copy_if::sptr send_ctrl_xport;
        uhd::transport::zero_copy_if::sptr recv_ctrl_xport;
        uhd::transport::zero_copy_if::sptr tx_data_xport;
        uhd::transport::zero_copy_if::sptr tx_flow_xport;
        uhd::transport::zero_copy_if::sptr rx_data_xport;
        uhd::transport::zero_copy_if::sptr rx_flow_xport;
        boost::weak_ptr<uhd::rx_streamer> rx_streamer;
        boost::weak_ptr<uhd::tx_streamer> tx_streamer;
    };
    radio_perifs_t _radio_perifs[1]; //TODO 1 for now
    void setup_radio(const size_t which_radio);

    double _tick_rate;
    double get_tick_rate(void){return _tick_rate;}
    double set_tick_rate(const double rate);

    void update_tick_rate(const double);
    void update_rx_samp_rate(const size_t, const double);
    void update_tx_samp_rate(const size_t, const double);

    void update_time_source(const std::string &);
    void update_clock_source(const std::string &);

    void update_rx_subdev_spec(const uhd::usrp::subdev_spec_t &spec);
    void update_tx_subdev_spec(const uhd::usrp::subdev_spec_t &spec);

    ad9361_ctrl_transport::sptr _codec_xport;
    ad9361_ctrl::sptr _codec_ctrl;
    void codec_loopback_self_test(uhd::wb_iface::sptr iface);

    //server stuff for network access
    void run_server(const std::string &port, const std::string &what);

    //frontend cache so we can update gpios
    struct fe_control_settings_t
    {
        fe_control_settings_t(void)
        {
            rx_ant = "RX2";
            tx_enb = false;
            rx_enb = false;
            rx_freq = 1e9;
            tx_freq = 1e9;
        }
        std::string rx_ant;
        bool tx_enb;
        bool rx_enb;
        double rx_freq;
        double tx_freq;
    };
    fe_control_settings_t _fe_control_settings[2];
    void update_atrs(const size_t &fe);
    void update_antenna_sel(const std::string &fe, const std::string &ant);
    void update_fe_lo_freq(const std::string &fe, const double freq);
    void update_active_frontends(void);

    uhd::spi_iface::sptr make_spidev(const std::string &device);
};

#endif /* INCLUDED_E300_IMPL_HPP */
