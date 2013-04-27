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

#ifndef INCLUDED_E200_IMPL_HPP
#define INCLUDED_E200_IMPL_HPP

#include <uhd/device.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/usrp/subdev_spec.hpp>
#include <uhd/types/serial.hpp>
#include <boost/weak_ptr.hpp>
#include "e200_fifo_config.hpp"
#include "e200_ctrl.hpp"
#include "rx_vita_core_3000.hpp"
#include "tx_vita_core_3000.hpp"
#include "time_core_3000.hpp"
#include "rx_dsp_core_3000.hpp"
#include "tx_dsp_core_3000.hpp"
#include "ad9361_ctrl.hpp"
#include "gpio_core_200.hpp"

static const std::string E200_FPGA_FILE_NAME = "usrp_e200_fpga.bin";

static std::string E200_SERVER_RX_PORT = "321756";
static std::string E200_SERVER_TX_PORT = "321757";
static std::string E200_SERVER_CTRL_PORT = "321758";
static std::string E200_SERVER_CODEC_PORT = "321759";

uhd::spi_iface::sptr e200_make_aux_spi_iface(void);

/*!
 * USRP-E200 implementation guts:
 * The implementation details are encapsulated here.
 * Handles properties on the mboard, dboard, dsps...
 */
class e200_impl : public uhd::device
{
public:
    //structors
    e200_impl(const uhd::device_addr_t &);
    ~e200_impl(void);

    //the io interface
    uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &);
    uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &);
    bool recv_async_msg(uhd::async_metadata_t &, double);

private:

    //device properties interface
    uhd::property_tree::sptr _tree;
    uhd::property_tree::sptr get_tree(void) const
    {
        return _tree;
    }

    void load_fpga_image(const std::string &path);

    e200_fifo_interface::sptr _fifo_iface;
    e200_ctrl::sptr _radio_ctrl;
    uhd::spi_iface::sptr _aux_spi;

    void register_loopback_self_test(wb_iface::sptr iface);

    rx_vita_core_3000::sptr _rx_framer;
    rx_dsp_core_3000::sptr _rx_dsp;
    tx_vita_core_3000::sptr _tx_deframer;
    tx_dsp_core_3000::sptr _tx_dsp;
    uhd::dict<size_t, boost::weak_ptr<uhd::rx_streamer> > _rx_streamers;
    uhd::dict<size_t, boost::weak_ptr<uhd::tx_streamer> > _tx_streamers;

    uhd::transport::zero_copy_if::sptr _send_ctrl_xport, _recv_ctrl_xport;

    uhd::transport::zero_copy_if::sptr _tx_data_xport, _tx_flow_xport;
    uhd::transport::zero_copy_if::sptr _rx_data_xport, _rx_flow_xport;

    time_core_3000::sptr _time64;
    gpio_core_200_32wo::sptr _atr0;
    gpio_core_200_32wo::sptr _atr1;

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

    //! this is half-assed, but temporary
    ad9361_ctrl::sptr _codec_ctrl;
    ad9361_ctrl_iface_sptr _codec_ctrl_iface;
    inline void transact_spi(
        unsigned char *tx_data,
        size_t,
        unsigned char *rx_data,
        size_t
    ){
        boost::uint32_t data = 0;
        data |= (tx_data[0] << 16);
        data |= (tx_data[1] << 8);
        data |= (tx_data[2] << 0);
        const boost::uint32_t result = _aux_spi->transact_spi(0, uhd::spi_config_t::EDGE_RISE, data, 24, rx_data != NULL);
        if (rx_data)
        {
            rx_data[0] = result & 0xff;
            rx_data[1] = result & 0xff;
            rx_data[2] = result & 0xff;
        }
    }

    //server stuff for network access
    void run_server(const std::string &port, const std::string &what);

    //frontend cache so we can update gpios
    struct fe_control_settings_t
    {
        std::string rx_ant;
        bool tx_enb;
        bool rx_enb;
        double rx_freq;
        double tx_freq;
    };
    uhd::dict<std::string, fe_control_settings_t> _fe_control_settings;
    void update_atrs(const std::string &which);

};

#endif /* INCLUDED_E200_IMPL_HPP */
