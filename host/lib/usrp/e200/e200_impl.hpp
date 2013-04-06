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
#include <boost/weak_ptr.hpp>
#include "e200_fifo_config.hpp"
#include "e200_ctrl.hpp"
#include "rx_vita_core_3000.hpp"
#include "tx_vita_core_3000.hpp"
#include "time_core_3000.hpp"
#include "rx_dsp_core_3000.hpp"
#include "tx_dsp_core_3000.hpp"

static const std::string E200_FPGA_FILE_NAME = "usrp_e200_fpga.bin";
static const double E200_RADIO_CLOCK_RATE = 50e6; //FIXME fixed for now

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

    void register_loopback_self_test(wb_iface::sptr iface);

    rx_vita_core_3000::sptr _rx_framer;
    rx_dsp_core_3000::sptr _rx_dsp;
    tx_vita_core_3000::sptr _tx_deframer;
    tx_dsp_core_3000::sptr _tx_dsp;
    uhd::dict<size_t, boost::weak_ptr<uhd::rx_streamer> > _rx_streamers;
    uhd::dict<size_t, boost::weak_ptr<uhd::tx_streamer> > _tx_streamers;

    uhd::transport::zero_copy_if::sptr _tx_data_xport, _tx_flow_xport;
    uhd::transport::zero_copy_if::sptr _rx_data_xport, _rx_flow_xport;

    time_core_3000::sptr _time64;
    
    void update_time_source(const std::string &);
    void update_clock_source(const std::string &);

};

#endif /* INCLUDED_E200_IMPL_HPP */
