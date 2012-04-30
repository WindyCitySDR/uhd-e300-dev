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

using namespace uhd;
using namespace uhd::usrp;

void b200_impl::update_rates(void)
{
    //TODO
}

void b200_impl::update_rx_subdev_spec(const uhd::usrp::subdev_spec_t &)
{
    //TODO
}

void b200_impl::update_tx_subdev_spec(const uhd::usrp::subdev_spec_t &)
{
    //TODO
}


void b200_impl::update_rx_samp_rate(const size_t, const double rate)
{
    //TODO
}

void b200_impl::update_tx_samp_rate(const size_t, const double rate)
{
    //TODO
}

/***********************************************************************
 * Async Data
 **********************************************************************/
bool b200_impl::recv_async_msg(
    async_metadata_t &async_metadata, double timeout
){
    return _ctrl->pop_async_msg(async_metadata, timeout);
}

/***********************************************************************
 * Receive streamer
 **********************************************************************/
rx_streamer::sptr b200_impl::get_rx_stream(const uhd::stream_args_t &args_)
{
    stream_args_t args = args_;
    //TODO
}

/***********************************************************************
 * Transmit streamer
 **********************************************************************/
tx_streamer::sptr b200_impl::get_tx_stream(const uhd::stream_args_t &args_)
{
    stream_args_t args = args_;
    //TODO
}
