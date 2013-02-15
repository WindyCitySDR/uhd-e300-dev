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

#include <uhd/device.hpp>
#include <uhd/property_tree.hpp>
#include <boost/weak_ptr.hpp>

#ifndef INCLUDED_E200_IMPL_HPP
#define INCLUDED_E200_IMPL_HPP

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
    uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &args){}
    uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &args){}
    bool recv_async_msg(uhd::async_metadata_t &, double){}

private:

    //device properties interface
    uhd::property_tree::sptr _tree;
    uhd::property_tree::sptr get_tree(void) const
    {
        return _tree;
    }

};

#endif /* INCLUDED_E200_IMPL_HPP */
