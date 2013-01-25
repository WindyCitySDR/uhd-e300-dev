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
#include <uhd/utils/static.hpp>
#include <uhd/utils/msg.hpp>

using namespace uhd;
using namespace uhd::transport;

/***********************************************************************
 * Discovery over the udp transport
 **********************************************************************/
static device_addrs_t b250_find(const device_addr_t &)
{
    device_addr_t dummy_addr;
    dummy_addr["addr"] = "192.168.10.2";
    device_addrs_t addrs;
    addrs.push_back(dummy_addr);
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

b250_impl::b250_impl(const uhd::device_addr_t &dev_addr)
{
    _tree = uhd::property_tree::make();
    ctrl.reset(new b250_ctrl_iface(udp_simple::make_connected(dev_addr["addr"], "12345")));
    UHD_MSG(status) << ctrl->peek32(0) << std::endl;
    UHD_MSG(status) << ctrl->peek32(4) << std::endl;
    UHD_MSG(status) << ctrl->peek32(8) << std::endl;
    UHD_MSG(status) << ctrl->peek32(12) << std::endl;
}

b250_impl::~b250_impl(void)
{
    //NOP
}
