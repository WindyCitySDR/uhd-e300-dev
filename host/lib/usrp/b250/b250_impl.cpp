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
#include <uhd/utils/images.hpp>
#include <fstream>

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

    //create basic communication
    ctrl.reset(new b250_ctrl_iface(udp_simple::make_connected(dev_addr["addr"], "12345")));

    //extract the FW path for the B250
    //and live load fw over ethernet link
    const std::string b250_fw_image = find_image_path(
        dev_addr.has_key("fw")? dev_addr["fw"] : B250_FW_FILE_NAME
    );
    this->load_fw(b250_fw_image);
}

b250_impl::~b250_impl(void)
{
    //NOP
}

void b250_impl::load_fw(const std::string &file_name)
{
    UHD_MSG(status) << "Loading firmware " << file_name << std::flush;
    std::ifstream fw_file(file_name.c_str());
    boost::uint32_t fw_file_buff[B250_FW_NUM_BYTES/sizeof(boost::uint32_t)];
    fw_file.read((char *)fw_file_buff, sizeof(fw_file_buff));
    fw_file.close();
    for (size_t i = 0; i < B250_FW_NUM_BYTES; i+=sizeof(boost::uint32_t))
    {
        const boost::uint32_t data = uhd::byteswap(fw_file_buff[i/sizeof(boost::uint32_t)]);
        ctrl->poke32(B250_FW_NUM_BYTES+i, data);
        if ((i & 0xfff) == 0) UHD_MSG(status) << "." << std::flush;
    }
    UHD_MSG(status) << " done!" << std::endl;
}
