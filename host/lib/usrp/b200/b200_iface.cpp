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

#include "b200_iface.hpp"

using namespace uhd;
using namespace uhd::transport;

/***********************************************************************
 * The implementation class
 **********************************************************************/
class b200_iface_impl : public b200_iface{
public:

    b200_iface_impl(usb_control::sptr ctrl_transport):
        _ctrl_transport(ctrl_transport)
    {
        //NOP
    }

    void write_i2c(boost::uint8_t addr, const byte_vector_t &bytes)
    {
        //TODO
    }

    byte_vector_t read_i2c(boost::uint8_t addr, size_t num_bytes)
    {
        //TODO
    }

    void load_firmware(const std::string &path)
    {
        //TODO
    }

    void load_fpga(const std::string &path)
    {
        //TODO
    }

private:
    usb_control::sptr _ctrl_transport;
};

/***********************************************************************
 * Make an instance of the implementation
 **********************************************************************/
b200_iface::sptr b200_iface::make(usb_control::sptr ctrl_transport)
{
    return sptr(new b200_iface_impl(ctrl_transport));
}
