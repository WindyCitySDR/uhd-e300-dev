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

#ifndef INCLUDED_B200_IFACE_HPP
#define INCLUDED_B200_IFACE_HPP

#include <uhd/transport/usb_control.hpp>
#include <uhd/types/serial.hpp> //i2c iface
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>

class b200_iface: boost::noncopyable, public uhd::i2c_iface{
public:
    typedef boost::shared_ptr<b200_iface> sptr;

    /*!
     * Make a b200 interface object from a control transport
     * \param ctrl_transport a USB control transport
     * \return a new b200 interface object
     */
    static sptr make(uhd::transport::usb_control::sptr ctrl_transport);

    //! load a firmware image
    virtual void load_firmware(const std::string &path) = 0;

    //! load an FPGA image
    virtual void load_fpga(const std::string &path) = 0;

};


#endif /* INCLUDED_B200_IFACE_HPP */
