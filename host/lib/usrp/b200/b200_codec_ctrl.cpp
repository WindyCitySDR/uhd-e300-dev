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

#include "b200_codec_ctrl.hpp"

using namespace uhd;
using namespace uhd::transport;

/***********************************************************************
 * The implementation class
 **********************************************************************/
class b200_codec_ctrl_impl : public b200_codec_ctrl{
public:

    b200_codec_ctrl_impl(spi_iface::sptr spiface, usb_control::sptr usb_ctrl)
    {
        _spiface = spiface;
        _usb_ctrl = usb_ctrl;
    }

    std::vector<std::string> get_gain_names(const std::string &which)
    {
        //TODO
    }

    double set_gain(const std::string &which, const std::string &name, const double value)
    {
        //TODO
    }

    uhd::meta_range_t get_gain_range(const std::string &which, const std::string &name)
    {
        //TODO
    }

    double set_clock_rate(const double rate)
    {
        //TODO
    }

    double tune(const std::string &which, const double value)
    {
        //TODO
    }

private:
    spi_iface::sptr _spiface;
    usb_control::sptr _usb_ctrl;
};

/***********************************************************************
 * Make an instance of the implementation
 **********************************************************************/
b200_codec_ctrl::sptr b200_codec_ctrl::make(spi_iface::sptr spiface, usb_control::sptr usb_ctrl)
{
    return sptr(new b200_codec_ctrl_impl(spiface, usb_ctrl));
}
