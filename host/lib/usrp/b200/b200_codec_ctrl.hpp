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

#ifndef INCLUDED_B200_CODEC_CTRL_HPP
#define INCLUDED_B200_CODEC_CTRL_HPP

#include "b200_iface.hpp"

#include <uhd/transport/usb_control.hpp>
#include <uhd/types/serial.hpp>
#include <uhd/types/ranges.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <vector>
#include <string>

class b200_codec_ctrl : boost::noncopyable{
public:
    typedef boost::shared_ptr<b200_codec_ctrl> sptr;

    //! make a new codec control object
    static sptr make(b200_iface::sptr);

    //! Get a list of gain names for RX or TX
    virtual std::vector<std::string> get_gain_names(const std::string &which) = 0;

    //! set the gain for a particular gain element
    virtual double set_gain(const std::string &which, const std::string &name, const double value) = 0;

    //! get the gain range for a particular gain element
    virtual uhd::meta_range_t get_gain_range(const std::string &which, const std::string &name) = 0;

    //! set a new clock rate, return the exact value
    virtual double set_clock_rate(const double rate) = 0;

    //! tune the given frontend, return the exact value
    virtual double tune(const std::string &which, const double value) = 0;

};

#endif /* INCLUDED_B200_CODEC_CTRL_HPP */
