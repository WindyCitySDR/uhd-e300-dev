//
// Copyright 2012-2013 Ettus Research LLC
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

#include "ad9361_ctrl.hpp"
#include "ad9361_transaction.h"

using namespace uhd;

struct ad9361_ctrl_impl : public ad9361_ctrl
{
    ad9361_ctrl_iface_sptr _iface;
    ad9361_ctrl_impl(ad9361_ctrl_iface_sptr iface):
        _iface(iface)
    {
        
    }

    double set_gain(const std::string &which, const double value)
    {
        
    }

    //! set a new clock rate, return the exact value
    double set_clock_rate(const double rate)
    {
        
    }

    //! set which RX and TX chains/antennas are active
    void set_active_chains(bool tx1, bool tx2, bool rx1, bool rx2)
    {
        
    }

    //! tune the given frontend, return the exact value
    double tune(const std::string &which, const double value)
    {
        
    }

    //! turn on/off Catalina's data port loopback
    void data_port_loopback(const bool on)
    {
        
    }
};


/***********************************************************************
 * Make an instance of the implementation
 **********************************************************************/
ad9361_ctrl::sptr ad9361_ctrl::make(ad9361_ctrl_iface_sptr iface)
{
    return sptr(new ad9361_ctrl_impl(iface));
}
