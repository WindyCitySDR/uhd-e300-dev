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

#ifndef INCLUDED_B200_CTRL_HPP
#define INCLUDED_B200_CTRL_HPP

#include <uhd/types/time_spec.hpp>
#include <uhd/transport/zero_copy.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/function.hpp>
#include "wb_iface.hpp"
#include <string>

/*!
 * Provide access to peek, poke
 */
class b200_ctrl : public wb_iface
{
public:
    typedef boost::shared_ptr<b200_ctrl> sptr;

    //! Make a new control object
    static sptr make(
        uhd::transport::zero_copy_if::sptr xport,
        boost::function<uhd::transport::managed_recv_buffer::sptr(const double)>
    );

    //! Set the command time that will activate
    virtual void set_time(const uhd::time_spec_t &time) = 0;

    //! Set the tick rate (converting time into ticks)
    virtual void set_tick_rate(const double rate) = 0;
};

#endif /* INCLUDED_B200_CTRL_HPP */
