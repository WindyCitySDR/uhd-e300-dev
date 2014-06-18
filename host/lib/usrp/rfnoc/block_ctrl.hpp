//
// Copyright 2014 Ettus Research LLC
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

#ifndef INCLUDED_LIBUHD_BLOCK_CTRL_HPP
#define INCLUDED_LIBUHD_BLOCK_CTRL_HPP

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <uhd/transport/zero_copy.hpp>
#include <uhd/types/wb_iface.hpp>

/*!
 * Provide access to peek, poke for noc shell ctrl modules
 */
class block_ctrl
{
public:
    typedef boost::shared_ptr<block_ctrl> sptr;

    //! Make a new control object
    static sptr make(
        const bool big_endian,
        uhd::transport::zero_copy_if::sptr ctrl_xport,
        uhd::transport::zero_copy_if::sptr resp_xport,
        const boost::uint32_t sid,
        const std::string &name = "block"
    );

    virtual void poke32(const uhd::wb_iface::wb_addr_type addr, const boost::uint32_t data) = 0;
    virtual boost::uint32_t peek32(const uhd::wb_iface::wb_addr_type addr) = 0;

};

#endif /* INCLUDED_LIBUHD_BLOCK_CTRL_HPP */
