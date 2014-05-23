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

#ifndef INCLUDED_E300_GLOBAL_REGS_HPP
#define INCLUDED_E300_GLOBAL_REGS_HPP

#include <uhd/types/wb_iface.hpp>
#include <uhd/transport/zero_copy.hpp>

namespace uhd { namespace usrp { namespace e300 {

struct global_regs_transaction_t {
    boost::uint32_t is_poke;
    boost::uint32_t addr;
    boost::uint32_t data;
    boost::uint32_t pad;
};

class global_regs : boost::noncopyable, public virtual uhd::wb_iface
{
public:
    typedef boost::shared_ptr<global_regs> sptr;

    static sptr make(const size_t ctrl_base);
    static sptr make(uhd::transport::zero_copy_if::sptr xport);
};

}}};

#endif /* INCLUDED_E300_GLOBAL_REGS_HPP */
