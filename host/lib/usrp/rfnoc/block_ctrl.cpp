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

#include "block_ctrl.hpp"
#include "radio_ctrl_core_3000.hpp"

class block_ctrl_impl: public block_ctrl
{
public:

    block_ctrl_impl(const bool big_endian,
            uhd::transport::zero_copy_if::sptr ctrl_xport,
            uhd::transport::zero_copy_if::sptr resp_xport,
            const boost::uint32_t sid, const std::string &name) :
            _ctrl_core(radio_ctrl_core_3000::make(big_endian, ctrl_xport, resp_xport, sid, name))
    {
    }

    ~block_ctrl_impl(void)
    {
    }

    /*******************************************************************
     * Peek and poke 32 bit implementation
     ******************************************************************/
    void poke32(const uhd::wb_iface::wb_addr_type addr, const boost::uint32_t data)
    {
        return _ctrl_core->poke32(addr, data);
    }

    boost::uint32_t peek32(const uhd::wb_iface::wb_addr_type addr)
    {
        return _ctrl_core->peek32(addr);
    }

    radio_ctrl_core_3000::sptr _ctrl_core;

};

block_ctrl::sptr block_ctrl::make(
                const bool big_endian,
		uhd::transport::zero_copy_if::sptr ctrl_xport,
                uhd::transport::zero_copy_if::sptr resp_xport,
		const boost::uint32_t sid,
                const std::string &name
) {
    return sptr(
            new block_ctrl_impl(big_endian, ctrl_xport, resp_xport, sid, name)
    );
}
// vim: sw=4 et:
