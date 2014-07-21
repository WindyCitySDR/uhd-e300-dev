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

#include <uhd/usrp/rfnoc/block_ctrl.hpp>

using namespace uhd::rfnoc;

class block_ctrl_impl : public block_ctrl
{
public:
    block_ctrl_impl(
            uhd::wb_iface::sptr ctrl_iface,
            uhd::sid_t ctrl_sid,
            size_t device_index,
            uhd::property_tree::sptr tree
    ) : block_ctrl_base(ctrl_iface, ctrl_sid, device_index, tree)
    {
        // nop
    }

    // Very empty class, this one
};

block_ctrl::sptr block_ctrl::make(
    uhd::wb_iface::sptr ctrl_iface,
    uhd::sid_t ctrl_sid,
    size_t device_index,
    uhd::property_tree::sptr tree
) {
    return sptr(
        new block_ctrl_impl(
            ctrl_iface, ctrl_sid, device_index, tree
        )
    );
}

