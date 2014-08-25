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

#include <uhd/usrp/rfnoc/tx_block_ctrl_base.hpp>
#include <uhd/utils/msg.hpp>

using namespace uhd;
using namespace uhd::rfnoc;

void tx_block_ctrl_base::setup_tx_streamer(uhd::stream_args_t &args)
{
    UHD_MSG(status) << "tx_block_ctrl_base::setup_tx_streamer() on " << get_block_id() << std::endl;

    // 1. Call our own init_tx() function
    // This may modify "args".
    _init_tx(args);
    // TODO: Decide if this is a good place to keep this
    reset_flow_control();

    // 2. Check if we're the last block
    if (_is_final_tx_block()) {
        UHD_MSG(status) << "tx_block_ctrl_base::setup_tx_streamer(): Final block, returning. " << std::endl;
        return;
    }

    // 3. Call all upstream blocks
    BOOST_FOREACH(const boost::weak_ptr<block_ctrl_base> upstream_block_ctrl, _upstream_blocks) {
        // Make a copy so that modifications downstream aren't propagated upstream
        uhd::stream_args_t new_args = args;
        sptr this_upstream_block_ctrl =
            boost::dynamic_pointer_cast<tx_block_ctrl_base>(upstream_block_ctrl.lock());
        if (this_upstream_block_ctrl) {
            this_upstream_block_ctrl->setup_tx_streamer(new_args);
        }
    }
}
// vim: sw=4 et:
