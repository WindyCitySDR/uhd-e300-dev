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

#include <uhd/usrp/rfnoc/rx_block_ctrl_base.hpp>
#include <uhd/utils/msg.hpp>

using namespace uhd;
using namespace uhd::rfnoc;

void rx_block_ctrl_base::issue_stream_cmd(
        const uhd::stream_cmd_t &stream_cmd
) {
    if (_upstream_blocks.empty()) {
        UHD_MSG(warning) << "issue_stream_cmd() not implemented for " << get_block_id() << std::endl;
        return;
    }

    BOOST_FOREACH(const boost::weak_ptr<block_ctrl_base> upstream_block_ctrl, _upstream_blocks) {
        sptr this_upstream_block_ctrl =
            boost::dynamic_pointer_cast<rx_block_ctrl_base>(upstream_block_ctrl.lock());
        this_upstream_block_ctrl->issue_stream_cmd(stream_cmd);
    }
}

void rx_block_ctrl_base::setup_rx_streamer(uhd::stream_args_t &, const uhd::sid_t &data_sid)
{
    set_destination(data_sid.get_src_address());
}

void rx_block_ctrl_base::handle_overrun()
{
    // nop
}

uhd::time_spec_t rx_block_ctrl_base::get_time_now(void)
{
    return uhd::time_spec_t();
}

bool rx_block_ctrl_base::in_continuous_streaming_mode(void)
{
    return false;
}

// vim: sw=4 et:
