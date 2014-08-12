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

#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/types/ranges.hpp>
#include "radio_ctrl.hpp"

using namespace uhd::rfnoc;

class radio_ctrl_impl : public radio_ctrl
{
public:
    UHD_RFNOC_BLOCK_CONSTRUCTOR(radio_ctrl)
    {

    }


    void issue_stream_cmd(const uhd::stream_cmd_t &stream_cmd)
    {
    }

    bool set_bytes_per_output_packet(
            size_t bpp,
            UHD_UNUSED(size_t out_block_port) // Is not relevant for this block
    ) {
	    //tbi

        return true;
    }

    size_t get_bytes_per_output_packet(UHD_UNUSED(size_t out_block_port))
    {
        return _tree->access<size_t>(_root_path / "bytes_per_packet/default").get();
    }

    void set_destination(
            boost::uint32_t next_address,
            UHD_UNUSED(size_t output_block_port)
    ) {
        uhd::sid_t sid(next_address);
        if (sid.get_src_address() == 0) {
            sid.set_src_address(get_address());
        }
        sr_write(SR_NEXT_DST, sid.get());
    }

private:

    //! Stores pointers to all radio cores
    struct radio_v_perifs_t
    {
        time_core_3000::sptr    time64;
        rx_vita_core_3000::sptr framer;
        rx_dsp_core_3000::sptr  ddc;
        tx_vita_core_3000::sptr deframer;
        tx_dsp_core_3000::sptr  duc;
    } _perifs;

};

UHD_RFNOC_BLOCK_MAKE_CALL(radio_ctrl);

