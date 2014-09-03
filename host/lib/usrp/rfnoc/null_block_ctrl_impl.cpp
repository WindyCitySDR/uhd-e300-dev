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
#include <uhd/usrp/rfnoc/null_block_ctrl.hpp>

using namespace uhd::rfnoc;

class null_block_ctrl_impl : public null_block_ctrl
{
public:
    UHD_RFNOC_BLOCK_CONSTRUCTOR(null_block_ctrl),
        _line_rate(0.0) // This is set implicitly by the subscriber in the prop tree
    {

        // Add prop tree entry for line rate
        // We actually use _line_rate to store the rate, because we can't
        // read back the actual register (yet)
        UHD_MSG(status) << "populating " << _root_path / "line_rate/value" << std::endl;
        _tree->create<double>(_root_path / "line_rate/value")
            .subscribe(boost::bind(&null_block_ctrl_impl::set_line_rate, this, _1))
            .set(_line_rate_from_reg_val(0xFFFF)); // Default: slowest rate possible
        _tree->create<uhd::meta_range_t>(_root_path / "line_rate/range")
            .set(uhd::meta_range_t(_line_rate_from_reg_val(0xFFFF), _line_rate_from_reg_val(0)));
    }

    double set_line_rate(double rate)
    {
        int cycs_between_lines = get_clock_rate() / rate - 1;
        if (cycs_between_lines > 0xFFFF) {
            cycs_between_lines = 0xFFFF;
            UHD_MSG(warning)
                << str(boost::format("null_block_ctrl: Requested rate %f is larger than possible with the current clock rate (%.2f MHz).") % rate % (get_clock_rate() / 1e6))
                << std::endl;
        }
        boost::uint32_t register_value = std::max(0, cycs_between_lines);
        sr_write(SR_LINE_RATE, register_value);
        _line_rate = _line_rate_from_reg_val(register_value);
        return _line_rate;
    }

    double get_line_rate(void) const
    {
        return _line_rate;
    }

    double _line_rate_from_reg_val(boost::uint32_t reg_val) const
    {
        return get_clock_rate() / (reg_val + 1);
    }

    void issue_stream_cmd(const uhd::stream_cmd_t &stream_cmd)
    {
        if (not stream_cmd.stream_now) {
            throw uhd::not_implemented_error("null_block does not support timed commands.");
        }
        switch (stream_cmd.stream_mode) {
            case uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS:
                sr_write(SR_ENABLE_STREAM, true);
                break;

            case uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS:
                sr_write(SR_ENABLE_STREAM, false);
                break;

            case uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE:
            case uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE:
                throw uhd::not_implemented_error("null_block does not support streaming modes other than CONTINUOUS");

            default:
                UHD_THROW_INVALID_CODE_PATH();
        }
    }

    bool set_bytes_per_output_packet(
            size_t bpp,
            UHD_UNUSED(size_t out_block_port) // Is not relevant for this block
    ) {
        if (bpp % BYTES_PER_LINE) {
            return false;
        }
        if (bpp == 0) {
            bpp = DEFAULT_LINES_PER_PACKET;
        }
        boost::uint32_t lines_per_packet = std::max(bpp / BYTES_PER_LINE, size_t(1));
        sr_write(SR_LINES_PER_PACKET, lines_per_packet);
        _tree->access<size_t>(_root_path / "bytes_per_packet/default").set(bpp);
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

    //! Store the line rate. TODO remove once we have readback from the block to query this.
    double _line_rate;

};

UHD_RFNOC_BLOCK_REGISTER(null_block_ctrl, "NullSrcSink");

