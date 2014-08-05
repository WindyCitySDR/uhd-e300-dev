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

#ifndef INCLUDED_LIBUHD_RFNOC_NULL_BLOCK_CTRL_HPP
#define INCLUDED_LIBUHD_RFNOC_NULL_BLOCK_CTRL_HPP

#include <uhd/usrp/rfnoc/block_ctrl_base.hpp>

namespace uhd {
    namespace rfnoc {

/*! \brief Provide access to a 'null block'.
 *
 * A 'null block' is a specific block, which comes with a couple
 * of features useful for testing:
 * - It can produce data at a given line rate, with a configurable
 *   packet size.
 * - It can be used to dump packets ("null sink", "bit bucket")
 *
 * This block also serves as an example of how to create your own
 * C++ classes to control your block. This class both has functions
 * that inherit from uhd::rfnoc::block_ctrl_base, as well as
 * functions that are specific to this block.
 */
class null_block_ctrl : virtual public block_ctrl_base
{
public:
    // TODO replace with macro
    typedef boost::shared_ptr<null_block_ctrl> sptr;
    static sptr make(
            uhd::wb_iface::sptr ctrl_iface,
            uhd::sid_t ctrl_sid,
            size_t device_index,
            uhd::property_tree::sptr tree
    );
    static sptr cast(block_ctrl_base::sptr P) { return boost::dynamic_pointer_cast<null_block_ctrl>(P); };

    //! Set this register to number of lines per packet
    static const boost::uint32_t SR_LINES_PER_PACKET = 9;
    //! Set this register to number of cycles between producing a line
    static const boost::uint32_t SR_LINE_RATE = 10;
    //! Set this register to non-zero to start producing data
    static const boost::uint32_t SR_ENABLE_STREAM = 11;

    static const size_t DEFAULT_LINES_PER_PACKET = 32;
    static const size_t BYTES_PER_LINE = 8;

    //! Custom function to set the rate at which data is produced.
    // Note: This is 'cycles per line', so the bit rate is actually
    // 64 times this value (byte/s is 8*rate etc.)
    //
    // \returns the actual line rate (will find closest possible).
    virtual double set_line_rate(double rate) = 0;

    //! This block can actually initiate streaming, so we need
    // override this.
    virtual void issue_stream_cmd(const uhd::stream_cmd_t &stream_cmd) = 0;

    //! This is important for the 'source' component.
    virtual bool set_bytes_per_output_packet(
            size_t bpp,
            size_t out_block_port=0
    ) = 0;

    virtual size_t get_bytes_per_output_packet(size_t out_block_port=0) = 0;

    //! This must be overridden because as a true source, we must also
    // set the source address.
    virtual void set_destination(boost::uint32_t next_address, size_t output_block_port = 0) = 0;

}; /* class block_ctrl*/

}} /* namespace uhd::rfnoc */

#endif /* INCLUDED_LIBUHD_RFNOC_NULL_BLOCK_CTRL_HPP */
// vim: sw=4 et:
