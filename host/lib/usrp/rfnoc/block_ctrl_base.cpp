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

#include <uhd/utils/msg.hpp>
#include <boost/format.hpp>

#include <uhd/usrp/rfnoc/block_ctrl_base.hpp>
#include "radio_ctrl_core_3000.hpp"

//! Convert register to a peek/poke compatible address
inline boost::uint32_t _sr_to_addr(boost::uint32_t reg) { return reg * 4; };
inline boost::uint32_t _sr_to_addr64(boost::uint32_t reg) { return reg * 8; }; // for peek64

using namespace uhd;
using namespace uhd::rfnoc;

static const size_t BYTES_PER_LINE = 8;

    struct block_connection_t {
        //! True for big endian transport
        const bool big_endian;
        //! Transport to send commands
        uhd::transport::zero_copy_if::sptr ctrl_xport;
        //! Transport for command responses (may be the same as ctrl_xport)
        uhd::transport::zero_copy_if::sptr resp_xport;
    };

block_ctrl_base::block_ctrl_base(
        wb_iface::sptr ctrl_iface,
        sid_t ctrl_sid,
        property_tree::sptr tree
) : _ctrl_iface(ctrl_iface),
    _ctrl_sid(ctrl_sid),
    _tree(tree),
    _buf_sizes(16, 0),
    _noc_id(0),
    _block_id()
{
    UHD_MSG(status) << "block_ctrl_base()" << std::endl;
    // Read NoC-ID
    _noc_id = sr_read64(SR_READBACK_REG_ID);
    UHD_MSG(status) << "NOC ID: " << str(boost::format("0x%016x") % _noc_id) << std::endl;

    // Read buffer sizes (also, identifies which ports may receive connections)
    for (size_t port_offset = 0; port_offset < 16; port_offset += 8) {
        settingsbus_reg_t reg =
            (port_offset == 0) ? SR_READBACK_REG_BUFFALLOC0 : SR_READBACK_REG_BUFFALLOC1;
        boost::uint64_t value = sr_read64(reg);
        UHD_MSG(status) << "On port offset " << port_offset << ", read from reg " << reg << ", got size value " << value << std::endl;
        for (size_t i = 0; i < 8; i++) {
            size_t buf_size_log2 = (value >> (i * 8)) & 0xFF; // Buffer size in x = log2(lines)
            _buf_sizes[i + port_offset] = BYTES_PER_LINE * (1 << buf_size_log2); // Bytes == 8 * 2^x
        }
    }

    UHD_MSG(status) << "Buffer size 0: " << _buf_sizes[0] << std::endl;

    // Figure out block ID

    // Populate property tree
}

block_ctrl_base::~block_ctrl_base() {
}

void block_ctrl_base::sr_write(const boost::uint32_t reg, const boost::uint32_t data) {
    UHD_MSG(status) << str(boost::format("sr_write(%d, %08x)") % reg % data) << std::endl;
    UHD_MSG(status) << str(boost::format("_sr_to_addr() == %d") % _sr_to_addr(reg)) << std::endl;
    _ctrl_iface->poke32(_sr_to_addr(reg), data);
}

boost::uint64_t block_ctrl_base::sr_read64(const settingsbus_reg_t reg)
{
    UHD_MSG(status) << str(boost::format("sr_read64(%d)") % reg) << std::endl;
    return _ctrl_iface->peek64(_sr_to_addr64(reg));
}

boost::uint32_t block_ctrl_base::sr_read32(const settingsbus_reg_t reg) {
    return _ctrl_iface->peek32(_sr_to_addr(reg));
}

size_t block_ctrl_base::get_fifo_size(size_t block_port) const {
    return _buf_sizes.at(block_port);
}

boost::uint32_t block_ctrl_base::get_address() {
    return _ctrl_sid.get_dst_address();
}

void block_ctrl_base::issue_stream_command(
        const uhd::stream_cmd_t &stream_cmd
) {
    // TODO: Avoid infinite loops by remembering who issued this stream cmd
    // TODO: Pass this command to all upstream blocks
    // If nothing is connected, throw an error
}

void block_ctrl_base::configure_flow_control_in(boost::uint32_t cycles, boost::uint32_t packets, size_t block_port) {
    boost::uint32_t cycles_word = 0;
    if (cycles) {
        cycles_word = (1<<31) | cycles;
    }
    sr_write(SR_FLOW_CTRL_CYCS_PER_ACK, cycles_word);

    boost::uint32_t packets_word = 0;
    if (packets) {
        packets_word = (1<<31) | packets;
    }
    sr_write(SR_FLOW_CTRL_PKTS_PER_ACK, packets_word);
}

void block_ctrl_base::configure_flow_control_out(boost::uint32_t buf_size_pkts, const uhd::sid_t &sid) {
    // This actually takes counts between acks. So if the buffer size is 1 packet, we set
    // set this to zero.
    sr_write(SR_FLOW_CTRL_BUF_SIZE, (buf_size_pkts == 0) ? 0 : buf_size_pkts-1);
    sr_write(SR_FLOW_CTRL_ENABLE, (buf_size_pkts != 0));
}

void block_ctrl_base::reset_flow_control() {
    sr_write(SR_FLOW_CTRL_CLR_SEQ, 0x00C1EA12); // 'CLEAR', but we can write anything, really
}

void block_ctrl_base::set_bytes_per_packet(size_t bpp) {
    // TODO tbw
}

// vim: sw=4 et:
