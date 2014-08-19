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
#include <uhd/utils/log.hpp>
#include <boost/format.hpp>

#include <uhd/usrp/rfnoc/block_ctrl_base.hpp>
// TODO can I delete this line? (5th august)
//#include "radio_ctrl_core_3000.hpp"

//! Convert register to a peek/poke compatible address
inline boost::uint32_t _sr_to_addr(boost::uint32_t reg) { return reg * 4; };
inline boost::uint32_t _sr_to_addr64(boost::uint32_t reg) { return reg * 8; }; // for peek64

using namespace uhd;
using namespace uhd::rfnoc;

// One line in FPGA is 64 Bits
static const size_t BYTES_PER_LINE = 8;

block_ctrl_base::block_ctrl_base(
        wb_iface::sptr ctrl_iface,
        sid_t ctrl_sid,
        size_t device_index,
        property_tree::sptr tree,
        bool transport_is_big_endian
) : _ctrl_sid(ctrl_sid),
    _ctrl_iface(ctrl_iface),
    _tree(tree),
    _transport_is_big_endian(transport_is_big_endian)
{
    UHD_MSG(status) << "block_ctrl_base()" << std::endl;
    // Read NoC-ID
    boost::uint64_t noc_id = sr_read64(SR_READBACK_REG_ID);
    UHD_MSG(status) << "NOC ID: " << str(boost::format("0x%016x") % noc_id) << std::endl;

    // Read buffer sizes (also, identifies which ports may receive connections)
    std::vector<size_t> buf_sizes(16, 0);
    for (size_t port_offset = 0; port_offset < 16; port_offset += 8) {
        settingsbus_reg_t reg =
            (port_offset == 0) ? SR_READBACK_REG_BUFFALLOC0 : SR_READBACK_REG_BUFFALLOC1;
        boost::uint64_t value = sr_read64(reg);
        for (size_t i = 0; i < 8; i++) {
            size_t buf_size_log2 = (value >> (i * 8)) & 0xFF; // Buffer size in x = log2(lines)
            buf_sizes[i + port_offset] = BYTES_PER_LINE * (1 << buf_size_log2); // Bytes == 8 * 2^x
        }
    }

    // Figure out block ID
    // TODO replace with something that actually sets a name
    std::string blockname = "CE"; // Until we can read the actual block names
    if (noc_id == 0xAAAABBBBCCCC0000) {
        blockname = "NullSrcSink";
    }
    else if (((noc_id >> 48) & 0xFFFF) != 0xAAAA) {
        blockname = "Radio";
    }

    _block_id.set(device_index, blockname, 0);
    while (_tree->exists("xbar/" + _block_id.get_local())) {
        _block_id++;
    }
    UHD_MSG(status) << "Using block ID: " << _block_id << std::endl;

    // Populate property tree
    _root_path = "xbar/" + _block_id.get_local();
    _tree->create<boost::uint64_t>(_root_path / "noc_id").set(noc_id);
    _tree->create<std::vector<size_t> >(_root_path / "input_buffer_size").set(buf_sizes);

    _tree->create<size_t>(_root_path / "bytes_per_packet/default").set(DEFAULT_PACKET_SIZE);
    // TODO this value might be different.
    // Figure out true value, or allow setter, or register publisher
    _tree->create<double>(_root_path / "clock_rate").set(160e6);

    // TODO: Add IO signature


    // FIXME remove this from here! It will always call block_ctrl_base::reset_flow_control().
    reset_flow_control();
}

block_ctrl_base::~block_ctrl_base() {
}

void block_ctrl_base::sr_write(const boost::uint32_t reg, const boost::uint32_t data) {
    UHD_MSG(status) << str(boost::format("sr_write(%d, %08x) on %s") % reg % data % get_block_id()) << std::endl;
    _ctrl_iface->poke32(_sr_to_addr(reg), data);
}

boost::uint64_t block_ctrl_base::sr_read64(const settingsbus_reg_t reg)
{
    return _ctrl_iface->peek64(_sr_to_addr64(reg));
}

boost::uint32_t block_ctrl_base::sr_read32(const settingsbus_reg_t reg) {
    return _ctrl_iface->peek32(_sr_to_addr(reg));
}

size_t block_ctrl_base::get_fifo_size(size_t block_port) const {
    return _tree->access<std::vector<size_t> >(_root_path / "input_buffer_size").get().at(block_port);
}

boost::uint32_t block_ctrl_base::get_address(size_t block_port) { // Accept the 'unused' warning as a reminder we actually need it!
    // TODO use this line once the block port feature is implemented on the crossbar
    //return (_ctrl_sid.get_dst_address() & 0xFFF0) | (block_port & 0xF);
    return _ctrl_sid.get_dst_address();
}

double block_ctrl_base::get_clock_rate() const
{
    return _tree->access<double>(_root_path / "clock_rate").get();
}

//void block_ctrl_base::issue_stream_cmd(
        //const uhd::stream_cmd_t &stream_cmd
//) {
    //if (_upstream_blocks.empty()) {
        //UHD_MSG(warning) << "issue_stream_cmd() not implemented for " << _block_id << std::endl;
        //return;
    //}

    //BOOST_FOREACH(const boost::weak_ptr<block_ctrl_base> upstream_block_ctrl, _upstream_blocks) {
        //sptr(upstream_block_ctrl)->issue_stream_cmd(stream_cmd);
    //}
//}

void block_ctrl_base::configure_flow_control_in(
        size_t cycles,
        size_t packets,
        UHD_UNUSED(size_t block_port)
) {
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

void block_ctrl_base::configure_flow_control_out(
            size_t buf_size_pkts,
            UHD_UNUSED(size_t block_port),
            UHD_UNUSED(const uhd::sid_t &sid)
) {
    // This actually takes counts between acks. So if the buffer size is 1 packet, we set
    // set this to zero.
    sr_write(SR_FLOW_CTRL_BUF_SIZE, (buf_size_pkts == 0) ? 0 : buf_size_pkts-1);
    sr_write(SR_FLOW_CTRL_ENABLE, (buf_size_pkts != 0));
}

void block_ctrl_base::reset_flow_control()
{
    sr_write(SR_FLOW_CTRL_CLR_SEQ, 0x00C1EA12); // 'CLEAR', but we can write anything, really
    return;
}


bool block_ctrl_base::set_bytes_per_output_packet(size_t bpp, UHD_UNUSED(size_t out_block_port))
{
    if (bpp % BYTES_PER_LINE) {
        return false;
    }

    fs_path bpp_path = _root_path / str(boost::format("bytes_per_packet/%d") % out_block_port);
    if (_tree->exists(bpp_path)) {
        _tree->access<size_t>(bpp_path).set(bpp);
    } else {
        _tree->create<size_t>(bpp_path).set(bpp);
    }
    return true;
}

bool block_ctrl_base::set_bytes_per_input_packet(UHD_UNUSED(size_t bpp), UHD_UNUSED(size_t in_block_port))
{
    return true;
}

size_t block_ctrl_base::get_bytes_per_output_packet(size_t out_block_port)
{
    fs_path bpp_path = _root_path / str(boost::format("bytes_per_packet/%d") % out_block_port);
    if (_tree->exists(bpp_path)) {
        return _tree->access<size_t>(bpp_path).get();
    }
    return _tree->access<size_t>(_root_path / "bytes_per_packet/default").get();
}

void block_ctrl_base::set_destination(
        boost::uint32_t next_address,
        size_t output_block_port
) {
    sid_t new_sid(next_address);
    new_sid.set_remote_src_address(_ctrl_sid.get_remote_src_address());
    new_sid.set_local_src_address(_ctrl_sid.get_local_src_address() + output_block_port);
    UHD_LOG << "In block: " << _block_id << "Setting SID: " << new_sid << std::endl;
    sr_write(SR_NEXT_DST, (1<<16) | next_address);
}

void block_ctrl_base::register_upstream_block(block_ctrl_base::sptr upstream_block)
{
    _upstream_blocks.push_back(boost::weak_ptr<block_ctrl_base>(upstream_block));
}

//void block_ctrl_base::setup_rx_streamer(uhd::stream_args_t &, const uhd::sid_t &data_sid)
//{
    //set_destination(data_sid.get_src_address());
//}

//void block_ctrl_base::setup_tx_streamer(uhd::stream_args_t &args)
//{
    //// nop
//}

//void block_ctrl_base::handle_overrun()
//{
    //// nop
//}

//uhd::time_spec_t block_ctrl_base::get_time_now(void)
//{
    //return uhd::time_spec_t();
//}

//bool block_ctrl_base::in_continuous_streaming_mode(void)
//{
    //return false;
//}

// vim: sw=4 et:
