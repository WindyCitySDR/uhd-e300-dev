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

#ifndef INCLUDED_LIBUHD_BLOCK_CTRL_BASE_HPP
#define INCLUDED_LIBUHD_BLOCK_CTRL_BASE_HPP

#include <vector>
#include <boost/cstdint.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/types/sid.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/types/wb_iface.hpp>
#include <uhd/usrp/rfnoc/constants.hpp>
#include <uhd/usrp/rfnoc/block_id.hpp>

namespace uhd {
    namespace rfnoc {

/*! \brief Base class for all block controller objects.
 *
 * Inside UHD, block controller objects must be derived from
 * uhd::rfnoc::block_ctrl.
 */
class UHD_API block_ctrl_base;
class block_ctrl_base : boost::noncopyable, public boost::enable_shared_from_this<block_ctrl_base>
{
public:
    typedef boost::shared_ptr<block_ctrl_base> sptr;

    /*! Allows setting one register on the settings bus.
     *
     * Note: There is no address translation necessary. Register 0 is 0, 1 is 1 etc.
     *
     * \param reg The settings register to write to.
     * \param data New value of this register.
     */
    void sr_write(const boost::uint32_t reg, const boost::uint32_t data);

    /*! Allows reading one register on the settings bus (64-Bit version).
     *
     * \param reg The settings register to be read.
     *
     * Returns the readback value.
     */
    boost::uint64_t sr_read64(const settingsbus_reg_t reg);

    /*! Allows reading one register on the settings bus (32-Bit version).
     *
     * \param reg The settings register to be read.
     *
     * Returns the readback value.
     */
    boost::uint32_t sr_read32(const settingsbus_reg_t reg);

    /*! Return the size of input buffer on a given block port.
     *
     * This is necessary for setting up flow control, among other things.
     * Note: This does not query the block's settings register. This happens
     * once during construction.
     *
     * \param block_port The block port (0 through 15).
     *
     * Returns the size of the buffer in bytes.
     */
    virtual size_t get_fifo_size(size_t block_port=0) const;

    /*! Returns the 16-Bit address for this block.
     */
    virtual boost::uint32_t get_address();

    /*! Returns the unique block ID for this block (e.g. "0/FFT_1").
     */
    virtual block_id_t get_block_id() const { return _block_id; };

    /*! Returns the SID for the control transport.
     */
    virtual uhd::sid_t get_ctrl_sid() const { return _ctrl_sid; };

    /*! Issue a stream command for this block.
     *
     * There is no guaranteed action for this command. The default implementation
     * is to send this command to the next upstream block, or issue a warning if
     * there is no upstream block registered.
     *
     * However, implementations of block_ctrl_base might choose to do whatever seems
     * appropriate, including throwing exceptions. This may also be true for some
     * stream commands and not for others (i.e. STREAM_MODE_START_CONTINUOUS may be
     * implemented, and STREAM_MODE_NUM_SAMPS_AND_DONE may be not).
     *
     * This function does not check for infinite loops. Example: Say you have two blocks,
     * which are both registered as upstream from one another. If they both use
     * block_ctrl_base::issue_stream_cmd(), then the stream command will be passed from
     * one block to another indefinitely. This will not happen if one the block's
     * controller classes overrides this function and actually handles it.
     *
     * See also register_upstream_block().
     *
     * \param stream_cmd The stream command.
     */
    virtual void issue_stream_cmd(const uhd::stream_cmd_t &stream_cmd);

    /*! Configure flow control for incoming streams.
     *
     * \param cycles Send an ACK after this many clock cycles.
     *               Setting this to zero disables this type of flow control acknowledgement.
     * \param packets Send an ACK after this many packets have been consumed.
     *               Setting this to zero disables this type of flow control acknowledgement.
     * \param block_port Set up flow control for a stream coming in on this particular block port.
     *
     * Returns true on success. When false is returned, the flow control is in an undefined state.
     */
    virtual void configure_flow_control_in(boost::uint32_t cycles, boost::uint32_t packets, size_t block_port=0);

    /*! Configure flow control for outgoing streams.
     *
     * \param buf_size_pkts The size of the downstream block's input FIFO size in number of packets. Setting
     *                      this to zero disables flow control. The block will then produce data as fast as it can.
     *                     \b Warning: This can cause head-of-line blocking, and potentially lock up your device!
     * \param sid The SID for which this is valid.
     *
     * Returns true on success. When false is returned, the flow control is in an undefined state.
     */
    virtual void configure_flow_control_out(boost::uint32_t buf_size_pkts, const uhd::sid_t &sid=uhd::sid_t());

    /*! Reset seqnum on flow control.
     */
    virtual void reset_flow_control();

    /*! Set the packet size in bytes.
     *
     * Note: block_ctrl_base only stores this value internally, but does not
     * set any registers. It is recommended to overload this function
     * to actually change settings.
     */
    virtual void set_bytes_per_packet(size_t bpp);

    /*! Register a block upstream of this one (i.e., a block that can send data to this block).
     *
     * Note: This does *not* affect any settings (flow control etc.). This literally only tells
     * this block about upstream blocks.
     *
     * \param upstream_block A pointer to the block instantiation
     * \param handles_stream_cmds Whether or not \p upstream_block can receive stream commands from this block.
     */
    void register_upstream_block(sptr upstream_block, bool handles_stream_cmds=true);

    /*! Clears the list of upstream blocks.
     *
     * After calling this, and before calling register_upstream_block() again,
     * block_ctrl_base::issue_stream_cmd() will not do anything but issue a warning.
     */
    void clear_upstream_blocks() { _upstream_blocks.clear(); };

    virtual ~block_ctrl_base();

protected:
    block_ctrl_base(void) {}; // To allow pure virtual (interface) sub-classes

    /*!
     * \param ctrl_iface A valid interface that allows us to do peeks and pokes
     * \param ctrl_sid The SID corresponding to ctrl_iface
     * \param device_index The device index (or motherboard index).
     * \param tree A property tree for this motherboard. Example: If the root a device's
     *             property tree is /mboards/0, pass a subtree starting at /mboards/0
     *             to the constructor.
     */
    block_ctrl_base(
            uhd::wb_iface::sptr ctrl_iface,
            uhd::sid_t ctrl_sid,
            size_t device_index,
            uhd::property_tree::sptr tree
    );

protected:
    //! Property sub-tree
    uhd::property_tree::sptr _tree;

    //! Root node of this block's properties
    uhd::fs_path _root_path;

private:
    //! An object to actually send and receive the commands
    wb_iface::sptr _ctrl_iface;

    //! The SID of the control transport.
    //_ctrl_sid.get_dst_address() must yield this block's address.
    uhd::sid_t _ctrl_sid;

    //! The (unique) block ID.
    block_id_t _block_id;

    //! List of upstream blocks
    std::vector< boost::weak_ptr<block_ctrl_base> > _upstream_blocks;

}; /* class block_ctrl_base */

}} /* namespace uhd::rfnoc */

#endif /* INCLUDED_LIBUHD_BLOCK_CTRL_BASE_HPP */
// vim: sw=4 et:
