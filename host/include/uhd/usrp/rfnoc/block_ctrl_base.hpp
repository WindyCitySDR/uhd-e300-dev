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
#include <uhd/stream.hpp>
#include <uhd/types/sid.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/types/wb_iface.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/usrp/rfnoc/constants.hpp>
#include <uhd/usrp/rfnoc/block_id.hpp>


namespace uhd {
    namespace rfnoc {

struct make_args_t
{
    make_args_t(const std::string &name = "") :
        device_index(0),
        is_big_endian(true),
        block_name(name)
    {}

    uhd::wb_iface::sptr ctrl_iface;
    uhd::sid_t ctrl_sid;
    size_t device_index;
    uhd::property_tree::sptr tree;
    bool is_big_endian;
    std::string block_name;
};

//! This macro must be put in the public section of an RFNoC
// block class
#define UHD_RFNOC_BLOCK_OBJECT(class_name)  \
    typedef boost::shared_ptr< class_name > sptr;

//! Shorthand for block constructor
#define UHD_RFNOC_BLOCK_CONSTRUCTOR(CLASS_NAME) \
    CLASS_NAME##_impl( \
        const make_args_t &make_args \
    ) : block_ctrl_base(make_args)

//! This macro must be placed inside a block implementation file
// after the class definition
#define UHD_RFNOC_BLOCK_REGISTER(CLASS_NAME, BLOCK_NAME) \
    block_ctrl_base::sptr CLASS_NAME##_make( \
        const make_args_t &make_args \
    ) { \
        return block_ctrl_base::sptr(new CLASS_NAME##_impl(make_args)); \
    } \
    UHD_STATIC_BLOCK(register_rfnoc_##CLASS_NAME) \
    { \
        uhd::rfnoc::block_ctrl_base::register_block(&CLASS_NAME##_make, BLOCK_NAME); \
    }

/*! \brief Base class for all block controller objects.
 *
 * Inside UHD, block controller objects must be derived from
 * uhd::rfnoc::block_ctrl_base. This class provides all functions
 * that a block *must* provide. Typically, you would not derive
 * a block controller class directly from block_ctrl_base, but
 * from a class such as rx_block_ctrl_base or tx_block_ctrl_base
 * which extends its functionality.
 */
class UHD_API block_ctrl_base;
class block_ctrl_base : boost::noncopyable, public boost::enable_shared_from_this<block_ctrl_base>
{
private:
    //! The SID of the control transport.
    // _ctrl_sid.get_dst_address() yields this block's address.
    uhd::sid_t _ctrl_sid;

    //! The (unique) block ID.
    block_id_t _block_id;

protected:
    block_ctrl_base(void) {}; // To allow pure virtual (interface) sub-classes

    /*!
     * \param ctrl_iface A valid interface that allows us to do peeks and pokes
     * \param ctrl_sid The SID corresponding to ctrl_iface. ctrl_sid.get_dst_address() must
     *                 yield this block's address.
     * \param device_index The device index (or motherboard index).
     * \param tree A property tree for this motherboard. Example: If the root a device's
     *             property tree is /mboards/0, pass a subtree starting at /mboards/0
     *             to the constructor.
     */
    block_ctrl_base(
            const make_args_t &make_args
    );

    //! An object to actually send and receive the commands
    wb_iface::sptr _ctrl_iface;

    //! Property sub-tree
    uhd::property_tree::sptr _tree;

    //! Root node of this block's properties
    uhd::fs_path _root_path;

    //! Endianness of underlying transport (for data transport)
    bool _transport_is_big_endian;

    //! Stores default block arguments
    uhd::device_addr_t _args;

    //! List of upstream blocks
    std::vector< boost::weak_ptr<block_ctrl_base> > _upstream_blocks;

    //! List of downstream blocks
    std::vector< boost::weak_ptr<block_ctrl_base> > _downstream_blocks;

public:
    typedef boost::shared_ptr<block_ctrl_base> sptr;
    typedef boost::function<sptr(const make_args_t &)> make_t;

    //! Returns a shared_ptr of type T. Use this to access the derived block types.
    template <class T> UHD_INLINE T cast(void) const { return boost::dynamic_pointer_cast<T>(shared_from_this()); };

    /*! Register a block controller class into the discovery and factory system.
     *
     * Note: It is not recommended to call this function directly.
     * Rather, use the UHD_RFNOC_BLOCK_REGISTER() macro, which will set up
     * the discovery and factory system correctly.
     *
     * \param make A factory function that makes a block controller object
     * \param name A unique block name, e.g. 'FFT'. If a block has this block name,
     *             it will use \p make to generate the block controller class.
     */
    static void register_block(const make_t &make, const std::string &name);

    /*!
     * \brief Create a block controller class given a NoC-ID or a block name.
     *
     * If a block name is given in \p make_args, it will directly try to
     * generate a block of this type. If no block name is given, it will
     * look up a name using the NoC-ID and use that.
     * If it can't find a suitable block controller class, it will generate
     * a uhd::rfnoc::block_ctrl. However, if a block name *is* specified,
     * it will throw a uhd::runtime_error if this block type is not registered.
     *
     * \param make_args Valid make args.
     * \param noc_id The 64-Bit NoC-ID.
     * \return a shared pointer to a new device instance
     */
    static sptr make(const make_args_t &make_args, boost::uint64_t noc_id = ~0);

    /*! Initialize the block arguments.
     */
    void set_args(const uhd::device_addr_t &args);

    /*! Allows setting one register on the settings bus.
     *
     * Note: There is no address translation ("memory mapping") necessary.
     * Register 0 is 0, 1 is 1 etc.
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
    size_t get_fifo_size(size_t block_port=0) const;

    /*! Returns the 16-Bit address for this block.
     */
    boost::uint32_t get_address(size_t block_port=0);

    /*! Returns the unique block ID for this block (e.g. "0/FFT_1").
     */
    block_id_t get_block_id() const { return _block_id; };

    /*! Returns the SID for the control transport.
     */
    uhd::sid_t get_ctrl_sid() const { return _ctrl_sid; };

    /*! Return the clock rate in Hz for this block.
     */
    virtual double get_clock_rate() const;

    /*! Configure flow control for incoming streams.
     *
     * If flow control is enabled for incoming streams, this block will periodically
     * send out ACKs, telling the upstream block which packets have been consumed,
     * so the upstream block can increase his flow control credit.
     *
     * In the default implementation, this just sets registers
     * SR_FLOW_CTRL_CYCS_PER_ACK and SR_FLOW_CTRL_PKTS_PER_ACK accordingly.
     *
     * Override this function if your block has port-specific flow control settings.
     *
     * \param cycles Send an ACK after this many clock cycles.
     *               Setting this to zero disables this type of flow control acknowledgement.
     * \param packets Send an ACK after this many packets have been consumed.
     *               Setting this to zero disables this type of flow control acknowledgement.
     * \param block_port Set up flow control for a stream coming in on this particular block port.
     */
    virtual void configure_flow_control_in(
            size_t cycles,
            size_t packets,
            size_t block_port=0
     );

    /*! Configure flow control for outgoing streams.
     *
     * In the default implementation, this just sets registers SR_FLOW_CTRL_BUF_SIZE
     * and SR_FLOW_CTRL_ENABLE accordingly; \b block_port and \p sid are ignored.
     *
     * Override this function if your block has port-specific flow control settings.
     *
     * \param buf_size_pkts The size of the downstream block's input FIFO size in number of packets. Setting
     *                      this to zero disables flow control. The block will then produce data as fast as it can.
     *                     \b Warning: This can cause head-of-line blocking, and potentially lock up your device!
     * \param Specify on which outgoing port this setting is valid.
     * \param sid The SID for which this is valid. This is meant for cases where the outgoing block port is
     *            not sufficient to set the flow control, and as such is rarely used.
     */
    virtual void configure_flow_control_out(
            size_t buf_size_pkts,
            size_t block_port=0,
            const uhd::sid_t &sid=uhd::sid_t()
     );

    /*! Reset seqnum on flow control.
     *
     * This function is called in the constructor.
     *
     * TODO explain when this is necessary
     */
    virtual void reset_flow_control();

    /*! Configure the size (in bytes) of the packets this block produces.
     *
     * Note: block_ctrl_base only stores this value internally, but does not
     * set any registers. It is recommended to overload this function
     * to actually change settings.
     *
     * If this block is not capable of setting the packet size as requested,
     * this block returns false (does not throw). The calling function must
     * check this and handle accordingly.
     *
     * Setting \p bpp to 0 indicates a variable packet size.
     */
    virtual bool set_bytes_per_output_packet(size_t bpp, size_t out_block_port=0);

    /*! Inform the block of the packet size (in bytes) it is to expect on a given input port.
     *
     * This usually is called after configuring an upstream block. That block will
     * probably produce data at a certain packet size, and this function is used
     * to tell this block about the packet size.
     *
     * If this block is not capable of receiving the specified packet size,
     * this block returns false (does not throw). The caller must check this
     * and handle accordingly.
     *
     * *Important*: It may be that calling this function also changes the size
     * of outgoing packets. Call get_bytes_per_output_packet() to get the
     * definitive value.
     *
     * *Default behaviour*: Does nothing, just returns true. This function
     * is most definitely one you want to override when subclassing
     * block_ctrl_base.
     *
     * Setting \p bpp to 0 indicates a variable packet size.
     */
    virtual bool set_bytes_per_input_packet(size_t bpp, size_t in_block_port=0);

    /*! Query the size of packets (in bytes) produced on a given output port.
     *
     * A return value of 0 indicates that the packet size is not yet set on this
     * port, or is variable in size.
     */
    virtual size_t get_bytes_per_output_packet(size_t out_block_port=0);

    /*! Configures data flowing from port \p output_block_port to go to \p next_address
     *
     * In the default implementation, this will write the value in \p next_address
     * to register SR_NEXT_DST of this blocks settings bus. The value will also
     * have bit 16 set to 1, since some blocks require this to respect this value.
     */
    virtual void set_destination(boost::uint32_t next_address, size_t output_block_port = 0);

    /*! Register a block upstream of this one (i.e., a block that can send data to this block).
     *
     * Note: This does *not* affect any settings (flow control etc.). This literally only tells
     * this block about upstream blocks.
     *
     * \param upstream_block A pointer to the block instantiation
     */
    void register_upstream_block(sptr upstream_block);

    /*! Register a block downstream of this one (i.e., a block that receives data from this block).
     *
     * Note: This does *not* affect any settings (flow control etc.). This literally only tells
     * this block about downstream blocks.
     *
     * \param downstream_block A pointer to the block instantiation
     */
    void register_downstream_block(sptr downstream_block);

    /*! Clears the lists of upstream and downstream blocks, respectively.
     *
     * After calling this, and before calling register_upstream_block() again,
     * block_ctrl_base::issue_stream_cmd() will not do anything but issue a warning.
     */
    void clear_connections() { _upstream_blocks.clear(); _downstream_blocks.clear(); };

    virtual ~block_ctrl_base();

}; /* class block_ctrl_base */

}} /* namespace uhd::rfnoc */

#endif /* INCLUDED_LIBUHD_BLOCK_CTRL_BASE_HPP */
// vim: sw=4 et:
