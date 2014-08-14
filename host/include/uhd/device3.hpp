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

#ifndef INCLUDED_UHD_DEVICE3_HPP
#define INCLUDED_UHD_DEVICE3_HPP

#include <uhd/device.hpp>
#include <uhd/usrp/rfnoc/block_ctrl_base.hpp>

namespace uhd {

/*!
 * \brief Extends uhd::device for third-generation USRP devices.
 *
 * One major feature of these devices is that they support
 * RFNoC (RF Network-on-Chip).
 */
class UHD_API device3 : public uhd::device {

  public:
    typedef boost::shared_ptr<device3> sptr;

    /*! \brief Returns a block controller class for an RFNoC block.
     *
     * If the given block ID is not valid (i.e. such a block does not exist
     * on this device), it will throw a uhd::lookup_error.
     *
     * \param block_id Canonical block name (e.g. "0/FFT_1").
     */
    rfnoc::block_ctrl_base::sptr get_block_ctrl(const rfnoc::block_id_t &block_id) const;

    /*! Same as get_block_ctrl(), but with a type cast.
     *
     * If you have a block controller class that is derived from block_ctrl_base,
     * use this function to access its specific methods.
     *
     * \code{.cpp}
     * // Assume DEV is a device3::sptr
     * uhd::rfnoc::my_block_ctrl::sptr block_controller = get_block_ctrl<my_block_ctrl>("0/MyBlock_0");
     * block_controller->my_own_block_method();
     * \endcode
     */
    template <typename T>
    boost::shared_ptr<T> get_block_ctrl(const rfnoc::block_id_t &block_id) const
    {
        return boost::dynamic_pointer_cast<T>(get_block_ctrl(block_id));
    }

    /*! Like get_block_ctrl(), but uses a less strict method for finding blocks.
     *
     * Uses block_ctrl_base::match() internally. The first block that matches
     * the given string will be returned.
     *
     * If no matching block is found, it returns a NULL pointer.
     * Boolean operations can be used to check for a valid block, e.g.
     * \code{.cpp}
     * // Assume DEV is a device3::sptr
     * if (not DEV->find_block_ctrl("Radio")) {
     *     std::cout << "Device has no radios." << std::endl;
     * }
     * \endcode
     *
     * \param block_id Block name (e.g. "FFT").
     */
    rfnoc::block_ctrl_base::sptr find_block_ctrl(const std::string &block_id) const;

    /*! Type-cast version of find_block_ctrl().
     *
     * See get_block_ctrl().
     */
    template <typename T>
    boost::shared_ptr<T> find_block_ctrl(const std::string &block_id) const
    {
        return boost::dynamic_pointer_cast<T>(find_block_ctrl(block_id));
    }

    /*!
     * \param dst Who gets this command (radio0, ce1, ...)
     * \param type Type of command (setup_radio, poke)
     * \param arg1 First command arg (for poke: settings register)
     * \param arg2 Second command arg (for poke: register value)
     */
    virtual boost::uint32_t rfnoc_cmd(
            const std::string &dst,
            const std::string &type,
            boost::uint32_t arg1=0,
            boost::uint32_t arg2=0
    ) {
        throw uhd::not_implemented_error(str(boost::format("%s %s %d %d") % dst % type % arg1 % arg2));
    }

  protected:
    //! List of *all* RFNoC blocks available on this device.
    //  It is the responsibility of the deriving class to make
    //  sure this gets correctly populated.
    std::vector< rfnoc::block_ctrl_base::sptr > _rfnoc_block_ctrl;
};

} //namespace uhd

#endif /* INCLUDED_UHD_DEVICE3_HPP */
// vim: sw=4 et:
