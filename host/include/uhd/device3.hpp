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
 */
class UHD_API device3 : public uhd::device {

  public:
    typedef boost::shared_ptr<device3> sptr;

    /* TODO Add ! when this is func is uncommented
     *
     * \brief Returns a block controller class for an RFNoC block.
     *
     * \param unique_block_name Canonical block name (e.g. "0/FFT_1").
     */
    //virtual rfnoc::block_ctrl_base::sptr get_block_ctrl(const std::string &unique_block_name) = 0;

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
    ) = 0;

};

} //namespace uhd

#endif /* INCLUDED_UHD_DEVICE3_HPP */
// vim: sw=4 et:
