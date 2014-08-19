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

#ifndef INCLUDED_LIBUHD_TX_BLOCK_CTRL_BASE_HPP
#define INCLUDED_LIBUHD_TX_BLOCK_CTRL_BASE_HPP

#include <uhd/usrp/rfnoc/block_ctrl_base.hpp>

namespace uhd {
    namespace rfnoc {

/*! \brief Extends block_ctrl_base with transmit capabilities.
 *
 * In RFNoC nomenclature, a transmit operation means streaming
 * data to the device (the crossbar) from the host.
 * If a block has transmit capabilities, this means we can transmit
 * data *to* this block.
 */
class UHD_API tx_block_ctrl_base;
class tx_block_ctrl_base : virtual public block_ctrl_base
{
public:
    typedef boost::shared_ptr<tx_block_ctrl_base> sptr;

    /*! Set stream args and SID before opening a TX streamer to this block.
     */
    virtual void setup_tx_streamer(uhd::stream_args_t &args);

}; /* class tx_block_ctrl_base */

}} /* namespace uhd::rfnoc */

#endif /* INCLUDED_LIBUHD_TX_BLOCK_CTRL_BASE_HPP */
// vim: sw=4 et:
