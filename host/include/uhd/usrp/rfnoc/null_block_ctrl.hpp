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

/*! \brief Provide access basic to functionality of an RFNoC block.
 */
class null_block_ctrl : virtual public block_ctrl_base
{
public:
    typedef boost::shared_ptr<null_block_ctrl> sptr;

    // TODO
    static sptr make(TODO);

    //! Custom function to set the rate at which data is produced.
    virtual void set_line_rate(size_t cycs_per_line) = 0;

}; /* class block_ctrl*/

#endif /* INCLUDED_LIBUHD_RFNOC_NULL_BLOCK_CTRL_HPP */
// vim: sw=4 et:
