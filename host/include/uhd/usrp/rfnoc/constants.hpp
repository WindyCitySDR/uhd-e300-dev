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

#ifndef INCLUDED_LIBUHD_RFNOC_CONSTANTS_HPP
#define INCLUDED_LIBUHD_RFNOC_CONSTANTS_HPP

namespace uhd {
    namespace rfnoc {

// Common settings registers.
static const boost::uint32_t SR_FLOW_CTRL_BUF_SIZE     = 0;
static const boost::uint32_t SR_FLOW_CTRL_ENABLE       = 1;
static const boost::uint32_t SR_FLOW_CTRL_CYCS_PER_ACK = 2;
static const boost::uint32_t SR_FLOW_CTRL_PKTS_PER_ACK = 3;
static const boost::uint32_t SR_FLOW_CTRL_CLR_SEQ      = 4;
static const boost::uint32_t SR_NEXT_DST               = 8;
static const boost::uint32_t SR_READBACK               = 32;

//! Settings register readback
enum settingsbus_reg_t {
    SR_READBACK_REG_ID         = 0,
    SR_READBACK_REG_BUFFALLOC0 = 1,
    SR_READBACK_REG_BUFFALLOC1 = 2,
    SR_READBACK_REG_USER       = 3,
};

}} /* namespace uhd::rfnoc */

#endif /* INCLUDED_LIBUHD_RFNOC_CONSTANTS_HPP */
// vim: sw=4 et:
