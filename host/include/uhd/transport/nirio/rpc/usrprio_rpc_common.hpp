//
// Copyright 2013 Ettus Research LLC
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

#ifndef INCLUDED_USRPRIO_RPC_COMMON_HPP
#define INCLUDED_USRPRIO_RPC_COMMON_HPP

namespace usrprio_rpc {

//Function IDs

static const func_id_t NIUSRPRIO_FUNC_BASE      = 0x100;

static const func_id_t NIUSRPRIO_INITIALIZE     = NIUSRPRIO_FUNC_BASE + 0;
static const func_id_t NIUSRPRIO_FINALIZE       = NIUSRPRIO_FUNC_BASE + 1;
static const func_id_t NIUSRPRIO_ENUMERATE      = NIUSRPRIO_FUNC_BASE + 2;
static const func_id_t NIUSRPRIO_OPEN_SESSION   = NIUSRPRIO_FUNC_BASE + 3;
static const func_id_t NIUSRPRIO_CLOSE_SESSION  = NIUSRPRIO_FUNC_BASE + 4;
static const func_id_t NIUSRPRIO_RESET_SESSION  = NIUSRPRIO_FUNC_BASE + 5;

//Function Args

#define NIUSRPRIO_INITIALIZE_ARGS       \
    void

#define NIUSRPRIO_FINALIZE_ARGS         \
    void

#define NIUSRPRIO_OPEN_SESSION_ARGS     \
    const std::string& resource,        \
    const std::string& path,            \
    const std::string& signature,       \
    const boost::uint32_t& attribute,   \
    boost::uint32_t& session

#define NIUSRPRIO_CLOSE_SESSION_ARGS    \
    const boost::uint32_t& session,     \
    const boost::uint32_t& attribute

#define NIUSRPRIO_RESET_SESSION_ARGS    \
    const boost::uint32_t& session

}

#endif /* INCLUDED_USRPRIO_RPC_COMMON_HPP */
