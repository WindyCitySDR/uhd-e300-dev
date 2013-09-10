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

#include <uhd/transport/nirio/rpc/rpc_common.hpp>

namespace usrprio_rpc {

//Function IDs

static const func_id_t NIUSRPRIO_FUNC_BASE              = 0x100;

static const func_id_t NIUSRPRIO_INITIALIZE             = NIUSRPRIO_FUNC_BASE + 0;
static const func_id_t NIUSRPRIO_FINALIZE               = NIUSRPRIO_FUNC_BASE + 1;
static const func_id_t NIUSRPRIO_ENUMERATE              = NIUSRPRIO_FUNC_BASE + 2;
static const func_id_t NIUSRPRIO_OPEN_SESSION           = NIUSRPRIO_FUNC_BASE + 3;
static const func_id_t NIUSRPRIO_CLOSE_SESSION          = NIUSRPRIO_FUNC_BASE + 4;
static const func_id_t NIUSRPRIO_RESET_SESSION          = NIUSRPRIO_FUNC_BASE + 5;
static const func_id_t NIUSRPRIO_DOWNLOAD_FPGA_TO_FLASH = NIUSRPRIO_FUNC_BASE + 6;

//Function Args

struct usrprio_device_info {
    boost::uint32_t interface_num;
    std::string     resource_name;
    std::string     serial_num;
    std::string     interface_path;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        if (version || !version) {  //Suppress unused warning
            ar & interface_num;
            ar & resource_name;
            ar & serial_num;
            ar & interface_path;
        }
    }
};
typedef std::vector<usrprio_device_info> usrprio_device_info_vtr;

#define NIUSRPRIO_INITIALIZE_ARGS       \
    void

#define NIUSRPRIO_FINALIZE_ARGS         \
    void

#define NIUSRPRIO_ENUMERATE_ARGS        \
    usrprio_device_info_vtr& device_info_vtr

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

#define NIUSRPRIO_DOWNLOAD_FPGA_TO_FLASH_ARGS   \
    const boost::uint32_t& interface_num,       \
    const std::string& bitstream_path

}

#endif /* INCLUDED_USRPRIO_RPC_COMMON_HPP */