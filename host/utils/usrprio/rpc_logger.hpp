///
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

#ifndef INCLUDED_RPC_LOGGER_HPP
#define INCLUDED_RPC_LOGGER_HPP

#include <iostream>

#define RPC_LOGGING_ENABLED

#ifdef RPC_LOGGING_ENABLED
    #define RPC_LOG(msg, level) \
        if (usrprio_rpc::logger::g_log_level >= usrprio_rpc::level) {\
            std::cout << "[" << usrprio_rpc::logger::g_server_id << "] " << msg << std::endl; }
    #define RPC_SET_LOG_LEVEL(lvl_str) \
        if      (lvl_str[0] == 'w' || lvl_str[0] == 'W') usrprio_rpc::logger::g_log_level = usrprio_rpc::LOG_WARNING; \
        else if (lvl_str[0] == 's' || lvl_str[0] == 'S') usrprio_rpc::logger::g_log_level = usrprio_rpc::LOG_STATUS; \
        else if (lvl_str[0] == 'v' || lvl_str[0] == 'V') usrprio_rpc::logger::g_log_level = usrprio_rpc::LOG_VERBOSE; \
        else                                             usrprio_rpc::logger::g_log_level = usrprio_rpc::LOG_NONE;
    #define RPC_SET_SVR_ID(svr_id) \
        usrprio_rpc::logger::g_server_id = svr_id;

    namespace usrprio_rpc {
    enum log_level {
        LOG_NONE    = 0,
        LOG_WARNING = 1,
        LOG_STATUS  = 2,
        LOG_VERBOSE = 3
    };

    class logger {
    public:
        static log_level g_log_level;
        static std::string g_server_id;
    };
    } // namespace usrprio_rpc
#else
    #define RPC_LOG(msg, level)
    #define RPC_SET_LOG_LEVEL(level)
    #define RPC_SET_SVR_ID(svr_id)
#endif

#endif /* INCLUDED_RPC_LOGGER_HPP */
