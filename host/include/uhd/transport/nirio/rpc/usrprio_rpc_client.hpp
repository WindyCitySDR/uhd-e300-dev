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

#ifndef INCLUDED_USRPRIO_RPC_CLIENT_HPP
#define INCLUDED_USRPRIO_RPC_CLIENT_HPP

#include <uhd/transport/nirio/rpc/rpc_common.hpp>
#include <uhd/transport/nirio/rpc/rpc_client.hpp>
#include <uhd/transport/nirio/rpc/usrprio_rpc_common.hpp>
#include <uhd/transport/nirio/status.h>

namespace usrprio_rpc {

class usrprio_rpc_client {
public:
    usrprio_rpc_client(
        std::string server,
        std::string port);
    ~usrprio_rpc_client();

    inline void set_rpc_timeout(boost::posix_time::milliseconds timeout_in_ms) {
        _timeout = timeout_in_ms;
    }

    nirio_status niusrprio_initialize(
        NIUSRPRIO_INITIALIZE_ARGS);
    nirio_status niusrprio_finalize(
        NIUSRPRIO_FINALIZE_ARGS);
    nirio_status niusrprio_enumerate(
            NIUSRPRIO_ENUMERATE_ARGS);
    nirio_status niusrprio_open_session(
        NIUSRPRIO_OPEN_SESSION_ARGS);
    nirio_status niusrprio_close_session(
        NIUSRPRIO_CLOSE_SESSION_ARGS);
    nirio_status niusrprio_reset_device(
        NIUSRPRIO_RESET_SESSION_ARGS);
    nirio_status niusrprio_download_fpga_to_flash(
            NIUSRPRIO_DOWNLOAD_FPGA_TO_FLASH_ARGS);

    static const boost::int64_t DEFAULT_TIMEOUT_IN_MS = 5000;

private:
    nirio_status static _boost_error_to_nirio_status(const boost::system::error_code& err);

    rpc_client                      _rpc_client;
    boost::posix_time::milliseconds _timeout;
};

}

#endif /* INCLUDED_USRPRIO_RPC_CLIENT_HPP */
