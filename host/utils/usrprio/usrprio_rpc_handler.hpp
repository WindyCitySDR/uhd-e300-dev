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

#ifndef INCLUDED_USRPRIO_RPC_HANDLER_HPP
#define INCLUDED_USRPRIO_RPC_HANDLER_HPP

#include <uhd/transport/nirio/rpc/rpc_common.hpp>
#include <uhd/transport/nirio/rpc/rpc_client.hpp>
#include <uhd/transport/nirio/rpc/usrprio_rpc_common.hpp>
#include <uhd/transport/nirio/status.h>
#include <boost/noncopyable.hpp>

namespace usrprio_rpc {

class usrprio_rpc_handler : private boost::noncopyable{
public:
    explicit usrprio_rpc_handler(nirio_status& status);
    ~usrprio_rpc_handler();

    void handle_function_call(
        func_id_t func_id,
        const func_args_reader_t& in_args,
        func_args_writer_t& out_args,
        client_id_t client_id);

    nirio_status niusrprio_enumerate(
        client_id_t client_id,
        NIUSRPRIO_ENUMERATE_ARGS);
    nirio_status niusrprio_open_session(
        client_id_t client_id,
        NIUSRPRIO_OPEN_SESSION_ARGS);
    nirio_status niusrprio_close_session(
        client_id_t client_id,
        NIUSRPRIO_CLOSE_SESSION_ARGS);
    nirio_status niusrprio_reset_device(
        client_id_t client_id,
        NIUSRPRIO_RESET_SESSION_ARGS);
    nirio_status niusrprio_query_session_lock(
        client_id_t client_id,
        NIUSRPRIO_QUERY_SESSION_LOCK_ARGS);
    nirio_status niusrprio_get_interface_path(
        client_id_t client_id,
        NIUSRPRIO_GET_INTERFACE_PATH_ARGS);
    nirio_status niusrprio_download_fpga_to_flash(
        client_id_t client_id,
        NIUSRPRIO_DOWNLOAD_FPGA_TO_FLASH_ARGS);

private:
    static inline uint8_t _reverse_bits(uint8_t b) {
       b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
       b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
       b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
       return b;
    }

    boost::uint32_t _read_bitstream_from_file(
        const std::string& filename,
        boost::scoped_array<uint8_t>& buffer);

    boost::uint32_t _get_interface_num(
        const std::string& resource);

    std::string _get_interface_path(
        boost::uint32_t interface_num);

    typedef std::map<boost::uint32_t, client_id_t>  session_map_t;
    typedef std::map<std::string, std::string>      fpga_sig_map_t;

    session_map_t   _session_map;
    fpga_sig_map_t  _fpga_sig_map;
    boost::mutex    _session_info_mutex;
    boost::mutex    _fpga_sig_mutex;
};

}

#endif /* INCLUDED_USRPRIO_RPC_HANDLER_HPP */
