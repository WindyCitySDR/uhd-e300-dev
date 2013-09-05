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

#include <uhd/transport/nirio/rpc/usrprio_rpc_client.hpp>
#include <uhd/utils/platform.hpp>

namespace usrprio_rpc {

usrprio_rpc_client::usrprio_rpc_client(
    std::string server,
    std::string port
) : _rpc_client(server, port, uhd::get_process_id(), uhd::get_host_id()),
    _timeout(boost::posix_time::milliseconds(DEFAULT_TIMEOUT_IN_MS))
{

}

usrprio_rpc_client::~usrprio_rpc_client() {

}

nirio_status usrprio_rpc_client::niusrprio_initialize(NIUSRPRIO_INITIALIZE_ARGS)
/*
#define NIUSRPRIO_INITIALIZE_ARGS       \
    void
*/
{
    usrprio_rpc::func_args_writer_t in_args;
    usrprio_rpc::func_args_reader_t out_args;
    nirio_status status = NiRio_Status_Success;

    status = _boost_error_to_nirio_status(
        _rpc_client.call(NIUSRPRIO_INITIALIZE, in_args, out_args, _timeout));

    if (nirio_status_not_fatal(status)) {
        out_args >> status;
    }

    return status;
}

nirio_status usrprio_rpc_client::niusrprio_finalize(NIUSRPRIO_FINALIZE_ARGS)
/*
#define NIUSRPRIO_FINALIZE_ARGS         \
    void
*/
{
    usrprio_rpc::func_args_writer_t in_args;
    usrprio_rpc::func_args_reader_t out_args;
    nirio_status status = NiRio_Status_Success;

    status = _boost_error_to_nirio_status(
        _rpc_client.call(NIUSRPRIO_FINALIZE, in_args, out_args, _timeout));

    if (nirio_status_not_fatal(status)) {
        out_args >> status;
    }

    return status;
}

nirio_status usrprio_rpc_client::niusrprio_enumerate(NIUSRPRIO_ENUMERATE_ARGS)
/*
#define NIUSRPRIO_ENUMERATE_ARGS         \
    usrprio_device_info_vtr& device_info_vtr
*/
{
    usrprio_rpc::func_args_writer_t in_args;
    usrprio_rpc::func_args_reader_t out_args;
    nirio_status status = NiRio_Status_Success;
    boost::uint32_t vtr_size = 0;

    status = _boost_error_to_nirio_status(
        _rpc_client.call(NIUSRPRIO_ENUMERATE, in_args, out_args, _timeout));

    if (nirio_status_not_fatal(status)) {
        out_args >> status;
        out_args >> vtr_size;
    }
    if (nirio_status_not_fatal(status) && vtr_size > 0) {
        device_info_vtr.resize(vtr_size);
        for (size_t i = 0; i < (size_t)vtr_size; i++) {
            usrprio_device_info info;
            out_args >> info;
            device_info_vtr[i] = info;
        }
    }
    return status;
}

nirio_status usrprio_rpc_client::niusrprio_open_session(NIUSRPRIO_OPEN_SESSION_ARGS)
/*
#define NIUSRPRIO_OPEN_SESSION_ARGS     \
    const std::string& resource,        \
    const std::string& path,            \
    const std::string& signature,       \
    const boost::uint32_t& attribute,   \
    boost::uint32_t& session
*/
{
    usrprio_rpc::func_args_writer_t in_args;
    usrprio_rpc::func_args_reader_t out_args;
    nirio_status status = NiRio_Status_Success;

    in_args << resource;
    in_args << path;
    in_args << signature;
    in_args << attribute;

    status = _boost_error_to_nirio_status(
        _rpc_client.call(NIUSRPRIO_OPEN_SESSION, in_args, out_args, _timeout));

    if (nirio_status_not_fatal(status)) {
        out_args >> status;
        out_args >> session;
    }

    return status;
}

nirio_status usrprio_rpc_client::niusrprio_close_session(NIUSRPRIO_CLOSE_SESSION_ARGS)
/*
#define NIUSRPRIO_CLOSE_SESSION_ARGS    \
    const boost::uint32_t& session,     \
    const boost::uint32_t& attribute
*/
{
    usrprio_rpc::func_args_writer_t in_args;
    usrprio_rpc::func_args_reader_t out_args;
    nirio_status status = NiRio_Status_Success;

    in_args << session;
    in_args << attribute;

    status = _boost_error_to_nirio_status(
        _rpc_client.call(NIUSRPRIO_CLOSE_SESSION, in_args, out_args, _timeout));

    if (nirio_status_not_fatal(status)) {
        out_args >> status;
    }

    return status;
}

nirio_status usrprio_rpc_client::niusrprio_reset_device(NIUSRPRIO_RESET_SESSION_ARGS)
/*
#define NIUSRPRIO_RESET_SESSION_ARGS    \
    const boost::uint32_t& session
*/
{
    usrprio_rpc::func_args_writer_t in_args;
    usrprio_rpc::func_args_reader_t out_args;
    nirio_status status = NiRio_Status_Success;

    in_args << session;

    status = _boost_error_to_nirio_status(
        _rpc_client.call(NIUSRPRIO_RESET_SESSION, in_args, out_args, _timeout));

    if (nirio_status_not_fatal(status)) {
        out_args >> status;
    }

    return status;
}

nirio_status usrprio_rpc_client::niusrprio_download_fpga_to_flash(NIUSRPRIO_DOWNLOAD_FPGA_TO_FLASH_ARGS)
/*
#define NIUSRPRIO_DOWNLOAD_FPGA_TO_FLASH_ARGS   \
    const boost::uint32_t& interface_num,       \
    const std::string& bitstream_path
*/
{
    usrprio_rpc::func_args_writer_t in_args;
    usrprio_rpc::func_args_reader_t out_args;
    nirio_status status = NiRio_Status_Success;

    in_args << interface_num;
    in_args << bitstream_path;

    status = _boost_error_to_nirio_status(
        _rpc_client.call(NIUSRPRIO_DOWNLOAD_FPGA_TO_FLASH, in_args, out_args, _timeout));

    if (nirio_status_not_fatal(status)) {
        out_args >> status;
    }

    return status;
}

nirio_status usrprio_rpc_client::_boost_error_to_nirio_status(const boost::system::error_code& err) {
    if (err) {
        return NiRio_Status_RpcConnectionError;
    } else {
        return NiRio_Status_Success;
    }
}

}
