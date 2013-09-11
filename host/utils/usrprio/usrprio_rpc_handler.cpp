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

#include "usrprio_rpc_handler.hpp"
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include "NiFpga.h"
#include "niusrprio.h"
#include <uhd/transport/nirio/status.h>
#include <uhd/transport/nirio/nirio_interface.h>
#include <iostream>
#include <fstream>
#include "rpc_logger.hpp"

#ifdef UNUSED
#elif defined(__GNUC__)
# define UNUSED(x) UNUSED_ ## x __attribute__((unused))
#elif defined(__LCLINT__)
# define UNUSED(x) /*@unused@*/ x
#else
# define UNUSED(x) x
#endif

namespace usrprio_rpc {

usrprio_rpc_handler::usrprio_rpc_handler(nirio_status& status)
{
    nirio_status_chain(NiFpga_Initialize(), status);
    nirio_status_chain(niusrprio_Initialize(), status);
}

usrprio_rpc_handler::~usrprio_rpc_handler()
{
    nirio_status status = NiRio_Status_Success;
    nirio_status_chain(NiFpga_Finalize(), status);
    nirio_status_chain(niusrprio_Finalize(), status);
}

void usrprio_rpc_handler::handle_function_call(
    func_id_t func_id,
    const func_args_reader_t& in_args,
    func_args_writer_t& out_args,
    client_id_t client_id)
{
    switch (func_id) {
        case NIUSRPRIO_ENUMERATE: {
            /*
            #define NIUSRPRIO_ENUMERATE_ARGS        \
                usrprio_device_info_vtr& device_info_vtr
            */
            usrprio_device_info_vtr device_info_vtr;
            nirio_status status = NiRio_Status_Success;

            nirio_status_chain(niusrprio_enumerate(client_id, device_info_vtr), status);

            out_args << status;
            if (nirio_status_not_fatal(status)) {
                out_args << static_cast<boost::uint32_t>(device_info_vtr.size());
                for (usrprio_device_info_vtr::const_iterator it = device_info_vtr.begin();
                     it != device_info_vtr.end(); it++) {
                    out_args << *it;
                }
            }
        } break;

        case NIUSRPRIO_OPEN_SESSION: {
            /*
            #define NIUSRPRIO_OPEN_SESSION_ARGS     \
                const std::string& resource,        \
                const std::string& path,            \
                const std::string& signature,       \
                const boost::uint32_t& attribute,   \
                boost::uint32_t& session
            */
            std::string path, signature, resource;
            boost::uint32_t attribute, session;
            in_args  >> resource;
            in_args  >> path;
            in_args  >> signature;
            in_args  >> attribute;
            out_args << niusrprio_open_session(client_id, resource, path, signature, attribute, session);
            out_args << session;
        } break;

        case NIUSRPRIO_CLOSE_SESSION: {
            /*
            #define NIUSRPRIO_CLOSE_SESSION_ARGS    \
                const boost::uint32_t& session,     \
                const boost::uint32_t& attribute
            */
            boost::uint32_t attribute, session;
            in_args  >> session;
            in_args  >> attribute;
            out_args << niusrprio_close_session(client_id, session, attribute);
        } break;

        case NIUSRPRIO_RESET_SESSION: {
            /*
            #define NIUSRPRIO_RESET_SESSION_ARGS    \
                const boost::uint32_t& session
            */
            boost::uint32_t session;
            in_args  >> session;
            out_args << niusrprio_reset_device(client_id, session);
        } break;

        case NIUSRPRIO_QUERY_SESSION_LOCK: {
            /*
            #define NIUSRPRIO_QUERY_SESSION_LOCK_ARGS   \
                const boost::uint32_t& session,         \
                boost::uint16_t& session_locked
            */
            boost::uint32_t session;
            boost::uint16_t session_locked;
            in_args  >> session;
            out_args << niusrprio_query_session_lock(client_id, session, session_locked);
            out_args << session_locked;
        } break;

        case NIUSRPRIO_DOWNLOAD_FPGA_TO_FLASH: {
            /*
            #define NIUSRPRIO_DOWNLOAD_FPGA_TO_FLASH_ARGS    \
                const boost::uint32_t& interface_num,
                const std::string& bitstream_path
            */
            std::string bitstream_path;
            boost::uint32_t interface_num;
            in_args  >> interface_num;
            in_args  >> bitstream_path;
            out_args << niusrprio_download_fpga_to_flash(client_id, interface_num, bitstream_path);
        } break;

        default: {
            out_args << NiRio_Status_FeatureNotSupported;
        }
    }
}

nirio_status usrprio_rpc_handler::niusrprio_enumerate(client_id_t UNUSED(client_id), NIUSRPRIO_ENUMERATE_ARGS)
/*
#define NIUSRPRIO_ENUMERATE_ARGS         \
    usrprio_device_info_vtr& device_info_vtr
*/
{
    nirio_status status = NiRio_Status_Success;
    device_info_vtr.clear();

    boost::uint64_t ndevs;
    nirio_status_chain(niusrprio_getNumberOfDevices(&ndevs), status);
    if (nirio_status_not_fatal(status) && ndevs > 0) {
        std::vector<uint32_t> nodes(static_cast<size_t>(ndevs));
        std::vector<uint64_t> serials(static_cast<size_t>(ndevs));

        nirio_status_chain(niusrprio_getDevicesInformation(ndevs, &nodes[0], &serials[0]), status);
        for(size_t i = 0; i < ndevs && nirio_status_not_fatal(status); i++) {
            usrprio_device_info info = usrprio_device_info();
            info.interface_num = nodes[i];
            info.resource_name = "RIO" + boost::lexical_cast<std::string>(nodes[i]);
            info.serial_num = boost::lexical_cast<std::string>(serials[i]);
            //@TODO: The interface path should come from niusrprio / helper
            info.interface_path = nirio_interface::niriok_proxy::get_interface_path(info.interface_num);

            if (info.interface_num != ((uint32_t)-1) && !info.interface_path.empty())
                device_info_vtr.push_back(info);
        }
    }
    return status;
}

nirio_status usrprio_rpc_handler::niusrprio_open_session(client_id_t client_id, NIUSRPRIO_OPEN_SESSION_ARGS)
/*
#define NIUSRPRIO_OPEN_SESSION_ARGS     \
    const std::string& resource,        \
    const std::string& path,            \
    const std::string& signature,       \
    const boost::uint32_t& attribute,   \
    boost::uint32_t& session
*/
{
    static const uint32_t MD5_HASH_STRLEN = 32;
    std::string lvbitx_signature = signature.substr(MD5_HASH_STRLEN, MD5_HASH_STRLEN);
    boost::uint32_t real_attribute = attribute;

    { //Critical section for signature map
        boost::mutex::scoped_lock lock(_fpga_sig_mutex);

        static const uint32_t ATTR_FORCE_DOWNLOAD = 1 << 29;

        fpga_sig_map_t::const_iterator it = _fpga_sig_map.find(resource);
        if (it == _fpga_sig_map.end() || ((*it).second != signature)) {
            real_attribute |= ATTR_FORCE_DOWNLOAD;
        }
        if (real_attribute & ATTR_FORCE_DOWNLOAD) {
            if (it != _fpga_sig_map.end()) _fpga_sig_map.erase(resource);
            _fpga_sig_map.insert(fpga_sig_map_t::value_type(resource, signature));
        }
    }

    nirio_status status = NiFpga_Open(path.c_str(), lvbitx_signature.c_str(), resource.c_str(), real_attribute, &session);
    if (nirio_status_not_fatal(status)) { //Critical section for session map
        boost::mutex::scoped_lock lock(_session_info_mutex);

        _session_map.insert(session_map_t::value_type(session, client_id));
    }
    return status;
}

nirio_status usrprio_rpc_handler::niusrprio_close_session(client_id_t UNUSED(client_id), NIUSRPRIO_CLOSE_SESSION_ARGS)
/*
#define NIUSRPRIO_CLOSE_SESSION_ARGS    \
    const boost::uint32_t& session,     \
    const boost::uint32_t& attribute
*/
{
    nirio_status status = NiFpga_Close(session, attribute);
    {   //Critical section for session map
        boost::mutex::scoped_lock lock(_session_info_mutex);

        _session_map.erase(session);
    }
    return status;
}

nirio_status usrprio_rpc_handler::niusrprio_reset_device(client_id_t UNUSED(client_id), NIUSRPRIO_RESET_SESSION_ARGS)
/*
#define NIUSRPRIO_RESET_SESSION_ARGS    \
    const boost::uint32_t& session
*/
{
    return NiFpga_Reset(session);
}

nirio_status usrprio_rpc_handler::niusrprio_query_session_lock(client_id_t client_id, NIUSRPRIO_QUERY_SESSION_LOCK_ARGS)
/*
#define NIUSRPRIO_QUERY_SESSION_LOCK_ARGS   \
    const boost::uint32_t& session,         \
    boost::uint16_t& session_locked
*/
{
    boost::mutex::scoped_lock lock(_session_info_mutex);
    session_map_t::const_iterator it = _session_map.find(session);
    session_locked = (it != _session_map.end() && ((*it).second != client_id)) ? 1 : 0;
    return NiRio_Status_Success;
}

nirio_status usrprio_rpc_handler::niusrprio_download_fpga_to_flash(client_id_t UNUSED(client_id), NIUSRPRIO_DOWNLOAD_FPGA_TO_FLASH_ARGS)
/*
#define NIUSRPRIO_DOWNLOAD_FPGA_TO_FLASH_ARGS   \
    const boost::uint32_t& interface_num,       \
    const std::string& bitstream_path
*/
{
    nirio_status status = NiRio_Status_Success;

    boost::scoped_array<uint8_t> buffer;
    uint32_t bytes_read = 0;
    if (!bitstream_path.empty()) {
        bytes_read = _read_bitstream_from_file(bitstream_path, buffer);
    } else {
        status = NiRio_Status_CorruptBitfile;
    }

    uint64_t usrprio_hdl;
    nirio_status_chain(niusrprio_open(interface_num, &usrprio_hdl), status);
    nirio_status_chain(niusrprio_downloadToFlash(usrprio_hdl, buffer.get(), bytes_read), status);
    nirio_status_chain(niusrprio_close(usrprio_hdl), status);
    return status;
}

uint32_t usrprio_rpc_handler::_read_bitstream_from_file(
    const std::string& filename,
    boost::scoped_array<uint8_t>& buffer)
{
    using namespace std;

    size_t file_size = 0;
    ifstream file(filename.c_str(), ios::in|ios::binary|ios::ate);
    if (file.is_open())
    {
        file_size = static_cast<size_t>(file.tellg());
        buffer.reset(new uint8_t[file_size + 1]);

        file.seekg(0, ios::beg);
        file.read((char*)buffer.get(), file_size);
        file.close();
    }

    for (size_t i = 0; i < file_size; i++)
        buffer.get()[i] = _reverse_bits(buffer.get()[i]);

    return file_size;
}

}

