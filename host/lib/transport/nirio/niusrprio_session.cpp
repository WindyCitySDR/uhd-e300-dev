/*
 * nifpga_interface.cpp
 *
 *  Created on: Mar 26, 2013
 *      Author: ashish
 */

#include <uhd/transport/nirio/niusrprio_session.h>
#include <uhd/transport/nirio/nirio_fifo.h>
#include <uhd/transport/nirio/status.h>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <stdio.h>
#include <fstream>
//@TODO: Move the register defs required by the class to a common location
#include "../../usrp/x300/x300_regs.hpp"

#define RPC_CLIENT_ARGS "localhost", "50000"

namespace nifpga_interface
{

niusrprio_session::niusrprio_session(const std::string& resource_name) :
    _resource_name(resource_name),
    _session(0),
    _resource_manager(_riok_proxy),
    _rpc_client(RPC_CLIENT_ARGS)
{
}

niusrprio_session::~niusrprio_session()
{
    close();
}

nirio_status niusrprio_session::enumerate(device_info_vtr& device_info_vtr)
{
    usrprio_rpc::usrprio_rpc_client temp_rpc_client(RPC_CLIENT_ARGS);
    nirio_status status = temp_rpc_client.get_ctor_status();
    nirio_status_chain(temp_rpc_client.niusrprio_enumerate(device_info_vtr), status);
    return status;
}

nirio_status niusrprio_session::open(
    nifpga_lvbitx::sptr lvbitx,
    bool force_download)
{
    boost::unique_lock<boost::recursive_mutex> lock(_session_mutex);

    using namespace nirio_interface;

    _lvbitx = lvbitx;

    nirio_status status = NiRio_Status_Success;
    std::string bitfile_path(_lvbitx->get_bitfile_path());
    std::string signature(_lvbitx->get_signature());

    //Make sure that the RPC client connected to the server properly
    nirio_status_chain(_rpc_client.get_ctor_status(), status);
    //Check if another process is using this device
    nirio_status_chain(_wait_for_device_available(), status);
    //Get a handle to the kernel driver
    nirio_status_chain(_rpc_client.niusrprio_get_interface_path(_resource_name, _interface_path), status);
    nirio_status_chain(_riok_proxy.open(_interface_path), status);

    if (nirio_status_not_fatal(status)) {
        //Bitfile build for a particular LVFPGA interface will have the same signature
        //because the API of the bitfile does not change. Two files with the same signature
        //can however have different bitstreams because of non-LVFPGA code differences.
        //That is why we need another identifier to qualify the signature. The BIN
        //checksum is a good candidate.
        std::string lvbitx_checksum(_lvbitx->get_bitstream_checksum());
        boost::uint16_t download_fpga = (force_download || (_read_bitstream_checksum() != lvbitx_checksum)) ? 1 : 0;

        nirio_status_chain(_rpc_client.niusrprio_open_session(
            _resource_name, bitfile_path, signature, download_fpga, _session), status);

        if (nirio_status_not_fatal(status)) {
            nirio_register_info_vtr reg_vtr;
            nirio_fifo_info_vtr fifo_vtr;
            _lvbitx->init_register_info(reg_vtr);
            _lvbitx->init_fifo_info(fifo_vtr);
            _resource_manager.initialize(reg_vtr, fifo_vtr);

            nirio_status_chain(_verify_signature(), status);
            nirio_status_chain(_write_bitstream_checksum(lvbitx_checksum), status);
        }
    }

    return status;
}

void niusrprio_session::close(bool reset_fpga)
{
    boost::unique_lock<boost::recursive_mutex> lock(_session_mutex);

    if (_session) {
        nirio_status status = NiRio_Status_Success;
        if (reset_fpga) reset();
        nirio_status_chain(_rpc_client.niusrprio_close_session(_session, 0), status);
        _session = 0;
    }
}

nirio_status niusrprio_session::reset()
{
    boost::unique_lock<boost::recursive_mutex> lock(_session_mutex);
    return _rpc_client.niusrprio_reset_device(_session);
}

nirio_status niusrprio_session::download_bitstream_to_flash(const std::string& bitstream_path)
{
    boost::unique_lock<boost::recursive_mutex> lock(_session_mutex);
    return _rpc_client.niusrprio_download_fpga_to_flash(_resource_name, bitstream_path);
}

nirio_interface::niriok_proxy::sptr niusrprio_session::create_kernel_proxy(const std::string& resource_name)
{
    usrprio_rpc::usrprio_rpc_client temp_rpc_client(RPC_CLIENT_ARGS);
    nirio_status status = temp_rpc_client.get_ctor_status();

    std::string interface_path;
    nirio_status_chain(temp_rpc_client.niusrprio_get_interface_path(resource_name, interface_path), status);

    nirio_interface::niriok_proxy::sptr proxy;
    if (nirio_status_not_fatal(status)) {
        proxy.reset(new nirio_interface::niriok_proxy());
        if (proxy) nirio_status_chain(proxy->open(interface_path), status);
    }

    return proxy;
}

nirio_status niusrprio_session::_verify_signature()
{
    //Validate the signature using the kernel proxy
    nirio_status status = NiRio_Status_Success;
    boost::uint32_t sig_offset = 0;
    nirio_status_chain(_riok_proxy.get_attribute(kRioFpgaDefaultSignatureOffset, sig_offset), status);
    nirio_status_chain(_riok_proxy.set_attribute(kRioAddressSpace, kRioAddressSpaceFpga), status);
    std::string signature;
    for (boost::uint32_t i = 0; i < 8; i++) {
        boost::uint32_t quarter_sig;
        nirio_status_chain(_riok_proxy.peek(sig_offset, quarter_sig), status);
        signature += boost::str(boost::format("%08x") % quarter_sig);
    }

    std::string expected_signature(_lvbitx->get_signature());
    boost::to_upper(signature);
    boost::to_upper(expected_signature);
    if (signature.find(expected_signature) == std::string::npos) {
        nirio_status_chain(NiRio_Status_SignatureMismatch, status);
    }

    return status;
}

std::string niusrprio_session::_read_bitstream_checksum()
{
    nirio_status status = NiRio_Status_Success;
    nirio_status_chain(_riok_proxy.set_attribute(kRioAddressSpace, kRioAddressSpaceBusInterface), status);
    std::string usr_signature;
    for (boost::uint32_t i = 0; i < FPGA_USR_SIG_REG_SIZE; i+=4) {
        boost::uint32_t quarter_sig;
        nirio_status_chain(_riok_proxy.peek(FPGA_USR_SIG_REG_BASE + i, quarter_sig), status);
        usr_signature += boost::str(boost::format("%08x") % quarter_sig);
    }
    boost::to_upper(usr_signature);

    return usr_signature;
}

nirio_status niusrprio_session::_write_bitstream_checksum(const std::string& checksum)
{
    nirio_status status = NiRio_Status_Success;
    nirio_status_chain(_riok_proxy.set_attribute(kRioAddressSpace, kRioAddressSpaceBusInterface), status);
    for (boost::uint32_t i = 0; i < FPGA_USR_SIG_REG_SIZE; i+=4) {
        boost::uint32_t quarter_sig;
        try {
            std::stringstream ss;
            ss << std::hex << checksum.substr(i*2,8);
            ss >> quarter_sig;
        } catch (std::exception&) {
            quarter_sig = 0;
        }
        nirio_status_chain(_riok_proxy.poke(FPGA_USR_SIG_REG_BASE + i, quarter_sig), status);
    }
    return status;
}

nirio_status niusrprio_session::_wait_for_device_available()
{
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration elapsed;

    nirio_status status = NiRio_Status_Success;
    do {
        boost::uint16_t locked;
        _rpc_client.niusrprio_query_device_lock(_resource_name, locked);
        if (locked == 0) {
            status = NiRio_Status_Success;
        } else {
            nirio_status_chain(NiRio_Status_DeviceLocked, status);
            boost::this_thread::sleep(boost::posix_time::milliseconds(SESSION_LOCK_RETRY_INT_IN_MS));
        }
        elapsed = boost::posix_time::microsec_clock::local_time() - start_time;
    } while (
        nirio_status_fatal(status) &&
        elapsed.total_milliseconds() < SESSION_LOCK_TIMEOUT_IN_MS);

    return status;
}

}
