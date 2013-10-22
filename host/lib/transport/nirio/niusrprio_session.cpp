/*
 * nifpga_interface.cpp
 *
 *  Created on: Mar 26, 2013
 *      Author: ashish
 */

#include <uhd/transport/nirio/niusrprio_session.h>
#include <uhd/transport/nirio/nirio_fifo.h>
#include <uhd/transport/nirio/status.h>
#include <boost/thread/locks.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <stdio.h>
#include <fstream>

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
	uint32_t attribute)
{
    boost::unique_lock<boost::recursive_mutex> lock(_session_mutex);

    using namespace nirio_interface;

	_lvbitx = lvbitx;

	nirio_status status = NiRio_Status_Success;
    std::string bitfile_path(_lvbitx->get_bitfile_path());
	std::string signature(_lvbitx->get_signature());

    nirio_status_chain(_rpc_client.get_ctor_status(), status);
	nirio_status_chain(_rpc_client.niusrprio_open_session(
        _resource_name, bitfile_path, signature, attribute, _session), status);

	_process_lock.initialize(_rpc_client, _session);
	nirio_status_chain(_process_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS, SESSION_LOCK_RETRY_INT_IN_MS), status);

	std::string interface_path;
    nirio_status_chain(_rpc_client.niusrprio_get_interface_path(_resource_name, interface_path), status);

	if (nirio_status_not_fatal(status)) {
	    _riok_proxy.open(interface_path);

		nirio_register_info_vtr reg_vtr;
		nirio_fifo_info_vtr fifo_vtr;
		_lvbitx->init_register_info(reg_vtr);
		_lvbitx->init_fifo_info(fifo_vtr);
		_resource_manager.initialize(reg_vtr, fifo_vtr);

		nirio_status_chain(_verify_posc_and_signature(), status);
	}

	_process_lock.release();
	return status;
}

void niusrprio_session::close(bool reset_fpga)
{
    boost::unique_lock<boost::recursive_mutex> lock(_session_mutex);

    if (_session) {
        nirio_status status = NiRio_Status_Success;
        nirio_status_chain(_process_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS, SESSION_LOCK_RETRY_INT_IN_MS), status);
        if (reset_fpga) reset();
        nirio_status_chain(_rpc_client.niusrprio_close_session(_session, 0), status);
        _process_lock.release();
    }
}

nirio_status niusrprio_session::reset()
{
    boost::unique_lock<boost::recursive_mutex> lock(_session_mutex);

    nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_process_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS, SESSION_LOCK_RETRY_INT_IN_MS), status);
	nirio_status_chain(_rpc_client.niusrprio_reset_device(_session), status);
	_process_lock.release();
	return status;
}

nirio_status niusrprio_session::download_bitstream_to_flash(const std::string& bitstream_path)
{
    boost::unique_lock<boost::recursive_mutex> lock(_session_mutex);

    nirio_status status = NiRio_Status_Success;
    nirio_status_chain(_process_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS, SESSION_LOCK_RETRY_INT_IN_MS), status);
    nirio_status_chain(_rpc_client.niusrprio_download_fpga_to_flash(_resource_name, bitstream_path), status);
    _process_lock.release();
    return status;
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

nirio_status niusrprio_session::_verify_posc_and_signature() {
    nirio_status status = 0;
    boost::uint32_t reg_data = 0xffffffff;
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration elapsed;

    static const boost::uint32_t READ_TIMEOUT_IN_MS = 100;
    static const boost::uint32_t POSC_STATUS_REG = 0x58;
    nirio_status_chain(_riok_proxy.set_attribute(kRioAddressSpace, kRioAddressSpaceBusInterface), status);
    if (nirio_status_not_fatal(status)) {
        do {
            boost::this_thread::sleep(boost::posix_time::microsec(5000)); //Avoid flooding the bus
            elapsed = boost::posix_time::microsec_clock::local_time() - start_time;
            nirio_status_chain(_riok_proxy.peek(POSC_STATUS_REG, reg_data), status);
        } while (
            nirio_status_not_fatal(status) &&
            (reg_data == 1) &&
            elapsed.total_milliseconds() < READ_TIMEOUT_IN_MS);
    }

    if (elapsed.total_milliseconds() > READ_TIMEOUT_IN_MS) nirio_status_chain(NiRio_Status_CommunicationTimeout, status);

    if (nirio_status_not_fatal(status)) {
        boost::uint32_t sig_offset = 0;
        nirio_status_chain(_riok_proxy.get_attribute(kRioFpgaDefaultSignatureOffset, sig_offset), status);
        nirio_status_chain(_riok_proxy.set_attribute(kRioAddressSpace, kRioAddressSpaceFpga), status);
        std::string signature;
        for (int i = 0; i < 8; i++) {
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
    }

    return status;
}


}
