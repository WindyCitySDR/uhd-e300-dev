/*
 * nifpga_interface.cpp
 *
 *  Created on: Mar 26, 2013
 *      Author: ashish
 */

#include <uhd/transport/nirio/niusrprio_session.h>
#include <uhd/transport/nirio/nirio_fifo.h>
#include <uhd/transport/nirio/status.h>
#include <stdio.h>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>

#define RPC_CLIENT_ARGS "localhost", "13"

namespace nifpga_interface
{

niusrprio_session::niusrprio_session(const std::string& resource_name) :
	_resource_name(resource_name),
    _session(0),
	_resource_manager(_riok_proxy),
	_rpc_client(RPC_CLIENT_ARGS)
{
    //@TODO: HACK: The interface number should come from the closed source helper.
    try {
        boost::smatch iface_match;
        if (boost::regex_search(_resource_name, iface_match, boost::regex("RIO([0-9]*)"))) {
            _interface_num = boost::lexical_cast<uint32_t>(std::string(iface_match[1].first, iface_match[1].second));
        }
    } catch (boost::exception& e) {
        _interface_num = (uint32_t)-1;
    }
}

niusrprio_session::~niusrprio_session()
{
    close();
}

nirio_status niusrprio_session::enumerate(device_info_vtr& device_info_vtr)
{
    usrprio_rpc::usrprio_rpc_client temp_rpc_client(RPC_CLIENT_ARGS);
    nirio_status status = NiRio_Status_Success;
    nirio_status_chain(temp_rpc_client.niusrprio_initialize(), status);
    nirio_status_chain(temp_rpc_client.niusrprio_enumerate(device_info_vtr), status);
    return status;
}

nirio_status niusrprio_session::open(
    nifpga_lvbitx::sptr lvbitx,
	uint32_t attribute)
{
	using namespace nirio_interface;

	_lvbitx = lvbitx;

	nirio_status status = NiRio_Status_Success;
    nirio_status_chain(_rpc_client.niusrprio_initialize(), status);

    std::string bitfile_path(_lvbitx->get_bitfile_path());
	std::string signature_without_checksum(_lvbitx->get_signature() + 32);

	nirio_status_chain(_rpc_client.niusrprio_open_session(
        _resource_name, bitfile_path, signature_without_checksum, attribute, _session), status);

	_lock.initialize(_session);
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);

    std::string interface_path = niriok_proxy::get_interface_path(_interface_num);
    if (interface_path.empty()) nirio_status_chain(NiRio_Status_ResourceNotFound, status);

	if (nirio_status_not_fatal(status)) {
	    _riok_proxy.open(interface_path);

		nirio_register_info_vtr reg_vtr;
		nirio_fifo_info_vtr fifo_vtr;
		_lvbitx->init_register_info(reg_vtr);
		_lvbitx->init_fifo_info(fifo_vtr);
		_resource_manager.initialize(reg_vtr, fifo_vtr);
	}

	_lock.release();
	return status;
}

void niusrprio_session::close(bool reset_fpga)
{
    if (_session) {
        nirio_status status = NiRio_Status_Success;
        nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
        if (reset_fpga) reset();
        nirio_status_chain(_rpc_client.niusrprio_close_session(_session, 0), status);
        _lock.release();
    }
}

nirio_status niusrprio_session::reset()
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	nirio_status_chain(_rpc_client.niusrprio_reset_device(_session), status);
	_lock.release();
	return status;
}

nirio_status niusrprio_session::download_bitstream_to_flash(const std::string& bitstream_path)
{
    nirio_status status = NiRio_Status_Success;
    nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
    nirio_status_chain(_rpc_client.niusrprio_download_fpga_to_flash(_interface_num, bitstream_path), status);
    _lock.release();
    return status;
}

}
