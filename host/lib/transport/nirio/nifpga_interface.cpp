/*
 * nifpga_interface.cpp
 *
 *  Created on: Mar 26, 2013
 *      Author: ashish
 */

#include <uhd/transport/nirio/nifpga_interface.h>
#include <uhd/transport/nirio/nirio_fifo.h>
#include <uhd/transport/nirio/nifpga_image.h>
#include <uhd/transport/nirio/status.h>
#include <stdio.h>
#include "NiFpga/NiFpga.h"
#include "NiFpga/niusrprio.h"
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>

#define REDOWNLOAD_IF_BIN_SIG_MISMATCH 1

namespace nifpga_interface
{

nifpga_session::nifpga_session(const std::string& resource_name) :
    _session(0),
	_resource_name(resource_name),
	_resource_manager(_riok_proxy)
{
}

nifpga_session::~nifpga_session()
{
    close();
}

nirio_status nifpga_session::load_lib()
{
    nirio_status status = NiRio_Status_Success;
    nirio_status_chain(NiFpga_Initialize(), status);
    nirio_status_chain(niusrprio_Initialize(), status);
    return status;
}

nirio_status nifpga_session::unload_lib()
{
    nirio_status status = NiRio_Status_Success;
    nirio_status_chain(NiFpga_Finalize(), status);
    nirio_status_chain(niusrprio_Finalize(), status);
    return status;
}

nirio_status nifpga_session::enumerate(nirio_device_info_vtr& device_info_vtr)
{
    device_info_vtr.clear();
    nirio_status status = NiRio_Status_Success;

    uint64_t ndevs;
    nirio_status_chain(niusrprio_getNumberOfDevices(&ndevs), status);
    if (ndevs > 0) {
        std::vector<uint32_t> nodes(ndevs);
        std::vector<uint64_t> serials(ndevs);

        nirio_status_chain(niusrprio_getDevicesInformation(ndevs, &nodes[0], &serials[0]), status);
        for(size_t i = 0; i < ndevs && nirio_status_not_fatal(status); i++) {
            nirio_device_info info = nirio_device_info();
            info.interface_num = nodes[i];
            info.resource_name = "RIO" + boost::lexical_cast<std::string>(nodes[i]);
            info.serial_num = boost::lexical_cast<std::string>(serials[i]);
            device_info_vtr.push_back(info);
        }
    }

    return status;
}

nirio_status nifpga_session::open(
	const std::string& bitfile_path,
	const char* signature,
	uint32_t attribute)
{
	using namespace nirio_interface;

	nirio_status status = NiRio_Status_Success;
	const char* signature_without_checksum = signature ? signature + 32 : NULL;
#if REDOWNLOAD_IF_BIN_SIG_MISMATCH
	if ((attribute & OPEN_ATTR_SKIP_SIGNATURE_CHECK) == 0 && _signature != signature)
		attribute |= OPEN_ATTR_FORCE_DOWNLOAD;
#endif

	nirio_status_chain(NiFpga_Open(bitfile_path.c_str(), signature_without_checksum, _resource_name.c_str(), attribute, &_session), status);
	_lock.initialize(_session);
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);

	//@TODO: HACK: The interface number should come from the closed source helper.
	uint32_t interface_num = -1;
	if (nirio_status_not_fatal(status)) {
		boost::smatch iface_match;
		if (boost::regex_search(_resource_name, iface_match, boost::regex("RIO([0-9]*)")))
		{
			//Assuming that NiFpga_Open was fine with the name, this lexical cast will not throw.
			interface_num = boost::lexical_cast<uint32_t>(std::string(iface_match[1].first, iface_match[1].second));
		}
	}

	nirio_status_chain(
		niriok_proxy_factory::get_by_interface_num(interface_num, _riok_proxy),
		status);

	if (nirio_status_not_fatal(status)) {
		_signature = signature ? signature : "";

		nirio_register_info_vtr reg_vtr;
		nirio_fifo_info_vtr fifo_vtr;
		nifpga_image::initialize_register_info(reg_vtr);
		nifpga_image::initialize_fifo_info(fifo_vtr);
		_resource_manager.initialize(reg_vtr, fifo_vtr);
	}

	_lock.release();
	return status;
}

void nifpga_session::close(bool reset_fpga)
{
    if (_session) {
        nirio_status status = NiRio_Status_Success;
        nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
        if (reset_fpga) reset();
        nirio_status_chain(NiFpga_Close(_session, 0), status);
        _lock.release();
    }
}

nirio_status nifpga_session::reset()
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	nirio_status_chain(NiFpga_Reset(_session), status);
	_lock.release();
	return status;
}

template<>
nirio_status nifpga_session::read<int8_t>(const char* indicator_name, int8_t& value)
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	uint32_t offset;
	nirio_status_chain(_resource_manager.get_register_offset(indicator_name, offset), status);
	nirio_status_chain(NiFpga_ReadI8(_session, offset, &value), status);
	_lock.release();
	return status;
}

template<>
nirio_status nifpga_session::read<uint8_t>(const char* indicator_name, uint8_t& value)
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	uint32_t offset;
	nirio_status_chain(_resource_manager.get_register_offset(indicator_name, offset), status);
	nirio_status_chain(NiFpga_ReadU8(_session, offset, &value), status);
	_lock.release();
	return status;
}

template<>
nirio_status nifpga_session::read<int16_t>(const char* indicator_name, int16_t& value)
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	uint32_t offset;
	nirio_status_chain(_resource_manager.get_register_offset(indicator_name, offset), status);
	nirio_status_chain(NiFpga_ReadI16(_session, offset, &value), status);
	_lock.release();
	return status;
}

template<>
nirio_status nifpga_session::read<uint16_t>(const char* indicator_name, uint16_t& value)
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	uint32_t offset;
	nirio_status_chain(_resource_manager.get_register_offset(indicator_name, offset), status);
	nirio_status_chain(NiFpga_ReadU16(_session, offset, &value), status);
	_lock.release();
	return status;
}

template<>
nirio_status nifpga_session::read<int32_t>(const char* indicator_name, int32_t& value)
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	uint32_t offset;
	nirio_status_chain(_resource_manager.get_register_offset(indicator_name, offset), status);
	nirio_status_chain(NiFpga_ReadI32(_session, offset, &value), status);
	_lock.release();
	return status;
}

template<>
nirio_status nifpga_session::read<uint32_t>(const char* indicator_name, uint32_t& value)
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	uint32_t offset;
	nirio_status_chain(_resource_manager.get_register_offset(indicator_name, offset), status);
	nirio_status_chain(NiFpga_ReadU32(_session, offset, &value), status);
	_lock.release();
	return status;
}

template<>
nirio_status nifpga_session::read<int64_t>(const char* indicator_name, int64_t& value)
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	uint32_t offset;
	nirio_status_chain(_resource_manager.get_register_offset(indicator_name, offset), status);
	nirio_status_chain(NiFpga_ReadI64(_session, offset, &value), status);
	_lock.release();
	return status;
}

template<>
nirio_status nifpga_session::read<uint64_t>(const char* indicator_name, uint64_t& value)
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	uint32_t offset;
	nirio_status_chain(_resource_manager.get_register_offset(indicator_name, offset), status);
	nirio_status_chain(NiFpga_ReadU64(_session, offset, &value), status);
	_lock.release();
	return status;
}

template<>
nirio_status nifpga_session::write<int8_t>(const char* control_name, const int8_t& value)
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	uint32_t offset;
	nirio_status_chain(_resource_manager.get_register_offset(control_name, offset), status);
	nirio_status_chain(NiFpga_WriteI8(_session, offset, value), status);
	_lock.release();
	return status;
}

template<>
nirio_status nifpga_session::write<uint8_t>(const char* control_name, const uint8_t& value)
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	uint32_t offset;
	nirio_status_chain(_resource_manager.get_register_offset(control_name, offset), status);
	nirio_status_chain(NiFpga_WriteU8(_session, offset, value), status);
	_lock.release();
	return status;
}

template<>
nirio_status nifpga_session::write<int16_t>(const char* control_name, const int16_t& value)
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	uint32_t offset;
	nirio_status_chain(_resource_manager.get_register_offset(control_name, offset), status);
	nirio_status_chain(NiFpga_WriteI16(_session, offset, value), status);
	_lock.release();
	return status;
}

template<>
nirio_status nifpga_session::write<uint16_t>(const char* control_name, const uint16_t& value)
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	uint32_t offset;
	nirio_status_chain(_resource_manager.get_register_offset(control_name, offset), status);
	nirio_status_chain(NiFpga_WriteU16(_session, offset, value), status);
	_lock.release();
	return status;
}

template<>
nirio_status nifpga_session::write<int32_t>(const char* control_name, const int32_t& value)
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	uint32_t offset;
	nirio_status_chain(_resource_manager.get_register_offset(control_name, offset), status);
	nirio_status_chain(NiFpga_WriteI32(_session, offset, value), status);
	_lock.release();
	return status;
}

template<>
nirio_status nifpga_session::write<uint32_t>(const char* control_name, const uint32_t& value)
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	uint32_t offset;
	nirio_status_chain(_resource_manager.get_register_offset(control_name, offset), status);
	nirio_status_chain(NiFpga_WriteU32(_session, offset, value), status);
	_lock.release();
	return status;
}

template<>
nirio_status nifpga_session::write<int64_t>(const char* control_name, const int64_t& value)
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	uint32_t offset;
	nirio_status_chain(_resource_manager.get_register_offset(control_name, offset), status);
	nirio_status_chain(NiFpga_WriteI64(_session, offset, value), status);
	_lock.release();
	return status;
}

template<>
nirio_status nifpga_session::write<uint64_t>(const char* control_name, const uint64_t& value)
{
	nirio_status status = NiRio_Status_Success;
	nirio_status_chain(_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS), status);
	uint32_t offset;
	nirio_status_chain(_resource_manager.get_register_offset(control_name, offset), status);
	nirio_status_chain(NiFpga_WriteU64(_session, offset, value), status);
	_lock.release();
	return status;
}

}
