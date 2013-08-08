/*
 * nifpga_interface.h
 *
 *  Created on: Mar 26, 2013
 *      Author: ashish
 */

#ifndef NIFPGA_INTERFACE_H_
#define NIFPGA_INTERFACE_H_

#include <uhd/transport/nirio/nirio_interface.h>
#include <uhd/transport/nirio/nirio_resource_manager.h>
#include <uhd/transport/nirio/locks.h>
#include <boost/smart_ptr.hpp>
#include <string>

namespace nifpga_interface
{

class nifpga_session
{
public:
    typedef boost::shared_ptr<nifpga_session> sptr;

    struct nirio_device_info {
        uint32_t interface_num;
        std::string resource_name;
        std::string serial_num;
    };
    typedef std::vector<nirio_device_info> nirio_device_info_vtr;

	static nirio_status load_lib();
	static nirio_status unload_lib();

	static nirio_status enumerate(nirio_device_info_vtr& device_info_vtr);

	nifpga_session(const std::string& resource_name);
	virtual ~nifpga_session();

	nirio_status open(
		const std::string& bitfile_path,
		const char* signature,
		uint32_t attribute = 0);

	void close(bool reset_fpga = false);

	nirio_status reset();

	template<typename data_t>
	nirio_status read(
		const char* indicator_name,
		data_t& value);

	template<typename data_t>
	nirio_status write(
		const char* control_name,
		const data_t& value);

	template<typename data_t>
	nirio_status create_tx_fifo(
		const char* fifo_name,
		nirio_interface::nirio_fifo<data_t>& fifo)
	{
		nirio_status status = _lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS);
		if (nirio_status_not_fatal(status))
			_resource_manager.create_tx_fifo(fifo_name, fifo);
		_lock.release();
		return status;
	}

	template<typename data_t>
	nirio_status create_rx_fifo(
		const char* fifo_name,
		nirio_interface::nirio_fifo<data_t>& fifo)
	{
		nirio_status status = _lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS);
		if (nirio_status_not_fatal(status))
			_resource_manager.create_rx_fifo(fifo_name, fifo);
		_lock.release();
		return status;
	}

	nirio_interface::niriok_proxy& get_kernel_proxy()
	{
	    return _riok_proxy;
	}

	static const uint32_t OPEN_ATTR_SKIP_SIGNATURE_CHECK	= 1 << 31;
	static const uint32_t OPEN_ATTR_FORCE_DOWNLOAD 			= 1 << 29;

private:
	void _init_fifo_info(nirio_interface::nirio_fifo_info_vtr& vtr);

	uint32_t								_session;
	std::string								_resource_name;
	std::string								_signature;
	nirio_interface::niriok_proxy			_riok_proxy;
	nirio_interface::nirio_resource_manager	_resource_manager;
	nifpga_session_lock						_lock;

	static const uint32_t SESSION_LOCK_TIMEOUT_IN_MS = 5000;
};

}

#endif /* NIFPGA_INTERFACE_H_ */
