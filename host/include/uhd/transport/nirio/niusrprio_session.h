/*
 * nifpga_interface.h
 *
 *  Created on: Mar 26, 2013
 *      Author: ashish
 */

#ifndef NIUSRPRIO_SESSION_H_
#define NIUSRPRIO_SESSION_H_

#include <uhd/transport/nirio/rpc/usrprio_rpc_client.hpp>
#include <uhd/transport/nirio/nirio_interface.h>
#include <uhd/transport/nirio/nirio_resource_manager.h>
#include <uhd/transport/nirio/locks.h>
#include <uhd/transport/nirio/nifpga_lvbitx.h>
#include <boost/noncopyable.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <string>

namespace nifpga_interface
{

class niusrprio_session : private boost::noncopyable
{
public:
    typedef boost::shared_ptr<niusrprio_session> sptr;
    typedef usrprio_rpc::usrprio_device_info device_info;
    typedef usrprio_rpc::usrprio_device_info_vtr device_info_vtr;

	static nirio_status enumerate(device_info_vtr& device_info_vtr);

	niusrprio_session(const std::string& resource_name);
	virtual ~niusrprio_session();

	nirio_status open(
        nifpga_lvbitx::sptr lvbitx,
		uint32_t attribute = 0);

	void close(bool reset_fpga = false);

	nirio_status reset();

	template<typename data_t>
	nirio_status create_tx_fifo(
		const char* fifo_name,
		nirio_interface::nirio_fifo<data_t>& fifo)
	{
		nirio_status status = _process_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS, SESSION_LOCK_RETRY_INT_IN_MS);
		if (nirio_status_not_fatal(status))
			_resource_manager.create_tx_fifo(fifo_name, fifo);
		_process_lock.release();
		return status;
	}

    template<typename data_t>
    nirio_status create_tx_fifo(
        uint32_t fifo_instance,
        nirio_interface::nirio_fifo<data_t>& fifo)
    {
        if (_lvbitx.get() == NULL) return NiRio_Status_ResourceNotInitialized;
        if ((size_t)fifo_instance >= _lvbitx->get_output_fifo_count()) return NiRio_Status_InvalidParameter;

        return create_tx_fifo(_lvbitx->get_output_fifo_names()[fifo_instance], fifo);
    }

    template<typename data_t>
	nirio_status create_rx_fifo(
		const char* fifo_name,
		nirio_interface::nirio_fifo<data_t>& fifo)
	{
		nirio_status status = _process_lock.acquire(SESSION_LOCK_TIMEOUT_IN_MS, SESSION_LOCK_RETRY_INT_IN_MS);
		if (nirio_status_not_fatal(status))
			_resource_manager.create_rx_fifo(fifo_name, fifo);
		_process_lock.release();
		return status;
	}

    template<typename data_t>
    nirio_status create_rx_fifo(
        uint32_t fifo_instance,
        nirio_interface::nirio_fifo<data_t>& fifo)
    {
        if (_lvbitx.get() == NULL) return NiRio_Status_ResourceNotInitialized;
        if ((size_t)fifo_instance >= _lvbitx->get_input_fifo_count()) return NiRio_Status_InvalidParameter;

        return create_rx_fifo(_lvbitx->get_input_fifo_names()[fifo_instance], fifo);
    }

	nirio_interface::niriok_proxy& get_kernel_proxy() {
	    return _riok_proxy;
	}

    nirio_status download_bitstream_to_flash(const std::string& bitstream_path);

    //Static
    static nirio_interface::niriok_proxy::sptr create_kernel_proxy(const std::string& resource_name);

	static const uint32_t OPEN_ATTR_SKIP_SIGNATURE_CHECK	= 1 << 31;
	static const uint32_t OPEN_ATTR_FORCE_DOWNLOAD 			= 1 << 29;

private:
	void _init_fifo_info(nirio_interface::nirio_fifo_info_vtr& vtr);

	std::string								_resource_name;
	nifpga_lvbitx::sptr                     _lvbitx;
    uint32_t                                _session;
	nirio_interface::niriok_proxy			_riok_proxy;
	nirio_interface::nirio_resource_manager	_resource_manager;
    usrprio_rpc::usrprio_rpc_client         _rpc_client;
	nifpga_session_lock						_process_lock;
	boost::recursive_mutex                  _session_mutex;

	static const uint32_t SESSION_LOCK_TIMEOUT_IN_MS    = 3000;
    static const uint32_t SESSION_LOCK_RETRY_INT_IN_MS  = 500;
};

}

#endif /* NIUSRPRIO_SESSION_H_ */
