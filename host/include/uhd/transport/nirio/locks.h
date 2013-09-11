
#ifndef LOCKS_H_
#define LOCKS_H_

#include <uhd/transport/nirio/rpc/usrprio_rpc_client.hpp>
#include <uhd/transport/nirio/status.h>

namespace nifpga_interface
{

class nifpga_session_lock
{
public:
	nifpga_session_lock(): _rpc_client_ptr(NULL), _session(0) {}
	virtual ~nifpga_session_lock() {}

	void initialize(usrprio_rpc::usrprio_rpc_client& rpc_client_ptr, uint32_t session) {
		_rpc_client_ptr = &rpc_client_ptr;
		_session = session;
	}

	nirio_status acquire(uint32_t timeout_ms, uint32_t retry_interval_ms) {
        if (_rpc_client_ptr == NULL) return NiRio_Status_SoftwareFault;

        boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration elapsed;

        nirio_status status = NiRio_Status_Success;
        do {
            if (_check_lock()) {
                status = NiRio_Status_Success;
            } else {
                status = NiRio_Status_DeviceLocked;
                boost::this_thread::sleep(boost::posix_time::milliseconds(retry_interval_ms));
            }
            elapsed = boost::posix_time::microsec_clock::local_time() - start_time;
        } while (
            nirio_status_fatal(status) &&
            elapsed.total_milliseconds() < timeout_ms);

        return status;
	}

	void release() {
        //NOP
	}

	bool _check_lock() {
	    boost::uint16_t session_locked = 0;
	    nirio_status status = _rpc_client_ptr->niusrprio_query_session_lock(_session, session_locked);
        return nirio_status_not_fatal(status) && !session_locked;
	}

private:
	usrprio_rpc::usrprio_rpc_client*    _rpc_client_ptr;
	uint32_t                            _session;
};

}

namespace nirio_interface
{

class nirio_fifo_lock
{
public:
	nirio_fifo_lock(uint32_t device_interface, uint32_t fifo_instance):
		_device_interface(device_interface),
		_fifo_instance(fifo_instance) {}
	virtual ~nirio_fifo_lock() {}

	nirio_status acquire(uint32_t timeout) {
        //@TODO: The closed helper should implement the shared memory mutex operations.
        timeout++;
		return NiRio_Status_Success;
	}

	void release() {
        //@TODO: The closed helper should implement the shared memory mutex operations.
	}

private:
	uint32_t        _device_interface;
	uint32_t        _fifo_instance;
};

}

#endif /* LOCKS_H_ */
