
#ifndef LOCKS_H_
#define LOCKS_H_

#include <uhd/transport/nirio/status.h>
#include <boost/thread/recursive_mutex.hpp>
#if defined(UHD_PLATFORM_WIN32)
    #include <Windows.h>
#else
    #include <unistd.h>
#endif

namespace nifpga_interface
{

class nifpga_session_lock
{
public:
	nifpga_session_lock(): _session(0), _pid(0) {}
	virtual ~nifpga_session_lock() {}

	void initialize(uint32_t session) {
		_session = session;
#if defined(UHD_PLATFORM_WIN32)
        _pid = GetCurrentProcessId();
#else
        _pid = getpid();
#endif
	}

	nirio_status acquire(uint32_t timeout) {
        //@TODO: The closed helper should implement the shared memory mutex operations.
        _mutex.lock();
        timeout++;
        return NiRio_Status_Success;
	}

	void release() {
        //@TODO: The closed helper should implement the shared memory mutex operations.
        _mutex.unlock();
	}

private:
	uint32_t 	            _session;
	int 		            _pid;
    boost::recursive_mutex  _mutex;
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
