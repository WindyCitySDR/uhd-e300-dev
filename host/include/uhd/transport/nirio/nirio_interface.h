/*
 * nirio_interface.h
 *
 *  Created on: Mar 20, 2013
 *      Author: ashish
 */


#ifndef NIRIO_INTERFACE_H_
#define NIRIO_INTERFACE_H_

#include <uhd/transport/nirio/RioConstants.h>
#include <uhd/transport/nirio/nirio_driver_iface.h>
#include <boost/smart_ptr.hpp>
#include <boost/noncopyable.hpp>

namespace nirio_interface
{
    class niriok_proxy : public boost::noncopyable {
    public:
        typedef boost::shared_ptr<niriok_proxy> sptr;

        enum version_t { CURRENT, OLDEST_COMPATIBLE };

    	niriok_proxy();
    	virtual ~niriok_proxy();

    	//File operations
    	nirio_status open(const std::string& interface_path);
        void close(void);

        nirio_status reset();

        inline uint32_t get_interface_num() { return _interface_num; }

        nirio_status get_cached_session(
    		uint32_t& session);

        nirio_status get_version(
    		version_t type,
    		uint32_t& major,
    		uint32_t& upgrade,
    		uint32_t& maintenance,
    		char& phase,
    		uint32_t& build);

        nirio_status sync_operation(
            const void *writeBuffer,
            size_t writeBufferLength,
            void *readBuffer,
            size_t readBufferLength);

        nirio_status get_attribute(
            const tRioDeviceAttribute32 attribute,
            uint32_t& attrValue);

        nirio_status get_attribute(
            const tRioDeviceAttributeStr  attribute,
            char* buf,
            const uint32_t bufLen,
            uint32_t& stringLen);

        nirio_status set_attribute(
            const tRioDeviceAttribute32 attribute,
            const uint32_t value);

        nirio_status set_attribute(
            const tRioDeviceAttributeStr attribute,
            const char* const buffer);

        template<typename data_t>
        nirio_status peek(uint32_t offset, data_t& value);

        template<typename data_t>
        nirio_status poke(uint32_t offset, const data_t& value);

        nirio_status map_fifo_memory(
        	uint32_t fifo_instance,
        	size_t size,
        	nirio_driver_iface::rio_mmap_t& map);

        nirio_status unmap_fifo_memory(
        	nirio_driver_iface::rio_mmap_t& map);

    private:	//Members
        nirio_driver_iface::rio_dev_handle_t    _device_handle;
        uint32_t 	                            _interface_num;
    };
}

#endif /* NIRIO_INTERFACE_H_ */
