/*
 * nirio_interface.hpp
 *
 *  Created on: Mar 20, 2013
 *      Author: ashish
 */

#include <uhd/transport/nirio/nirio_interface.h>
#include <stdio.h>
#include <errno.h>

#define PATH_MAX 		4096
#define NI_VENDOR_NUM 	0x1093

#define VERSION_BUILD_SHIFT		0
#define VERSION_PHASE_SHIFT		14
#define VERSION_MAINT_SHIFT		16
#define VERSION_UPGRD_SHIFT		20
#define VERSION_MAJOR_SHIFT		24
#define VERSION_BUILD_MASK		0x00003FFF
#define VERSION_PHASE_MASK		0x0000C000
#define VERSION_MAINT_MASK		0x000F0000
#define VERSION_UPGRD_MASK		0x00F00000
#define VERSION_MAJOR_MASK		0xFF000000

#define GET_FIFO_MEMORY_TYPE(fifo_inst) (static_cast<uint16_t>(0x0100 | static_cast<uint16_t>(fifo_inst)))

//@TODO: Figure out a better way to suppress anonymous struct init warnings
#ifdef __GNUC__
    #pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif

namespace nirio_interface
{
	//-------------------------------------------------------
	// niriok_proxy
	//-------------------------------------------------------
	niriok_proxy::niriok_proxy(): _device_handle(nirio_driver_iface::INVALID_RIO_HANDLE)
	{
	}

	niriok_proxy::~niriok_proxy()
	{
	}

	nirio_status niriok_proxy::open(const std::string& interface_path)
    {
        nirio_status status = NiRio_Status_Success;

        //close if already open.
        close();

        nirio_status_chain(nirio_driver_iface::rio_open(
            interface_path, _device_handle), status);
        if (nirio_status_not_fatal(status)) {
            nirio_status_chain(nirio_driver_iface::rio_ioctl(_device_handle,
                                        nNIRIOSRV200::kRioIoctlPostOpen,
                                        NULL, 0, NULL, 0), status);
            nNIRIOSRV200::tIoctlPacketOut out(&_interface_num, sizeof(_interface_num), 0);
            nirio_status_chain(nirio_driver_iface::rio_ioctl(_device_handle,
                                        nNIRIOSRV200::kRioIoctlGetInterfaceNumber,
                                        NULL, 0,
                                        &out, sizeof(out)), status);

            if (nirio_status_fatal(status)) close();
        }
		return status;
    }

    void niriok_proxy::close(void)
    {
       if(nirio_driver_iface::rio_isopen(_device_handle))
       {
			nirio_driver_iface::rio_ioctl(
				_device_handle, nNIRIOSRV200::kRioIoctlPreClose, NULL, 0, NULL, 0);
            nirio_driver_iface::rio_close(_device_handle);
       }
    }

    nirio_status niriok_proxy::reset()
    {
		nNIRIOSRV200::tRioDeviceSocketInputParameters in = {};
		nNIRIOSRV200::tRioDeviceSocketOutputParameters out = {};

		in.function = nNIRIOSRV200::nRioFunction::kReset;

		return sync_operation(&in, sizeof(in), &out, sizeof(out));
    }

    nirio_status niriok_proxy::get_cached_session(
		uint32_t& session)
	{
		nNIRIOSRV200::tIoctlPacketOut out(&session, sizeof(session), 0);
		return nirio_driver_iface::rio_ioctl(_device_handle,
									nNIRIOSRV200::kRioIoctlGetSession,
									NULL, 0,
									&out, sizeof(out));
	}

    nirio_status niriok_proxy::get_version(
		version_t type,
		uint32_t& major,
		uint32_t& upgrade,
		uint32_t& maintenance,
		char& phase,
		uint32_t& build)
	{
		tRioDeviceAttribute32 version_attr = (type==CURRENT)?kRioCurrentVersion:kRioOldestCompatibleVersion;
		uint32_t raw_version = 0;
		nirio_status status = get_attribute(version_attr, raw_version);

		major		= (raw_version & VERSION_MAJOR_MASK) >> VERSION_MAJOR_SHIFT;
		upgrade		= (raw_version & VERSION_UPGRD_MASK) >> VERSION_UPGRD_SHIFT;
		maintenance	= (raw_version & VERSION_MAINT_MASK) >> VERSION_MAINT_SHIFT;
		build		= (raw_version & VERSION_BUILD_MASK) >> VERSION_BUILD_SHIFT;

		uint32_t phase_num = (raw_version & VERSION_PHASE_MASK) >> VERSION_PHASE_SHIFT;
		switch (phase_num) {
			case 0: phase = 'd'; break;
			case 1: phase = 'a'; break;
			case 2: phase = 'b'; break;
			case 3: phase = 'f'; break;
		}

		return status;
	}

    nirio_status niriok_proxy::sync_operation(
		const void *writeBuffer,
		size_t writeBufferLength,
		void *readBuffer,
		size_t readBufferLength)
	{
		nNIRIOSRV200::tIoctlPacketOut out(readBuffer, readBufferLength, 0);
		nirio_status ioctl_status = nirio_driver_iface::rio_ioctl(_device_handle,
									nNIRIOSRV200::kRioIoctlSyncOp,
									writeBuffer, writeBufferLength,
									&out, sizeof(out));
		if (nirio_status_fatal(ioctl_status)) return ioctl_status;

		return out.statusCode;
	}

    nirio_status niriok_proxy::get_attribute(
		const tRioDeviceAttribute32 attribute,
		uint32_t& attrValue)
	{
		nNIRIOSRV200::tRioDeviceSocketInputParameters in = {};
		nNIRIOSRV200::tRioDeviceSocketOutputParameters out = {};

		in.function = nNIRIOSRV200::nRioFunction::kGet32;
		in.params.attribute32.attribute = attribute;

		nirio_status status = sync_operation(&in, sizeof(in), &out, sizeof(out));

		attrValue = out.params.attribute32.value;
		return status;
	}

    nirio_status niriok_proxy::get_attribute(
		const tRioDeviceAttributeStr  attribute,
		char *buf,
		const uint32_t bufLen,
		uint32_t& stringLen)
	{
		nNIRIOSRV200::tRioDeviceSocketInputParameters in = {};
		nNIRIOSRV200::tRioDeviceSocketOutputParameters out = {};
		nNIRIOSRV200::initRioDeviceSocketOutputParameters(out, buf, bufLen);

		in.function = nNIRIOSRV200::nRioFunction::kGetString;
		in.params.attributeStr.attribute = attribute;

		nirio_status status = sync_operation(&in, sizeof(in), &out, sizeof(out));

		stringLen = out.params.stringLength;
		return status;
	}

    nirio_status niriok_proxy::set_attribute(
		const tRioDeviceAttribute32 attribute,
		const uint32_t value)
	{
		nNIRIOSRV200::tRioDeviceSocketInputParameters in = {};
		nNIRIOSRV200::tRioDeviceSocketOutputParameters out = {};

	   in.function = nNIRIOSRV200::nRioFunction::kSet32;
	   in.params.attribute32.attribute = attribute;
	   in.params.attribute32.value  = value;

	   return sync_operation(&in, sizeof(in), &out, sizeof(out));
	}

    nirio_status niriok_proxy::set_attribute(
		const tRioDeviceAttributeStr attribute,
		const char* const buffer)
	{
		nNIRIOSRV200::tRioDeviceSocketInputParameters in = {};
		nNIRIOSRV200::initRioDeviceSocketInputParameters(in, buffer, strlen(buffer) + 1);
		nNIRIOSRV200::tRioDeviceSocketOutputParameters out = {};

		in.function = nNIRIOSRV200::nRioFunction::kSetString;
		in.params.attributeStr.attribute = attribute;

		return sync_operation(&in, sizeof(in), &out, sizeof(out));
	}

    template<>
    nirio_status niriok_proxy::peek(uint32_t offset, uint32_t& value)
    {
		nNIRIOSRV200::tRioDeviceSocketInputParameters in = {};
		nNIRIOSRV200::tRioDeviceSocketOutputParameters out = {};

		in.function    		= nNIRIOSRV200::nRioFunction::kIO;
		in.subfunction 		= nNIRIOSRV200::nRioDeviceIOFunction::kPeek32;
		in.params.io.offset = offset;

		nirio_status status = sync_operation(&in, sizeof(in), &out, sizeof(out));
		value = out.params.io.value.value32;
		return status;
    }

    template<>
    nirio_status niriok_proxy::peek(uint32_t offset, uint64_t& value)
    {
		nNIRIOSRV200::tRioDeviceSocketInputParameters in = {};
		nNIRIOSRV200::tRioDeviceSocketOutputParameters out = {};

		in.function    		= nNIRIOSRV200::nRioFunction::kIO;
		in.subfunction 		= nNIRIOSRV200::nRioDeviceIOFunction::kPeek64;
		in.params.io.offset = offset;

		nirio_status status = sync_operation(&in, sizeof(in), &out, sizeof(out));
		value = out.params.io.value.value64;
		return status;
    }

    template<>
    nirio_status niriok_proxy::poke(uint32_t offset, const uint8_t& value)
    {
		nNIRIOSRV200::tRioDeviceSocketInputParameters in = {};
		nNIRIOSRV200::tRioDeviceSocketOutputParameters out = {};

		in.function    				= nNIRIOSRV200::nRioFunction::kIO;
		in.subfunction 				= nNIRIOSRV200::nRioDeviceIOFunction::kPoke8;
		in.params.io.offset    		= offset;
		in.params.io.value.value32 	= value;

		return sync_operation(&in, sizeof(in), &out, sizeof(out));
    }

    template<>
    nirio_status niriok_proxy::poke(uint32_t offset, const uint32_t& value)
    {
		nNIRIOSRV200::tRioDeviceSocketInputParameters in = {};
		nNIRIOSRV200::tRioDeviceSocketOutputParameters out = {};

		in.function    				= nNIRIOSRV200::nRioFunction::kIO;
		in.subfunction 				= nNIRIOSRV200::nRioDeviceIOFunction::kPoke32;
		in.params.io.offset    		= offset;
		in.params.io.value.value32 	= value;

		return sync_operation(&in, sizeof(in), &out, sizeof(out));
    }

    template<>
    nirio_status niriok_proxy::poke(uint32_t offset, const uint64_t& value)
    {
		nNIRIOSRV200::tRioDeviceSocketInputParameters in = {};
		nNIRIOSRV200::tRioDeviceSocketOutputParameters out = {};

		in.function    				= nNIRIOSRV200::nRioFunction::kIO;
		in.subfunction 				= nNIRIOSRV200::nRioDeviceIOFunction::kPoke64;
		in.params.io.offset    		= offset;
		in.params.io.value.value64 	= value;

		return sync_operation(&in, sizeof(in), &out, sizeof(out));
    }

    nirio_status niriok_proxy::map_fifo_memory(
    	uint32_t fifo_instance,
    	size_t size,
    	nirio_driver_iface::rio_mmap_t& map)
    {
		return nirio_driver_iface::rio_mmap(_device_handle,
				GET_FIFO_MEMORY_TYPE(fifo_instance),
				size, true, map);
    }

    nirio_status niriok_proxy::unmap_fifo_memory(
    	nirio_driver_iface::rio_mmap_t& map)
    {
		return nirio_driver_iface::rio_munmap(map);
    }

//@TODO: Ashish: This function should be removed when enumeration is done
//               using niusrprio / helper.
#if defined(UHD_PLATFORM_LINUX)
    #include <glob.h>

    std::string niriok_proxy::get_interface_path(
        uint32_t interface_num)
    {
        glob_t glob_instance;
        nirio_status status = glob("/proc/driver/ni/*/deviceInterfaces/nirio_transport\\\\*", GLOB_ONLYDIR, NULL, &glob_instance);

        if (nirio_status_fatal(status) || status == GLOB_NOMATCH) return "";

        std::string iface_path;

        for (size_t i = 0; i < glob_instance.gl_pathc && iface_path.empty(); i++) {
            char* current_path = glob_instance.gl_pathv[i];

            char path[PATH_MAX];
            snprintf(path, sizeof(path), "%s/interfacePath", current_path);

            FILE* prop_file_ptr = fopen(path, "r");
            if (prop_file_ptr == NULL) return "";
            char file_buffer[4096];
            size_t num_read = fread(file_buffer, 1 /* size of element */, sizeof(file_buffer), prop_file_ptr);
            fclose(prop_file_ptr);

            niriok_proxy temp_proxy;
            status = temp_proxy.open(file_buffer);

            if (nirio_status_fatal(status) && num_read > 0) return "";

            uint32_t actual_interface_num = -1;
            status = temp_proxy.get_attribute(kRioInterfaceNumber, actual_interface_num);
            if (nirio_status_not_fatal(status) && actual_interface_num == interface_num)
                iface_path = file_buffer;

            temp_proxy.close();
        }
        return iface_path;
    }
#elif defined(UHD_PLATFORM_WIN32)
    #include <setupapi.h>
    #include <guiddef.h>

    LPWSTR MAGICAL_GUID = L"{C7110F75-8354-442E-8EE8-24556DC68714}";

    std::string niriok_proxy::get_interface_path(
        uint32_t interface_num)
    {
        GUID                       interfaceGUID_;
        HDEVINFO                   devInfoSet_ = INVALID_HANDLE_VALUE;
        DWORD                      indexIntoSet_ = 0;
        SP_DEVICE_INTERFACE_DATA   deviceInterfaceData_;
        BOOL                       isValid = false;
        deviceInterfaceData_.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

        CLSIDFromString(MAGICAL_GUID, (LPCLSID)&interfaceGUID_);

        DWORD flags = DIGCF_PRESENT | DIGCF_DEVICEINTERFACE;

        devInfoSet_ = SetupDiGetClassDevs( &interfaceGUID_,
                            NULL, // Define no enumerator (global)
                            NULL, // Define no parent
                            flags);

        if( devInfoSet_ == INVALID_HANDLE_VALUE )
        {
            return "";
        }

        isValid = SetupDiEnumDeviceInterfaces(  devInfoSet_,
                                    NULL,
                                    &interfaceGUID_,
                                    indexIntoSet_,
                                    &deviceInterfaceData_ );

        std::string iface_path;

        while (isValid && !iface_path.empty())
        {
            size_t BUF_SIZE = 1024;
            char buffer[1024];

            PSP_DEVICE_INTERFACE_DETAIL_DATA_A diDetail =
                reinterpret_cast<PSP_DEVICE_INTERFACE_DETAIL_DATA_A>(buffer);
            DWORD diDetailRealSize = static_cast<DWORD>(BUF_SIZE);
            DWORD diDetailNeededSize;
            BOOL pass;

            diDetail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);
            pass = SetupDiGetDeviceInterfaceDetailA(
                                                    devInfoSet_,
                                                    &deviceInterfaceData_,
                                                    diDetail,
                                                    diDetailRealSize,
                                                    &diDetailNeededSize,
                                                    NULL
                                                    );

            if (diDetailNeededSize > BUF_SIZE)
            {
                return "";
            }

            if (pass)
            {
                std::string temp_str;
                // if success, shift back the data that's now at an offset inside the diDetail
                size_t length = strlen(diDetail->DevicePath)+1;
                memmove(buffer, buffer+offsetof(SP_DEVICE_INTERFACE_DETAIL_DATA_A, DevicePath), length);
                buffer[length] = '\0';

                nirio_status status = 0;
                niriok_proxy temp_proxy;
                status = temp_proxy.open(buffer);

                if (nirio_status_fatal(status)) return "";

                uint32_t actual_interface_num = -1;
                status = temp_proxy.get_attribute(kRioInterfaceNumber, actual_interface_num);
                if (nirio_status_not_fatal(status) && actual_interface_num == interface_num)
                    iface_path = buffer;

                temp_proxy.close();
            }

            indexIntoSet_++;
            isValid = SetupDiEnumDeviceInterfaces( devInfoSet_,
                                                    NULL,
                                                    &interfaceGUID_,
                                                    indexIntoSet_,
                                                    &deviceInterfaceData_ );
        }

        return iface_path;
    }
#else
    #error OS not supported by niriok_proxy::get_interface_path.
#endif

}

#ifdef __GNUC__
    #pragma GCC diagnostic pop
#endif
