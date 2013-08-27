#include <uhd/transport/nirio/nirio_driver_iface.h>

namespace nirio_driver_iface {

nirio_status rio_open(
    const std::string& device_path,
    rio_dev_handle_t& device_handle)
{
    return NiRio_Status_FeatureNotSupported;
}

void rio_close(rio_dev_handle_t& device_handle)
{
}

bool rio_isopen(rio_dev_handle_t device_handle)
{
    return false;
}

nirio_status rio_ioctl(
    rio_dev_handle_t device_handle,
	uint32_t ioctl_code,
	const void *write_buf,
	size_t write_buf_len,
	void *read_buf,
	size_t read_buf_len)
{
    return NiRio_Status_FeatureNotSupported;
}

nirio_status rio_mmap(
    rio_dev_handle_t device_handle,
	uint16_t memory_type,
	size_t size,
	bool writable,
	rio_mmap_t &map)
{
    return NiRio_Status_FeatureNotSupported;
}

nirio_status rio_munmap(rio_mmap_t &map)
{
    return NiRio_Status_FeatureNotSupported;
}

}
