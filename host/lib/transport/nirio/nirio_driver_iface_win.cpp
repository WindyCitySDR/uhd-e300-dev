#include <uhd/transport/nirio/nirio_driver_iface.h>
#include <process.h>

namespace nirio_driver_iface {

nirio_status rio_open(
    const char* device_path,
    rio_dev_handle_t& device_handle)
{
    //@TODO: Implement me!
    return -1;
}

void rio_close(rio_dev_handle_t& device_handle)
{
    //@TODO: Implement me!
}

bool rio_isopen(rio_dev_handle_t device_handle)
{
    //@TODO: Implement me!
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
    //@TODO: Implement me!
    return -1;
}

nirio_status rio_mmap(
    rio_dev_handle_t device_handle,
    uint16_t memory_type,
    size_t size,
    int access_mode,
    rio_mmap_t &map)
{
    //@TODO: Implement me!
    return -1;
}

nirio_status rio_munmap(rio_mmap_t &map)
{
    //@TODO: Implement me!
    return -1;
}

}
