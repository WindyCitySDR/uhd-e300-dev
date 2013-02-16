//
// Copyright 2013 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

// constants coded into the fpga parameters
#define ZF_CONFIG_BASE 0x40000000
#define ZF_PAGE_WIDTH 10
#define H2S_STREAMS_WIDTH 1
#define H2S_ACONFIG_WIDTH 2
#define S2H_STREAMS_WIDTH 1
#define S2H_ACONFIG_WIDTH 2

// calculate more useful constants for this module
#define ZF_PAGE_SIZE    (1 << ZF_PAGE_WIDTH)
#define H2S_NUM_STREAMS (1 << H2S_STREAMS_WIDTH)
#define H2S_NUM_ACONFIG (1 << H2S_ACONFIG_WIDTH)
#define S2H_NUM_STREAMS (1 << S2H_STREAMS_WIDTH)
#define S2H_NUM_ACONFIG (1 << S2H_ACONFIG_WIDTH)

//offsets for the arbiter memory map
#define ARBITER_INIT_OFF    (0x0 << (ZF_PAGE_WIDTH-5))
#define ARBITER_HOST_OFF    (0x4 << (ZF_PAGE_WIDTH-5))
#define ARBITER_DEVICE_OFF  (0x5 << (ZF_PAGE_WIDTH-5))
#define ARBITER_BEGIN_OFF   (0x6 << (ZF_PAGE_WIDTH-5))
#define ARBITER_END_OFF     (0x7 << (ZF_PAGE_WIDTH-5))
#define ARBITER_ADDR_OFF    (0x8 << (ZF_PAGE_WIDTH-5))
#define ARBITER_SIZE_OFF    (0xC << (ZF_PAGE_WIDTH-5))

//helper macros to determine config addrs
#define H2S_BASE(base) (size_t(base) + (ZF_PAGE_SIZE*1))
#define S2H_BASE(base) (size_t(base) + (ZF_PAGE_SIZE*0))

#include <boost/cstdint.hpp>

inline void zf_poke32(const boost::uint32_t addr, const boost::uint32_t data)
{
    volatile boost::uint32_t *p = (boost::uint32_t *)addr;
    *p = data;
}

inline boost::uint32_t zf_peek32(const boost::uint32_t addr)
{
    volatile const boost::uint32_t *p = (const boost::uint32_t *)addr;
    return *p;
}

#include "e200_fifo_config.hpp"
#include <sys/mman.h> //mmap
#include <fcntl.h> //open, close
#include <uhd/utils/log.hpp>

#define DEFAULT_FRAME_SIZE 16
#define DEFAULT_NUM_FRAMES 2

using namespace uhd;
using namespace uhd::transport;

/***********************************************************************
 * managed buffer
 **********************************************************************/
struct e200_fifo_shadow
{
    size_t base_addr;
    size_t which_stream;

    size_t host_ptr;
    size_t device_ptr;
    size_t begin_ptr;
    size_t end_ptr;

    UHD_INLINE void enable(const bool enb)
    {
        zf_poke32(base_addr + ARBITER_INIT_OFF + which_stream*4, enb?1:0);
    }

    UHD_INLINE void do_sync(void)
    {
        zf_poke32(base_addr + ARBITER_HOST_OFF + which_stream*4, host_ptr);
        zf_poke32(base_addr + ARBITER_DEVICE_OFF + which_stream*4, device_ptr);
        zf_poke32(base_addr + ARBITER_BEGIN_OFF + which_stream*4, begin_ptr);
        zf_poke32(base_addr + ARBITER_END_OFF + which_stream*4, end_ptr);
    }

    UHD_INLINE void readback(void)
    {
        device_ptr = zf_peek32(base_addr + ARBITER_DEVICE_OFF + which_stream*4);
    }

    UHD_INLINE void do_release(const size_t ptr, const size_t addr, const size_t size)
    {
        host_ptr = ptr;
        zf_poke32(base_addr + ARBITER_ADDR_OFF + ptr*4, addr);
        zf_poke32(base_addr + ARBITER_SIZE_OFF + ptr*4, size);
        zf_poke32(base_addr + ARBITER_HOST_OFF + which_stream*4, host_ptr);
    }
};

struct e200_fifo_mb : managed_buffer
{
    e200_fifo_mb(const size_t ptr, void *mem, const size_t len, e200_fifo_shadow &shadow):
        ptr(ptr), mem(mem), len(len), shadow(shadow){}

    void release(void)
    {
        shadow.do_release(ptr, size_t(mem), this->size());
    }

    template <typename T>
    UHD_INLINE typename T::sptr get_new(void)
    {
        return make(reinterpret_cast<T *>(this), mem, len);
    }

    const size_t ptr;
    void *mem;
    const size_t len;
    e200_fifo_shadow &shadow;
};

/***********************************************************************
 * transport
 **********************************************************************/
template <typename BaseClass>
struct e200_transport : zero_copy_if
{

    e200_transport(const e200_fifo_shadow &shadow, const size_t data_base, const size_t num_frames, const size_t frame_size):
        _shadow(shadow), _num_frames(num_frames), _frame_size(frame_size), _index(0)
    {
        _shadow.enable(false);
        for (size_t i = 0; i < num_frames; i++)
        {
            void *mem = (void *)(data_base + (i*frame_size));
            boost::shared_ptr<e200_fifo_mb> mb;
            mb.reset(new e200_fifo_mb(shadow.begin_ptr+i, mem, frame_size, _shadow));
            _buffs.push_back(mb);
        }
        _shadow.do_sync();
        _shadow.enable(true);
    }

    template <typename T>
    UHD_INLINE typename T::sptr get_buff(const double)
    {
        //TODO probably wait in poll
        //look for device ptr
        {
            _shadow.readback();
        }

        if (_index == _num_frames) _index = 0;
        return _buffs[_index++]->get_new<T>();
    }

    managed_recv_buffer::sptr get_recv_buff(const double timeout)
    {
        return this->get_buff<managed_recv_buffer>(timeout);
    }

    size_t get_num_recv_frames(void) const
    {
        return _num_frames;
    }

    size_t get_recv_frame_size(void) const
    {
        return _frame_size;
    }

    managed_send_buffer::sptr get_send_buff(const double timeout)
    {
        return this->get_buff<managed_send_buffer>(timeout);
    }

    size_t get_num_send_frames(void) const
    {
        return _num_frames;
    }

    size_t get_send_frame_size(void) const
    {
        return _frame_size;
    }

    e200_fifo_shadow _shadow;
    const size_t _num_frames;
    const size_t _frame_size;
    size_t _index;
    std::vector<boost::shared_ptr<e200_fifo_mb> > _buffs;
};

/***********************************************************************
 * memory mapping
 **********************************************************************/
struct e200_fifo_interface_impl : e200_fifo_interface
{
    e200_fifo_interface_impl(const e200_fifo_config_t &config):
        config(config),
        bytes_in_use(0),
        recv_entries_in_use(0),
        send_entries_in_use(0)
    {
        //open the file descriptor to our kernel module
        const std::string dev = "/dev/axi_fpga";
        fd = ::open(dev.c_str(), O_RDWR|O_SYNC);
        if (fd < 0)
        {
            throw uhd::runtime_error("E200: failed to open " + dev);
        }

        //mmap the control and data regions into virtual space
        void *buff = ::mmap(NULL, config.ctrl_length + config.buff_length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
        if (buff == MAP_FAILED)
        {
            ::close(fd);
            throw uhd::runtime_error("E200: failed to mmap " + dev);
        }

        //segment the memory according to zynq fifo arbiter
        ctrl_space = size_t(buff);
        data_space = size_t(buff) + config.ctrl_length;
    }

    ~e200_fifo_interface_impl(void)
    {
        UHD_LOG << "cleanup: munmap" << std::endl;
        ::munmap(buff, config.ctrl_length + config.buff_length);
        ::close(fd);
    }

    uhd::transport::zero_copy_if::sptr make_xport(const size_t which_stream, const uhd::device_addr_t &args, const bool is_recv)
    {
        const size_t frame_size(size_t(args.cast<double>((is_recv)? "recv_frame_size" : "send_frame_size", DEFAULT_FRAME_SIZE)));
        const size_t num_frames(size_t(args.cast<double>((is_recv)? "num_recv_frames" : "num_send_frames", DEFAULT_NUM_FRAMES)));
        size_t &entries_in_use = (is_recv)? recv_entries_in_use : send_entries_in_use;

        e200_fifo_shadow shadow;
        shadow.which_stream = which_stream;
        shadow.base_addr = (is_recv)? S2H_BASE(ctrl_space) : H2S_BASE(ctrl_space);
        shadow.begin_ptr = entries_in_use;
        shadow.end_ptr = entries_in_use + num_frames;
        shadow.host_ptr = shadow.begin_ptr;
        shadow.device_ptr = shadow.begin_ptr;

        uhd::transport::zero_copy_if::sptr xport;
        if (is_recv) xport.reset(new e200_transport<managed_recv_buffer>(shadow, data_space+bytes_in_use, num_frames, frame_size));
        else         xport.reset(new e200_transport<managed_send_buffer>(shadow, data_space+bytes_in_use, num_frames, frame_size));

        bytes_in_use += num_frames*frame_size;
        entries_in_use += num_frames;

        UHD_ASSERT_THROW(recv_entries_in_use <= S2H_NUM_ACONFIG);
        UHD_ASSERT_THROW(send_entries_in_use <= H2S_NUM_ACONFIG);
        UHD_ASSERT_THROW(bytes_in_use <= config.buff_length);

        return xport;
    }

    uhd::transport::zero_copy_if::sptr make_recv_xport(const size_t which_stream, const uhd::device_addr_t &args)
    {
        return this->make_xport(which_stream, args, true);
    }

    uhd::transport::zero_copy_if::sptr make_send_xport(const size_t which_stream, const uhd::device_addr_t &args)
    {
        return this->make_xport(which_stream, args, false);
    }

    e200_fifo_config_t config;
    size_t bytes_in_use;
    int fd;
    void *buff;
    size_t ctrl_space;
    size_t data_space;
    size_t recv_entries_in_use;
    size_t send_entries_in_use;
};

e200_fifo_interface::sptr e200_fifo_interface::make(const e200_fifo_config_t &config)
{
    return e200_fifo_interface::sptr(new e200_fifo_interface_impl(config));
}
