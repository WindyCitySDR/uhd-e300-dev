//
// Copyright 2013-2014 Ettus Research LLC
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

#ifdef E300_NATIVE

#include <boost/cstdint.hpp>
#include <uhd/config.hpp>

// constants coded into the fpga parameters
static const size_t ZF_CONFIG_BASE    = 0x40000000;
static const size_t ZF_PAGE_WIDTH     = 10;
static const size_t H2S_STREAMS_WIDTH = 3;
static const size_t H2S_CMDFIFO_DEPTH = 10;
static const size_t S2H_STREAMS_WIDTH = 3;
static const size_t S2H_CMDFIFO_DEPTH = 10;

// calculate more useful constants for this module
static const size_t ZF_PAGE_SIZE(1 << ZF_PAGE_WIDTH);
static const size_t H2S_NUM_STREAMS(1 << H2S_STREAMS_WIDTH);
static const size_t H2S_NUM_CMDS(1 << H2S_CMDFIFO_DEPTH);
static const size_t S2H_NUM_STREAMS(1 << S2H_STREAMS_WIDTH);
static const size_t S2H_NUM_CMDS(1 << S2H_CMDFIFO_DEPTH);

//offsetsinto the arbiter memory map
static const size_t ARBITER_WR_CLEAR      = 0;
static const size_t ARBITER_RD_SIG        = 0;
static const size_t ARBITER_WR_ADDR       = 4;
static const size_t ARBITER_WR_SIZE       = 8;
static const size_t ARBITER_WR_STS_RDY    = 12;
static const size_t ARBITER_WR_STS        = 16;
static const size_t ARBITER_RB_STATUS     = 16;
static const size_t ARBITER_RB_STATUS_OCC = 20;
static const size_t ARBITER_RB_ADDR_SPACE = 24;
static const size_t ARBITER_RB_SIZE_SPACE = 28;

// registers for the wb32_iface
static const size_t SR_CORE_READBACK = 0;


static UHD_INLINE size_t S2H_BASE(const size_t base)
{
    return base + ZF_PAGE_SIZE * 0;
}

static UHD_INLINE size_t H2S_BASE(const size_t base)
{
    return base + ZF_PAGE_SIZE * 1;
}

static UHD_INLINE size_t REG_BASE(const size_t base)
{
    return base + ZF_PAGE_SIZE * 2;
}

static UHD_INLINE size_t DST_BASE(const size_t base)
{
    return base + ZF_PAGE_SIZE * 3;
}

static UHD_INLINE size_t ZF_STREAM_OFF(const size_t which)
{
    return which * 32;
}

#include "e300_fifo_config.hpp"
#include <sys/mman.h> //mmap
#include <fcntl.h> //open, close
#include <poll.h> //poll
#include <uhd/utils/log.hpp>
#include <uhd/utils/msg.hpp>
#include <boost/format.hpp>
#include <boost/thread/thread.hpp> //sleep
#include <uhd/types/time_spec.hpp> //timeout
#include <uhd/utils/log.hpp>
#include <uhd/utils/atomic.hpp>

//locking stuff for shared irq
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

struct e300_fifo_poll_waiter
{
    e300_fifo_poll_waiter(const int fd):
        fd(fd)
    {
        //NOP
    }

    void wait(const double timeout)
    {
        if (_poll_claimed.cas(1, 0))
        {
            boost::mutex::scoped_lock l(mutex);
            cond.wait(l);
        }
        else
        {
            struct pollfd fds[1];
            fds[0].fd = fd;
            fds[0].events = POLLIN;
            ::poll(fds, 1, long(timeout*1000));
            if (fds[0].revents & POLLIN)
                ::read(fd, NULL, 0);

            _poll_claimed.write(0);
            cond.notify_all();
        }
    }

    uhd::atomic_uint32_t _poll_claimed;
    boost::condition_variable cond;
    boost::mutex mutex;
    int fd;
};

static const size_t DEFAULT_FRAME_SIZE = 2048;
static const size_t DEFAULT_NUM_FRAMES = 32;

using namespace uhd;
using namespace uhd::transport;

struct __mem_addrz_t
{
    size_t which, phys, data, ctrl;
};

/***********************************************************************
 * peek n' poke mmapped space
 **********************************************************************/
UHD_INLINE void zf_poke32(const boost::uint32_t addr, const boost::uint32_t data)
{
    volatile boost::uint32_t *p = reinterpret_cast<boost::uint32_t *>(addr);
    *p = data;
}

UHD_INLINE boost::uint32_t zf_peek32(const boost::uint32_t addr)
{
    volatile const boost::uint32_t *p = reinterpret_cast<const boost::uint32_t *>(addr);
    return *p;
}

/***********************************************************************
 * managed buffer
 **********************************************************************/
struct e300_fifo_mb : managed_buffer
{
    e300_fifo_mb(const __mem_addrz_t &addrs, const size_t len):
        ctrl_base(addrs.ctrl), phys_mem(addrs.phys), mem((void *)addrs.data), len(len){}

    void release(void)
    {
        UHD_ASSERT_THROW(zf_peek32(ctrl_base+ARBITER_RB_ADDR_SPACE) > 0);
        UHD_ASSERT_THROW(zf_peek32(ctrl_base+ARBITER_RB_SIZE_SPACE) > 0);
        zf_poke32(ctrl_base + ARBITER_WR_ADDR, phys_mem);
        zf_poke32(ctrl_base + ARBITER_WR_SIZE, this->size());
    }

    template <typename T>
    UHD_INLINE typename T::sptr get_new(void)
    {
        return make(reinterpret_cast<T *>(this), mem, len);
    }

    const size_t ctrl_base;
    const size_t phys_mem;
    void *const mem;
    const size_t len;
};

/***********************************************************************
 * transport
 **********************************************************************/
template <typename BaseClass>
class e300_transport : public zero_copy_if
{
public:
    e300_transport(
        boost::shared_ptr<void> allocator,
        const __mem_addrz_t &addrs,
        const size_t num_frames,
        const size_t frame_size,
        e300_fifo_poll_waiter *waiter,
        const bool auto_release
    ):
        _allocator(allocator),
        _addrs(addrs),
        _num_frames(num_frames),
        _frame_size(frame_size),
        _index(0),
        _waiter(waiter)
    {
        //UHD_MSG(status) << boost::format("phys 0x%x") % addrs.phys << std::endl;
        //UHD_MSG(status) << boost::format("data 0x%x") % addrs.data << std::endl;
        //UHD_MSG(status) << boost::format("ctrl 0x%x") % addrs.ctrl << std::endl;

        const boost::uint32_t sig = zf_peek32(_addrs.ctrl + ARBITER_RD_SIG);
        UHD_ASSERT_THROW((sig >> 16) == 0xACE0);

        zf_poke32(_addrs.ctrl + ARBITER_WR_CLEAR, 1);
        for (size_t i = 0; i < num_frames; i++)
        {
            //create a managed buffer at the given offset
            __mem_addrz_t mb_addrs = addrs;
            mb_addrs.phys += (i*frame_size);
            mb_addrs.data += (i*frame_size);
            boost::shared_ptr<e300_fifo_mb> mb(new e300_fifo_mb(mb_addrs, frame_size));

            //setup the buffers so they are "positioned for use"
            const size_t sts_good = (1 << 7) | (_addrs.which & 0xf);
            if (auto_release) mb->get_new<managed_recv_buffer>(); //release for read
            else zf_poke32(_addrs.ctrl + ARBITER_WR_STS, sts_good); //poke an ok into the sts fifo

            _buffs.push_back(mb);
        }
    }

    ~e300_transport(void)
    {
        //NOP
    }

    template <typename T>
    UHD_INLINE typename T::sptr get_buff(const double timeout)
    {
        const time_spec_t exit_time = time_spec_t::get_system_time() + time_spec_t(timeout);
        do
        {
            if (zf_peek32(_addrs.ctrl + ARBITER_RB_STATUS_OCC))
            {
                const boost::uint32_t sts = zf_peek32(_addrs.ctrl + ARBITER_RB_STATUS);
                UHD_ASSERT_THROW((sts >> 7) & 0x1); //assert OK
                UHD_ASSERT_THROW((sts & 0xf) == _addrs.which); //expected tag
                zf_poke32(_addrs.ctrl + ARBITER_WR_STS_RDY, 1); //pop from sts fifo
                if (_index == _num_frames)
                    _index = 0;
                return _buffs[_index++]->get_new<T>();
            }
            _waiter->wait(timeout);
            //boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
        while (time_spec_t::get_system_time() < exit_time);

        return typename T::sptr();
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

private:
    boost::shared_ptr<void> _allocator;
    const __mem_addrz_t _addrs;
    const size_t _num_frames;
    const size_t _frame_size;
    size_t _index;
    e300_fifo_poll_waiter *_waiter;
    std::vector<boost::shared_ptr<e300_fifo_mb> > _buffs;
};

/***********************************************************************
 * memory mapping
 **********************************************************************/
class e300_fifo_interface_impl : public virtual e300_fifo_interface
{
public:
    e300_fifo_interface_impl(const e300_fifo_config_t &config):
        _config(config),
        _bytes_in_use(0),
        _recv_entries_in_use(0),
        _send_entries_in_use(0)
    {
        //open the file descriptor to our kernel module
        const std::string dev = "/dev/axi_fpga";
        _fd = ::open(dev.c_str(), O_RDWR|O_SYNC);
        if (_fd < 0)
        {
            throw uhd::runtime_error("e300: failed to open " + dev);
        }

        //mmap the control and data regions into virtual space
        //UHD_VAR(_config.ctrl_length);
        //UHD_VAR(_config.buff_length);
        //UHD_VAR(_config.phys_addr);
        _buff = ::mmap(NULL, _config.ctrl_length + _config.buff_length, PROT_READ|PROT_WRITE, MAP_SHARED, _fd, 0);
        if (_buff == MAP_FAILED)
        {
            ::close(_fd);
            throw uhd::runtime_error("e300: failed to mmap " + dev);
        }

        //segment the memory according to zynq fifo arbiter
        _ctrl_space = size_t(_buff);
        _data_space = size_t(_buff) + _config.ctrl_length;

        //zero out the data region
        std::memset((void *)_data_space, 0, _config.buff_length);

        //create a poll _waiter for the transports
        _waiter = new e300_fifo_poll_waiter(_fd);
    }

    virtual ~e300_fifo_interface_impl(void)
    {
        delete _waiter;
        UHD_LOG << "cleanup: munmap" << std::endl;
        ::munmap(_buff, _config.ctrl_length + _config.buff_length);
        ::close(_fd);
    }

    uhd::transport::zero_copy_if::sptr make_recv_xport(const size_t which_stream, const uhd::device_addr_t &args)
    {
        return this->_make_xport(which_stream, args, true);
    }

    uhd::transport::zero_copy_if::sptr make_send_xport(const size_t which_stream, const uhd::device_addr_t &args)
    {
        return this->_make_xport(which_stream, args, false);
    }

private:
    uhd::transport::zero_copy_if::sptr _make_xport(const size_t which_stream, const uhd::device_addr_t &args, const bool is_recv)
    {
        boost::mutex::scoped_lock lock(_setup_mutex);

        const size_t frame_size(size_t(args.cast<double>((is_recv)? "recv_frame_size" : "send_frame_size", DEFAULT_FRAME_SIZE)));
        const size_t num_frames(size_t(args.cast<double>((is_recv)? "num_recv_frames" : "num_send_frames", DEFAULT_NUM_FRAMES)));
        size_t &entries_in_use = (is_recv)? _recv_entries_in_use : _send_entries_in_use;

        __mem_addrz_t addrs;
        addrs.which = which_stream;
        addrs.phys = _config.phys_addr + _bytes_in_use;
        addrs.data = _data_space + _bytes_in_use;
        addrs.ctrl = ((is_recv)? S2H_BASE(_ctrl_space) : H2S_BASE(_ctrl_space)) + ZF_STREAM_OFF(which_stream);

        uhd::transport::zero_copy_if::sptr xport;
        if (is_recv) xport.reset(new e300_transport<managed_recv_buffer>(shared_from_this(), addrs, num_frames, frame_size, _waiter, is_recv));
        else         xport.reset(new e300_transport<managed_send_buffer>(shared_from_this(), addrs, num_frames, frame_size, _waiter, is_recv));

        _bytes_in_use += num_frames*frame_size;
        entries_in_use += num_frames;

        UHD_ASSERT_THROW(_recv_entries_in_use <= S2H_NUM_CMDS);
        UHD_ASSERT_THROW(_send_entries_in_use <= H2S_NUM_CMDS);
        UHD_ASSERT_THROW(_bytes_in_use <= _config.buff_length);

        //program the dest table based on the stream
        //TODO make this part of SID allocation
        if (is_recv)
        {
            zf_poke32(DST_BASE(_ctrl_space) + which_stream*4, which_stream);
        }

        return xport;
    }

    size_t get_global_regs_base() const
    {
        return REG_BASE(_ctrl_space);
    }

    e300_fifo_config_t _config;
    e300_fifo_poll_waiter *_waiter;
    size_t _bytes_in_use;
    int _fd;
    void *_buff;
    size_t _ctrl_space;
    size_t _data_space;
    size_t _recv_entries_in_use;
    size_t _send_entries_in_use;
    boost::mutex _setup_mutex;
};

e300_fifo_interface::sptr e300_fifo_interface::make(const e300_fifo_config_t &config)
{
    return e300_fifo_interface::sptr(new e300_fifo_interface_impl(config));
}

#else //E300_NATIVE

#include "e300_fifo_config.hpp"
#include <uhd/exception.hpp>

e300_fifo_interface::sptr e300_fifo_interface::make(const e300_fifo_config_t &)
{
    throw uhd::runtime_error("e300_fifo_interface::make() !E300_NATIVE");
}

#endif //E300_NATIVE