//
// Copyright 2010-2013 Ettus Research LLC
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

#include <uhd/transport/nirio_zero_copy.hpp>
#include <stdio.h>
#include <uhd/transport/nirio/nirio_fifo.h>
#include <uhd/transport/nirio/nirio_fifo.h>
#include <uhd/transport/buffer_pool.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/utils/atomic.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp> //sleep
#include <vector>
//@TODO: Move the register defs required by the class to a common location
#include "../usrp/x300/x300_regs.hpp"

using namespace uhd;
using namespace uhd::transport;
using namespace uhd::niusrprio;

//A reasonable number of frames for send/recv and async/sync
static const size_t DEFAULT_NUM_FRAMES  = 32;
static const size_t DEFAULT_FRAMES_SIZE = 8192;

typedef uint64_t fifo_data_t;

class nirio_zero_copy_mrb : public managed_recv_buffer
{
public:
    nirio_zero_copy_mrb(nirio_fifo<fifo_data_t>& fifo, const size_t frame_size):
        _fifo(fifo), _frame_size(frame_size) { }

    void release(void)
    {
        _fifo.release(_frame_size / sizeof(fifo_data_t));
    }

    UHD_INLINE sptr get_new(const double timeout, size_t &index)
    {
        nirio_status status = 0;
        size_t elems_acquired, elems_remaining;
        nirio_status_chain(_fifo.acquire(
            _typed_buffer, _frame_size / sizeof(fifo_data_t),
            static_cast<uint32_t>(timeout*1000),
            elems_acquired, elems_remaining), status);
        _length = elems_acquired * sizeof(fifo_data_t);
        _buffer = static_cast<void*>(_typed_buffer);

        if (nirio_status_not_fatal(status)) {
            index++;        //Advances the caller's buffer
            return make(this, _buffer, _length);
        } else {
            return sptr();  //NULL for timeout or error.
        }
    }

private:
    nirio_fifo<fifo_data_t>&    _fifo;
    fifo_data_t*                _typed_buffer;
    const size_t                _frame_size;
    size_t                      _num_frames;
};

class nirio_zero_copy_msb : public managed_send_buffer
{
public:
    nirio_zero_copy_msb(nirio_fifo<fifo_data_t>& fifo, const size_t frame_size):
        _fifo(fifo), _frame_size(frame_size) { }

    void release(void)
    {
        _fifo.release(_frame_size / sizeof(fifo_data_t));
    }

    UHD_INLINE sptr get_new(const double timeout, size_t &index)
    {
        nirio_status status = 0;
        size_t elems_acquired, elems_remaining;
        nirio_status_chain(_fifo.acquire(
            _typed_buffer, _frame_size / sizeof(fifo_data_t),
            static_cast<uint32_t>(timeout*1000),
            elems_acquired, elems_remaining), status);
        _length = elems_acquired * sizeof(fifo_data_t);
        _buffer = static_cast<void*>(_typed_buffer);

        if (nirio_status_not_fatal(status)) {
            index++;        //Advances the caller's buffer
            return make(this, _buffer, _length);
        } else {
            return sptr();  //NULL for timeout or error.
        }
    }

private:
    nirio_fifo<fifo_data_t>&    _fifo;
    fifo_data_t*                _typed_buffer;
    const size_t                _frame_size;
    size_t                      _num_frames;
};

class nirio_zero_copy_impl : public nirio_zero_copy {
public:
    typedef boost::shared_ptr<nirio_zero_copy_impl> sptr;

    nirio_zero_copy_impl(
        uhd::niusrprio::niusrprio_session::sptr fpga_session,
        uint32_t instance,
        const device_addr_t &hints
    ):
        _reg_int(fpga_session->get_kernel_proxy()),
        _fifo_instance(instance),
        _recv_frame_size(size_t(hints.cast<double>("recv_frame_size", DEFAULT_FRAMES_SIZE))),
        _num_recv_frames(size_t(hints.cast<double>("num_recv_frames", DEFAULT_NUM_FRAMES))),
        _send_frame_size(size_t(hints.cast<double>("send_frame_size", DEFAULT_FRAMES_SIZE))),
        _num_send_frames(size_t(hints.cast<double>("num_send_frames", DEFAULT_NUM_FRAMES))),
        _recv_buffer_pool(buffer_pool::make(_num_recv_frames, _recv_frame_size)),
        _send_buffer_pool(buffer_pool::make(_num_send_frames, _send_frame_size)),
        _next_recv_buff_index(0), _next_send_buff_index(0)
    {
        UHD_LOG << boost::format("Creating PCIe transport for instance %d") % instance << std::endl;

        nirio_status status = 0;
        size_t actual_depth = 0, actual_size = 0;

        //Configure frame width
        nirio_status_chain(
            _reg_int.poke(PCIE_TX_DMA_REG(DMA_FRAME_SIZE_REG, _fifo_instance),
                          static_cast<uint32_t>(_send_frame_size/sizeof(fifo_data_t))),
            status);
        nirio_status_chain(
            _reg_int.poke(PCIE_RX_DMA_REG(DMA_FRAME_SIZE_REG, _fifo_instance),
                          static_cast<uint32_t>(_recv_frame_size/sizeof(fifo_data_t))),
            status);
        //Config 32-bit word flipping and Reset DMA streams
        nirio_status_chain(
            _reg_int.poke(PCIE_TX_DMA_REG(DMA_CTRL_STATUS_REG, _fifo_instance),
                          DMA_CTRL_SW_BUF_U32 | DMA_CTRL_RESET),
            status);
        nirio_status_chain(
            _reg_int.poke(PCIE_RX_DMA_REG(DMA_CTRL_STATUS_REG, _fifo_instance),
                          DMA_CTRL_SW_BUF_U32 | DMA_CTRL_RESET),
            status);

        //Create FIFOs
        nirio_status_chain(
            fpga_session->create_rx_fifo(_fifo_instance, _recv_fifo),
            status);
        nirio_status_chain(
            fpga_session->create_tx_fifo(_fifo_instance, _send_fifo),
            status);

        if ((_recv_fifo.get() != NULL) && (_send_fifo.get() != NULL)) {
            //Initialize FIFOs
            nirio_status_chain(
                _recv_fifo->initialize((_recv_frame_size*_num_recv_frames)/sizeof(fifo_data_t), actual_depth, actual_size),
                status);
            nirio_status_chain(
                _send_fifo->initialize((_send_frame_size*_num_send_frames)/sizeof(fifo_data_t), actual_depth, actual_size),
                status);

            nirio_status_chain(_recv_fifo->start(), status);
            nirio_status_chain(_send_fifo->start(), status);

            //allocate re-usable managed receive buffers
            for (size_t i = 0; i < get_num_recv_frames(); i++){
                _mrb_pool.push_back(boost::shared_ptr<nirio_zero_copy_mrb>(new nirio_zero_copy_mrb(
                    *_recv_fifo, get_recv_frame_size())));
            }

            //allocate re-usable managed send buffers
            for (size_t i = 0; i < get_num_send_frames(); i++){
                _msb_pool.push_back(boost::shared_ptr<nirio_zero_copy_msb>(new nirio_zero_copy_msb(
                    *_send_fifo, get_send_frame_size())));
            }
        } else {
            nirio_status_chain(NiRio_Status_ResourceNotInitialized, status);
        }

        nirio_status_to_exception(status, "Could not create nirio_zero_copy transport.");
    }

    ~nirio_zero_copy_impl() {
        //Reset DMA streams
        nirio_status status = 0;
        nirio_status_chain(
            _reg_int.poke(PCIE_TX_DMA_REG(DMA_CTRL_STATUS_REG, _fifo_instance), DMA_CTRL_RESET),
            status);
        nirio_status_chain(
            _reg_int.poke(PCIE_RX_DMA_REG(DMA_CTRL_STATUS_REG, _fifo_instance), DMA_CTRL_RESET),
            status);
    }

    /*******************************************************************
     * Receive implementation:
     * Block on the managed buffer's get call and advance the index.
     ******************************************************************/
    managed_recv_buffer::sptr get_recv_buff(double timeout){
        if (_next_recv_buff_index == _num_recv_frames) _next_recv_buff_index = 0;
        return _mrb_pool[_next_recv_buff_index]->get_new(timeout, _next_recv_buff_index);
    }

    size_t get_num_recv_frames(void) const {return _num_recv_frames;}
    size_t get_recv_frame_size(void) const {return _recv_frame_size;}

    /*******************************************************************
     * Send implementation:
     * Block on the managed buffer's get call and advance the index.
     ******************************************************************/
    managed_send_buffer::sptr get_send_buff(double timeout){
        if (_next_send_buff_index == _num_send_frames) _next_send_buff_index = 0;
        return _msb_pool[_next_send_buff_index]->get_new(timeout, _next_send_buff_index);
    }

    size_t get_num_send_frames(void) const {return _num_send_frames;}
    size_t get_send_frame_size(void) const {return _send_frame_size;}

private:
    //memory management -> buffers and fifos
    niriok_proxy& _reg_int;
    uint32_t _fifo_instance;
    nirio_fifo<fifo_data_t>::sptr _recv_fifo, _send_fifo;
    const size_t _recv_frame_size, _num_recv_frames;
    const size_t _send_frame_size, _num_send_frames;
    buffer_pool::sptr _recv_buffer_pool, _send_buffer_pool;
    std::vector<boost::shared_ptr<nirio_zero_copy_msb> > _msb_pool;
    std::vector<boost::shared_ptr<nirio_zero_copy_mrb> > _mrb_pool;
    size_t _next_recv_buff_index, _next_send_buff_index;
};


nirio_zero_copy::sptr nirio_zero_copy::make(
    uhd::niusrprio::niusrprio_session::sptr fpga_session,
    const uint32_t instance,
    const device_addr_t &hints
){
    return nirio_zero_copy::sptr(new nirio_zero_copy_impl(fpga_session, instance, hints));
}

size_t nirio_zero_copy::get_default_buffer_size(void) {
    return (DEFAULT_NUM_FRAMES * DEFAULT_FRAMES_SIZE);
}

