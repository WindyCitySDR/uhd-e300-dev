/*
 * b250_nirio_test.cpp
 *
 *  Created on: Jun 27, 2013
 *      Author: ashish
 */

#include <uhd/transport/nirio_zero_copy.hpp>
#include <uhd/transport/nirio/nifpga_interface.h>
#include <uhd/transport/nirio/nirio_interface.h>
#include <uhd/transport/nirio/nirio_fifo.h>
#include <uhd/transport/nirio/nifpga_image.h>

using namespace uhd;
using namespace uhd::transport;
using namespace nirio_interface;

#define TX_FIFO_TIMEOUT 1000

class vita_chdr_packet_t {
public:
    struct sid_t {
        sid_t(uint8_t ah, uint8_t dh, uint8_t at, uint8_t dt) :
            addr_here(ah), dest_here(dh), addr_there(at), dest_there(dt) {}

        uint8_t addr_here;
        uint8_t dest_here;
        uint8_t addr_there;
        uint8_t dest_there;
    };

    vita_chdr_packet_t():stream_id(0),sequence_num(0),flags(0),_length(0),_data_buf(NULL) {}

    void set_size(uint16_t size) {
        _data_buf.reset(new uint64_t[size + 1]);
        _length = size;
    }
    uint16_t get_size() {
        return static_cast<uint16_t>(_length);
    }
    void set_data(uint16_t index, uint64_t data) {
        _data_buf[index + 1] = data;
    }
    uint64_t get_data(uint16_t index) {
        return _data_buf[index + 1];
    }
    void set_sid(sid_t sid) {
        stream_id = 0
        | (static_cast<uint32_t>(sid.addr_here) << 24)
        | (static_cast<uint32_t>(sid.dest_here) << 16)
        | (static_cast<uint32_t>(sid.addr_there) << 8)
        | (static_cast<uint32_t>(sid.dest_there) << 0);
    }
    void set_sid(uint32_t sid_u32) {
        stream_id = sid_u32;
    }

    nirio_status send(nirio_fifo<uint64_t>& fifo) {
        uint64_t header = static_cast<uint64_t>(stream_id);
        header |= static_cast<uint64_t>((_length+1)*2 & 0xFFFF) << 32;
        header |= static_cast<uint64_t>(sequence_num & 0x0FFF) << 48;
        header |= static_cast<uint64_t>(flags & 0x0F) << 60;

        _data_buf[0] = header;

        uint32_t dummy;
        return fifo.write(_data_buf.get(), _length+1, 5000, dummy);
    }

    nirio_status recv(nirio_fifo<uint64_t>& fifo, uint16_t seqno = 0) {
        nirio_status status = 0;
        uint32_t dummy;

        uint64_t header = 0;
        nirio_status_chain(fifo.read(&header, 1, 5000, dummy, dummy), status);
        if (nirio_status_fatal(status)) return status;


        stream_id = static_cast<uint32_t>(header);
        _length = (static_cast<uint16_t>((header >> 32) & 0xFFFF) / 2) - 1;
        sequence_num = static_cast<uint16_t>((header >> 48) & 0x0FFF);
        flags = static_cast<uint8_t>((header >> 60) & 0x0F);

        if (seqno && sequence_num != seqno) {
            printf("ERROR: Sequence number check failed!\n");
            set_size(0);
            return -1;
        }

        set_size(_length);
        nirio_status_chain(fifo.read(&(_data_buf[1]), _length, 5000, dummy, dummy), status);

        return status;
    }

    void send(uhd::transport::zero_copy_if::sptr transport) {
        uint64_t header = static_cast<uint64_t>(stream_id);
        header |= static_cast<uint64_t>((_length+1)*2 & 0xFFFF) << 32;
        header |= static_cast<uint64_t>(sequence_num & 0x0FFF) << 48;
        header |= static_cast<uint64_t>(flags & 0x0F) << 60;

        managed_send_buffer::sptr buf = transport->get_send_buff();
        uint64_t* xport_buf = buf->cast<uint64_t*>();
        for (size_t i = 0; i < (size_t)_length + 1; i++)
            xport_buf[i] = _data_buf[i];

        buf->commit((_length + 1) * sizeof(uint64_t));
        buf->release();
    }

    uint32_t    stream_id;
    uint16_t    sequence_num;
    uint8_t     flags;
private:
    uint16_t                        _length;
    boost::scoped_array<uint64_t>   _data_buf;
};
