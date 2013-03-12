//
// Copyright 2012-2013 Ettus Research LLC
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

#include "b200_ctrl.hpp"
#include "b200_impl.hpp"
#include "b200_regs.hpp"
#include "async_packet_handler.hpp"
#include <uhd/exception.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/safe_call.hpp>
#include <uhd/transport/vrt_if_packet.hpp>
#include <uhd/transport/bounded_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <queue>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;

static const double ACK_TIMEOUT = 0.5;
static const double MASSIVE_TIMEOUT = 10.0; //for when we wait on a timed command
static const size_t RESP_QUEUE_SIZE = 2;

class b200_ctrl_impl : public b200_ctrl
{
public:

    b200_ctrl_impl(zero_copy_if::sptr xport):
        _xport(xport),
        _seq_out(0),
        _timeout(ACK_TIMEOUT),
        _resp_queue(RESP_QUEUE_SIZE)
    {
        while (_xport->get_recv_buff(0.0)){} //flush
        this->set_time(uhd::time_spec_t(0.0));
        this->set_tick_rate(1.0); //something possible but bogus
    }

    ~b200_ctrl_impl(void)
    {
        _timeout = ACK_TIMEOUT; //reset timeout to something small
        UHD_SAFE_CALL(
            this->peek32(0); //dummy peek with the purpose of ack'ing all packets
        )
    }

    /*******************************************************************
     * Peek and poke 32 bit implementation
     ******************************************************************/
    void poke32(wb_addr_type addr, boost::uint32_t data)
    {
        boost::mutex::scoped_lock lock(_mutex);

        this->send_pkt(addr/4, data);
        this->wait_for_ack(false);
    }

    boost::uint32_t peek32(wb_addr_type addr)
    {
        boost::mutex::scoped_lock lock(_mutex);

        this->send_pkt(SR_READBACK, addr/8);
        this->wait_for_ack(false);

        this->send_pkt(0);
        const boost::uint64_t res = this->wait_for_ack(true);
        const boost::uint32_t lo = boost::uint32_t(res & 0xffffffff);
        const boost::uint32_t hi = boost::uint32_t(res >> 32);
        return ((addr/4) & 0x1)? hi : lo;
    }

    boost::uint64_t peek64(wb_addr_type addr)
    {
        boost::mutex::scoped_lock lock(_mutex);

        this->send_pkt(SR_READBACK, addr/8);
        this->wait_for_ack(false);

        this->send_pkt(0);
        return this->wait_for_ack(true);
    }

    /*******************************************************************
     * Update methods for time
     ******************************************************************/
    void set_time(const uhd::time_spec_t &time)
    {
        boost::mutex::scoped_lock lock(_mutex);
        _time = time;
        _use_time = _time != uhd::time_spec_t(0.0);
        if (_use_time) _timeout = MASSIVE_TIMEOUT; //permanently sets larger timeout
    }

    void set_tick_rate(const double rate)
    {
        boost::mutex::scoped_lock lock(_mutex);
        _tick_rate = rate;
    }

private:

    /*******************************************************************
     * Primary control and interaction private methods
     ******************************************************************/
    UHD_INLINE void send_pkt(const boost::uint32_t addr, const boost::uint32_t data = 0)
    {
        managed_send_buffer::sptr buff = _xport->get_send_buff(0.0);
        if (not buff){
            throw uhd::runtime_error("fifo ctrl timed out getting a send buffer");
        }
        boost::uint32_t *pkt = buff->cast<boost::uint32_t *>();

        //load packet info
        vrt::if_packet_info_t packet_info;
        packet_info.link_type = vrt::if_packet_info_t::LINK_TYPE_CHDR;
        packet_info.packet_type = vrt::if_packet_info_t::PACKET_TYPE_EXTENSION;
        packet_info.num_payload_words32 = 2;
        packet_info.num_payload_bytes = packet_info.num_payload_words32*sizeof(boost::uint32_t);
        packet_info.packet_count = _seq_out;
        packet_info.tsf = _time.to_ticks(_tick_rate);
        packet_info.sob = false;
        packet_info.eob = false;
        packet_info.sid = B200_CTRL_MSG_SID;
        packet_info.has_sid = true;
        packet_info.has_cid = false;
        packet_info.has_tsi = false;
        packet_info.has_tsf = _use_time;
        packet_info.has_tlr = false;

        //load header
        vrt::if_hdr_pack_le(pkt, packet_info);

        //load payload
        pkt[packet_info.num_header_words32+0] = uhd::htowx(addr);
        pkt[packet_info.num_header_words32+1] = uhd::htowx(data);

        //send the buffer over the interface
        _outstanding_seqs.push(_seq_out);
        buff->commit(sizeof(boost::uint32_t)*(packet_info.num_packet_words32));

        _seq_out++; //inc seq for next call
    }

    UHD_INLINE boost::uint64_t wait_for_ack(const bool readback)
    {
        while (readback or (_outstanding_seqs.size() >= RESP_QUEUE_SIZE))
        {
            //get seq to ack from outstanding packets list
            UHD_ASSERT_THROW(not _outstanding_seqs.empty());
            const size_t seq_to_ack = _outstanding_seqs.front();
            _outstanding_seqs.pop();

            //get buffer from response endpoing - or die in timeout
            resp_buff_type resp_buff;
            UHD_ASSERT_THROW(_resp_queue.pop_with_timed_wait(resp_buff, _timeout));

            //parse the buffer
            const boost::uint32_t *pkt = resp_buff.data;
            vrt::if_packet_info_t packet_info;
            packet_info.link_type = vrt::if_packet_info_t::LINK_TYPE_CHDR;
            packet_info.num_packet_words32 = sizeof(resp_buff)/sizeof(boost::uint32_t);
            try
            {
                vrt::if_hdr_unpack_le(pkt, packet_info);
            }
            catch(const std::exception &ex)
            {
                UHD_MSG(error) << "B200 ctrl bad VITA packet: " << ex.what() << std::endl;
                UHD_MSG(status) << std::hex << pkt[0] << std::dec << std::endl;
                UHD_MSG(status) << std::hex << pkt[1] << std::dec << std::endl;
                UHD_MSG(status) << std::hex << pkt[2] << std::dec << std::endl;
                UHD_MSG(status) << std::hex << pkt[3] << std::dec << std::endl;
            }

            //check the buffer
            UHD_ASSERT_THROW(packet_info.has_sid);
            UHD_ASSERT_THROW(packet_info.sid == B200_RESP_MSG_SID);
            UHD_ASSERT_THROW(packet_info.packet_count == (seq_to_ack & 0xfff));
            UHD_ASSERT_THROW(packet_info.num_payload_words32 == 2);
            UHD_ASSERT_THROW(packet_info.packet_type == vrt::if_packet_info_t::PACKET_TYPE_EXTENSION);

            //return the readback value
            if (readback and _outstanding_seqs.empty())
            {
                const boost::uint64_t hi = uhd::wtohx(pkt[packet_info.num_header_words32+0]);
                const boost::uint64_t lo = uhd::wtohx(pkt[packet_info.num_header_words32+1]);
                return ((hi << 32) | lo);
            }
        }
        return 0;
    }

    void push_resp(const boost::uint32_t *buff)
    {
        resp_buff_type resp_buff;
        std::memcpy(resp_buff.data, buff, sizeof(resp_buff));
        _resp_queue.push_with_haste(resp_buff);
    }

    zero_copy_if::sptr _xport;
    boost::mutex _mutex;
    size_t _seq_out;
    uhd::time_spec_t _time;
    bool _use_time;
    double _tick_rate;
    double _timeout;
    std::queue<size_t> _outstanding_seqs;
    struct resp_buff_type
    {
        boost::uint32_t data[8];
    };
    bounded_buffer<resp_buff_type> _resp_queue;
};


b200_ctrl::sptr b200_ctrl::make(zero_copy_if::sptr xport)
{
    return sptr(new b200_ctrl_impl(xport));
}
