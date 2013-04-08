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

#include "e200_ctrl.hpp"
#include "e200_impl.hpp"
#include "e200_regs.hpp"
#include "async_packet_handler.hpp"
#include <uhd/exception.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/safe_call.hpp>
#include <uhd/transport/vrt_if_packet.hpp>
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
static const size_t RESP_QUEUE_SIZE = 13;

class e200_ctrl_impl : public e200_ctrl
{
public:

    e200_ctrl_impl(
        uhd::transport::zero_copy_if::sptr ctrl_xport,
        uhd::transport::zero_copy_if::sptr resp_xport,
        const boost::uint32_t sid
    ):
        _ctrl_xport(ctrl_xport),
        _resp_xport(resp_xport),
        _sid(sid),
        _seq_out(0),
        _timeout(ACK_TIMEOUT)
    {
        while (_resp_xport->get_recv_buff(0.0)){} //flush
        this->set_time(uhd::time_spec_t(0.0));
        this->set_tick_rate(1.0); //something possible but bogus
    }

    ~e200_ctrl_impl(void)
    {
        _timeout = ACK_TIMEOUT; //reset timeout to something small
        UHD_SAFE_CALL(
            this->peek32(0); //dummy peek with the purpose of ack'ing all packets
        )
    }

    /*******************************************************************
     * Peek and poke 32 bit implementation
     ******************************************************************/
    void poke32(const wb_addr_type addr, const boost::uint32_t data)
    {
        boost::mutex::scoped_lock lock(_mutex);

        this->send_pkt(addr/4, data);
        this->wait_for_ack(false);
    }

    boost::uint32_t peek32(const wb_addr_type addr)
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

    boost::uint64_t peek64(const wb_addr_type addr)
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
        managed_send_buffer::sptr buff = _ctrl_xport->get_send_buff(0.0);
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
        packet_info.sid = _sid;
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
            managed_recv_buffer::sptr buff = _resp_xport->get_recv_buff(_timeout);
            UHD_ASSERT_THROW(bool(buff));
            UHD_ASSERT_THROW(bool(buff->size()));

            //parse the buffer
            const boost::uint32_t *pkt = buff->cast<const boost::uint32_t *>();
            vrt::if_packet_info_t packet_info;
            packet_info.link_type = vrt::if_packet_info_t::LINK_TYPE_CHDR;
            packet_info.num_packet_words32 = buff->size()/sizeof(boost::uint32_t);
            try
            {
                vrt::if_hdr_unpack_le(pkt, packet_info);
            }
            catch(const std::exception &ex)
            {
                UHD_MSG(error) << "E200 ctrl bad VITA packet: " << ex.what() << std::endl;
                for (size_t i = 0; i < 5; i++)
                {
                    UHD_MSG(status) << boost::format("pkt[%u] = 0x%08x\n") % i % pkt[i];
                }
            }

            //check the buffer
            UHD_ASSERT_THROW(packet_info.has_sid);
            UHD_ASSERT_THROW(packet_info.sid == boost::uint32_t((_sid >> 16) | (_sid << 16)));
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

    uhd::transport::zero_copy_if::sptr _ctrl_xport;
    uhd::transport::zero_copy_if::sptr _resp_xport;
    const boost::uint32_t _sid;
    boost::mutex _mutex;
    size_t _seq_out;
    uhd::time_spec_t _time;
    bool _use_time;
    double _tick_rate;
    double _timeout;
    std::queue<size_t> _outstanding_seqs;
};


e200_ctrl::sptr e200_ctrl::make(
    uhd::transport::zero_copy_if::sptr ctrl_xport,
    uhd::transport::zero_copy_if::sptr resp_xport,
    const boost::uint32_t sid
)
{
    return sptr(new e200_ctrl_impl(ctrl_xport, resp_xport, sid));
}
