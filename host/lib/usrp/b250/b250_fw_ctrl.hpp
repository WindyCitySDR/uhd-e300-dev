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

#ifndef INCLUDED_B250_FW_CTRL_HPP
#define INCLUDED_B250_FW_CTRL_HPP

#include "wb_iface.hpp"
#include <uhd/transport/udp_simple.hpp>
#include <uhd/utils/byteswap.hpp>

struct b250_ctrl_iface : wb_iface
{
    b250_ctrl_iface(uhd::transport::udp_simple::sptr udp):
        udp(udp), seq(0){this->flush();}

    uhd::transport::udp_simple::sptr udp;
    size_t seq;

    void flush(void)
    {
        char buff[B250_FW_COMMS_MTU] = {};
        while (udp->recv(boost::asio::buffer(buff), 0.0)){} //flush
    }

    void poke32(wb_addr_type addr, boost::uint32_t data)
    {
        //load request struct
        b250_fw_comms_t request = b250_fw_comms_t();
        request.flags = uhd::htonx<boost::uint32_t>(B250_FW_COMMS_FLAGS_ACK | B250_FW_COMMS_FLAGS_POKE32);
        request.sequence = uhd::htonx<boost::uint32_t>(seq++);
        request.addr = uhd::htonx(addr);
        request.data = uhd::htonx(data);

        //send request
        udp->send(boost::asio::buffer(&request, sizeof(request)));

        //recv reply
        this->flush();
        b250_fw_comms_t reply = b250_fw_comms_t();
        const size_t nbytes = udp->recv(boost::asio::buffer(&reply, sizeof(reply)));

        //sanity checks
        UHD_ASSERT_THROW(nbytes == sizeof(reply));
        UHD_ASSERT_THROW(not (uhd::ntohx(reply.flags) & B250_FW_COMMS_FLAGS_ERROR));
        UHD_ASSERT_THROW(uhd::ntohx(reply.flags) & B250_FW_COMMS_FLAGS_POKE32);
        UHD_ASSERT_THROW(uhd::ntohx(reply.flags) & B250_FW_COMMS_FLAGS_ACK);
        UHD_ASSERT_THROW(reply.sequence == request.sequence);
        UHD_ASSERT_THROW(reply.addr == request.addr);
        UHD_ASSERT_THROW(reply.data == request.data);
    }

    boost::uint32_t peek32(wb_addr_type addr)
    {
        //load request struct
        b250_fw_comms_t request = b250_fw_comms_t();
        request.flags = uhd::htonx<boost::uint32_t>(B250_FW_COMMS_FLAGS_ACK | B250_FW_COMMS_FLAGS_PEEK32);
        request.sequence = uhd::htonx<boost::uint32_t>(seq++);
        request.addr = uhd::htonx(addr);
        request.data = 0;

        //send request
        udp->send(boost::asio::buffer(&request, sizeof(request)));

        //recv reply
        this->flush();
        b250_fw_comms_t reply = b250_fw_comms_t();
        const size_t nbytes = udp->recv(boost::asio::buffer(&reply, sizeof(reply)));

        //sanity checks
        UHD_ASSERT_THROW(nbytes == sizeof(reply));
        UHD_ASSERT_THROW(not (uhd::ntohx(reply.flags) & B250_FW_COMMS_FLAGS_ERROR));
        UHD_ASSERT_THROW(uhd::ntohx(reply.flags) & B250_FW_COMMS_FLAGS_PEEK32);
        UHD_ASSERT_THROW(uhd::ntohx(reply.flags) & B250_FW_COMMS_FLAGS_ACK);
        UHD_ASSERT_THROW(reply.sequence == request.sequence);
        UHD_ASSERT_THROW(reply.addr == request.addr);

        //return result!
        return uhd::ntohx(reply.data);
    }

    void poke16(wb_addr_type, boost::uint16_t)
    {
        throw uhd::not_implemented_error("b250_ctrl_iface::poke16");
    }

    boost::uint16_t peek16(wb_addr_type)
    {
        throw uhd::not_implemented_error("b250_ctrl_iface::peek16");
    }
};

#endif /* INCLUDED_B250_FW_CTRL_HPP */
