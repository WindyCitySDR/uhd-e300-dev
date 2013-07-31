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
#include <uhd/utils/msg.hpp>
#include <uhd/exception.hpp>
#include <boost/format.hpp>
#include <boost/thread/mutex.hpp>

struct b250_ctrl_iface : wb_iface
{
    enum {num_retries = 3};

    b250_ctrl_iface(uhd::transport::udp_simple::sptr udp):
        udp(udp), seq(0)
    {
        try
        {
            this->peek32(0);
        }
        catch(...){}
    }

    uhd::transport::udp_simple::sptr udp;
    size_t seq;
    boost::mutex mutex;

    void flush(void)
    {
        char buff[X300_FW_COMMS_MTU] = {};
        while (udp->recv(boost::asio::buffer(buff), 0.0)){} //flush
    }

    void poke32(const wb_addr_type addr, const boost::uint32_t data)
    {
        for (size_t i = 1; i <= num_retries; i++)
        {
            try
            {
                return this->__poke32(addr, data);
            }
            catch(const std::exception &ex)
            {
                const std::string error_msg = str(boost::format(
                    "b250 fw communication failure #%u\n%s") % i % ex.what());
                UHD_MSG(error) << error_msg << std::endl;
                if (i == num_retries) throw uhd::io_error(error_msg);
            }
        }
    }

    boost::uint32_t peek32(const wb_addr_type addr)
    {
        for (size_t i = 1; i <= num_retries; i++)
        {
            try
            {
                return this->__peek32(addr);
            }
            catch(const std::exception &ex)
            {
                const std::string error_msg = str(boost::format(
                    "b250 fw communication failure #%u\n%s") % i % ex.what());
                UHD_MSG(error) << error_msg << std::endl;
                if (i == num_retries) throw uhd::io_error(error_msg);
            }
        }
        return 0;
    }

    void __poke32(const wb_addr_type addr, const boost::uint32_t data)
    {
        boost::mutex::scoped_lock lock(mutex);

        //load request struct
        x300_fw_comms_t request = x300_fw_comms_t();
        request.flags = uhd::htonx<boost::uint32_t>(X300_FW_COMMS_FLAGS_ACK | X300_FW_COMMS_FLAGS_POKE32);
        request.sequence = uhd::htonx<boost::uint32_t>(seq++);
        request.addr = uhd::htonx(addr);
        request.data = uhd::htonx(data);

        //send request
        this->flush();
        udp->send(boost::asio::buffer(&request, sizeof(request)));

        //recv reply
        x300_fw_comms_t reply = x300_fw_comms_t();
        const size_t nbytes = udp->recv(boost::asio::buffer(&reply, sizeof(reply)), 1.0);
        if (nbytes == 0) throw uhd::io_error("b250 fw poke32 - reply timed out");

        //sanity checks
        const size_t flags = uhd::ntohx<boost::uint32_t>(reply.flags);
        UHD_ASSERT_THROW(nbytes == sizeof(reply));
        UHD_ASSERT_THROW(not (flags & X300_FW_COMMS_FLAGS_ERROR));
        UHD_ASSERT_THROW(flags & X300_FW_COMMS_FLAGS_POKE32);
        UHD_ASSERT_THROW(flags & X300_FW_COMMS_FLAGS_ACK);
        UHD_ASSERT_THROW(reply.sequence == request.sequence);
        UHD_ASSERT_THROW(reply.addr == request.addr);
        UHD_ASSERT_THROW(reply.data == request.data);
    }

    boost::uint32_t __peek32(const wb_addr_type addr)
    {
        boost::mutex::scoped_lock lock(mutex);

        //load request struct
        x300_fw_comms_t request = x300_fw_comms_t();
        request.flags = uhd::htonx<boost::uint32_t>(X300_FW_COMMS_FLAGS_ACK | X300_FW_COMMS_FLAGS_PEEK32);
        request.sequence = uhd::htonx<boost::uint32_t>(seq++);
        request.addr = uhd::htonx(addr);
        request.data = 0;

        //send request
        this->flush();
        udp->send(boost::asio::buffer(&request, sizeof(request)));

        //recv reply
        x300_fw_comms_t reply = x300_fw_comms_t();
        const size_t nbytes = udp->recv(boost::asio::buffer(&reply, sizeof(reply)), 1.0);
        if (nbytes == 0) throw uhd::io_error("b250 fw peek32 - reply timed out");

        //sanity checks
        const size_t flags = uhd::ntohx<boost::uint32_t>(reply.flags);
        UHD_ASSERT_THROW(nbytes == sizeof(reply));
        UHD_ASSERT_THROW(not (flags & X300_FW_COMMS_FLAGS_ERROR));
        UHD_ASSERT_THROW(flags & X300_FW_COMMS_FLAGS_PEEK32);
        UHD_ASSERT_THROW(flags & X300_FW_COMMS_FLAGS_ACK);
        UHD_ASSERT_THROW(reply.sequence == request.sequence);
        UHD_ASSERT_THROW(reply.addr == request.addr);

        //return result!
        return uhd::ntohx<boost::uint32_t>(reply.data);
    }
};

#endif /* INCLUDED_B250_FW_CTRL_HPP */
