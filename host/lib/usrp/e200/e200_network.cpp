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

#include "e200_impl.hpp"
#include <uhd/utils/msg.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

using namespace uhd;
using namespace uhd::transport;
namespace asio = boost::asio;

static inline bool wait_for_recv_ready(int sock_fd, const size_t timeout_ms)
{
    //setup timeval for timeout
    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = timeout_ms*1000;

    //setup rset for timeout
    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(sock_fd, &rset);

    //call select with timeout on receive socket
    return ::select(sock_fd+1, &rset, NULL, NULL, &tv) > 0;
}

/***********************************************************************
 * Receive tunnel - forwards recv interface to send socket
 **********************************************************************/
static void e200_recv_tunnel(
    const std::string &name,
    uhd::transport::zero_copy_if::sptr recver,
    boost::shared_ptr<asio::ip::tcp::socket> sender
)
{
    try
    {
        while (not boost::this_thread::interruption_requested())
        {
            //step 1 - get the buffer
            managed_recv_buffer::sptr buff = recver->get_recv_buff();
            if (not buff) continue;

            //step 2 - send to the socket
            sender->send(asio::buffer(buff->cast<const void *>(), buff->size()));
        }
    }
    catch(...)
    {
        UHD_MSG(error) << "e200_recv_tunnel exit " << name << std::endl;
    }
}

/***********************************************************************
 * Send tunnel - forwards recv socket to send interface
 **********************************************************************/
static void e200_send_tunnel(
    const std::string &name,
    boost::shared_ptr<asio::ip::tcp::socket> recver,
    uhd::transport::zero_copy_if::sptr sender
)
{
    try
    {
        while (not boost::this_thread::interruption_requested())
        {
            //step 1 - get the buffer
            managed_send_buffer::sptr buff = sender->get_send_buff();
            if (not buff) continue;

            //step 2 - recv from socket
            while (not wait_for_recv_ready(recver->native(), 100))
            {
                if (boost::this_thread::interruption_requested()) return;
            }
            const size_t num_bytes = recver->receive(asio::buffer(buff->cast<void *>(), buff->size()));

            //step 3 - commit the buffer
            buff->commit(num_bytes);
        }
    }
    catch(...)
    {
        UHD_MSG(error) << "e200_send_tunnel exit " << name << std::endl;
    }
}

/***********************************************************************
 * codec gateway
 **********************************************************************/
static void codec_gateway(ad9361_ctrl_iface_sptr ctrl, boost::shared_ptr<asio::ip::tcp::socket> sock)
{
    unsigned char in_buff[64];
    unsigned char out_buff[64];
    try
    {
        while (not boost::this_thread::interruption_requested())
        {
            sock->receive(asio::buffer(in_buff));
            ctrl->transact(in_buff, out_buff);
            sock->send(asio::buffer(out_buff));
        }
    }
    catch(...)
    {
        UHD_MSG(error) << "codec_gateway exit" << std::endl;
    }
}

/***********************************************************************
 * The TCP server itself
 **********************************************************************/
void e200_impl::run_server(const std::string &port, const std::string &what)
{
    UHD_MSG(status) << "e200 run server on port " << port << " for " << what << std::endl;
    while (true)
    {
        try
        {
            asio::io_service io_service;
            asio::ip::tcp::resolver resolver(io_service);
            asio::ip::tcp::resolver::query query(asio::ip::tcp::v4(), "0.0.0.0", port);
            asio::ip::tcp::endpoint endpoint = *resolver.resolve(query);

            boost::shared_ptr<asio::ip::tcp::acceptor> acceptor(new asio::ip::tcp::acceptor(io_service, endpoint));
            {
                boost::shared_ptr<asio::ip::tcp::socket> socket(new asio::ip::tcp::socket(io_service));
                while (not wait_for_recv_ready(acceptor->native(), 100))
                {
                    if (boost::this_thread::interruption_requested()) return;
                }
                acceptor->accept(*socket);
                boost::thread_group tg;
                if (what == "RX")
                {
                    tg.create_thread(boost::bind(&e200_recv_tunnel, "RX data tunnel", _rx_data_xport, socket));
                    tg.create_thread(boost::bind(&e200_send_tunnel, "RX flow tunnel", socket, _rx_flow_xport));
                }
                if (what == "TX")
                {
                    tg.create_thread(boost::bind(&e200_recv_tunnel, "TX flow tunnel", _tx_flow_xport, socket));
                    tg.create_thread(boost::bind(&e200_send_tunnel, "TX data tunnel", socket, _tx_data_xport));
                }
                if (what == "CTRL")
                {
                    tg.create_thread(boost::bind(&e200_recv_tunnel, "response tunnel", _recv_ctrl_xport, socket));
                    tg.create_thread(boost::bind(&e200_send_tunnel, "control tunnel", socket, _send_ctrl_xport));
                }
                if (what == "CODEC")
                {
                    codec_gateway(_codec_ctrl_iface, socket);
                }
                tg.join_all();
            }
        }
        catch(...){}
    }
}
