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

static const size_t E200_NETWORK_DEBUG = false;

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
    boost::shared_ptr<asio::ip::udp::socket> sender,
    asio::ip::udp::endpoint *endpoint,
    bool *running
)
{
    try
    {
        while (*running)
        {
            //step 1 - get the buffer
            managed_recv_buffer::sptr buff = recver->get_recv_buff();
            if (not buff) continue;
            if (E200_NETWORK_DEBUG) UHD_MSG(status) << name << " got " << buff->size() << std::endl;

            //step 2 - send to the socket
            sender->send_to(asio::buffer(buff->cast<const void *>(), buff->size()), *endpoint);
        }
    }
    catch(const std::exception &ex)
    {
        UHD_MSG(error) << "e200_recv_tunnel exit " << name << " " << ex.what() << std::endl;
    }
    catch(...)
    {
        UHD_MSG(error) << "e200_recv_tunnel exit " << name << std::endl;
    }
    UHD_MSG(status) << "e200_recv_tunnel exit " << name << std::endl;
    *running = false;
}

/***********************************************************************
 * Send tunnel - forwards recv socket to send interface
 **********************************************************************/
static void e200_send_tunnel(
    const std::string &name,
    boost::shared_ptr<asio::ip::udp::socket> recver,
    uhd::transport::zero_copy_if::sptr sender,
    asio::ip::udp::endpoint *endpoint,
    bool *running
)
{
    try
    {
        while (*running)
        {
            //step 1 - get the buffer
            managed_send_buffer::sptr buff = sender->get_send_buff();
            if (not buff) continue;

            //step 2 - recv from socket
            while (not wait_for_recv_ready(recver->native(), 100) and *running){}
            if (not *running) break;
            const size_t num_bytes = recver->receive_from(asio::buffer(buff->cast<void *>(), buff->size()), *endpoint);
            if (E200_NETWORK_DEBUG) UHD_MSG(status) << name << " got " << num_bytes << std::endl;

            //step 3 - commit the buffer
            buff->commit(num_bytes);
        }
    }
    catch(const std::exception &ex)
    {
        UHD_MSG(error) << "e200_send_tunnel exit " << name << " " << ex.what() << std::endl;
    }
    catch(...)
    {
        UHD_MSG(error) << "e200_send_tunnel exit " << name << std::endl;
    }
    UHD_MSG(status) << "e200_send_tunnel exit " << name << std::endl;
    *running = false;
}

/***********************************************************************
 * codec gateway
 **********************************************************************/
static void codec_gateway(
    ad9361_ctrl_iface_sptr ctrl,
    boost::shared_ptr<asio::ip::udp::socket> sock,
    asio::ip::udp::endpoint *endpoint,
    bool *running
)
{
    unsigned char in_buff[64];
    unsigned char out_buff[64];
    try
    {
        while (*running)
        {
            const size_t num_bytes = sock->receive_from(asio::buffer(in_buff), *endpoint);
            if (E200_NETWORK_DEBUG) UHD_MSG(status) << "codec_gateway got " << num_bytes << std::endl;
            if (num_bytes < 64) continue;
            ctrl->transact(in_buff, out_buff);
            sock->send_to(asio::buffer(out_buff), *endpoint);
        }
    }
    catch(const std::exception &ex)
    {
        UHD_MSG(error) << "codec_gateway exit" << " " << ex.what() << std::endl;
    }
    catch(...)
    {
        UHD_MSG(error) << "codec_gateway exit" << std::endl;
    }
    UHD_MSG(status) << "codec_gateway exit" << std::endl;
    *running = false;
}

/***********************************************************************
 * The TCP server itself
 **********************************************************************/
void e200_impl::run_server(const std::string &port, const std::string &what)
{
    asio::io_service io_service;
    asio::ip::udp::resolver resolver(io_service);
    asio::ip::udp::resolver::query query(asio::ip::udp::v4(), "0.0.0.0", port);
    asio::ip::udp::endpoint endpoint = *resolver.resolve(query);

    //boost::shared_ptr<asio::ip::udp::acceptor> acceptor(new asio::ip::udp::acceptor(io_service, endpoint));
    while (not boost::this_thread::interruption_requested())
    {
        UHD_MSG(status) << "e200 run server on port " << port << " for " << what << std::endl;
        try
        {
            //while (not wait_for_recv_ready(acceptor->native(), 100))
            //{
            //    if (boost::this_thread::interruption_requested()) return;
            //}
            boost::shared_ptr<asio::ip::udp::socket> socket;
            socket.reset(new asio::ip::udp::socket(io_service, endpoint));
            //acceptor->accept(*socket);
            UHD_MSG(status) << "e200 socket accept on port " << port << " for " << what << std::endl;
            //asio::ip::udp::no_delay option(true);
            //socket->set_option(option);
            boost::thread_group tg;
            bool running = true;
            radio_perifs_t &perif = _radio_perifs[0];
            if (what == "RX")
            {
                tg.create_thread(boost::bind(&e200_recv_tunnel, "RX data tunnel", perif.rx_data_xport, socket, &endpoint, &running));
                tg.create_thread(boost::bind(&e200_send_tunnel, "RX flow tunnel", socket, perif.rx_flow_xport, &endpoint, &running));
            }
            if (what == "TX")
            {
                tg.create_thread(boost::bind(&e200_recv_tunnel, "TX flow tunnel", perif.tx_flow_xport, socket, &endpoint, &running));
                tg.create_thread(boost::bind(&e200_send_tunnel, "TX data tunnel", socket, perif.tx_data_xport, &endpoint, &running));
            }
            if (what == "CTRL")
            {
                tg.create_thread(boost::bind(&e200_recv_tunnel, "response tunnel", perif.recv_ctrl_xport, socket, &endpoint, &running));
                tg.create_thread(boost::bind(&e200_send_tunnel, "control tunnel", socket, perif.send_ctrl_xport, &endpoint, &running));
            }
            if (what == "CODEC")
            {
                codec_gateway(_codec_ctrl_iface, socket, &endpoint, &running);
            }
            tg.join_all();
            socket->close();
            socket.reset();
        }
        catch(...){}
    }
}
