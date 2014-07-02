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

#include "e300_impl.hpp"

#include "ad9361_ctrl.hpp"
#include "ad9361_driver/ad9361_transaction.h"

#include <uhd/utils/msg.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

using namespace uhd;
using namespace uhd::transport;
namespace asio = boost::asio;

namespace uhd { namespace usrp { namespace e300 {

static const size_t E300_NETWORK_DEBUG = false;

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

static boost::mutex endpoint_mutex;

/***********************************************************************
 * Receive tunnel - forwards recv interface to send socket
 **********************************************************************/
static void e300_recv_tunnel(
    const std::string &name,
    uhd::transport::zero_copy_if::sptr recver,
    boost::shared_ptr<asio::ip::udp::socket> sender,
    asio::ip::udp::endpoint *endpoint,
    bool *running
)
{
    asio::ip::udp::endpoint _tx_endpoint;
    try
    {
        while (*running)
        {
            //step 1 - get the buffer
            managed_recv_buffer::sptr buff = recver->get_recv_buff();
            if (not buff) continue;
            if (E300_NETWORK_DEBUG) UHD_MSG(status) << name << " got " << buff->size() << std::endl;

            //step 1.5 -- update endpoint
            {
                boost::mutex::scoped_lock l(endpoint_mutex);
                _tx_endpoint = *endpoint;
            }

            //step 2 - send to the socket
            sender->send_to(asio::buffer(buff->cast<const void *>(), buff->size()), _tx_endpoint);
        }
    }
    catch(const std::exception &ex)
    {
        UHD_MSG(error) << "e300_recv_tunnel exit " << name << " " << ex.what() << std::endl;
    }
    catch(...)
    {
        UHD_MSG(error) << "e300_recv_tunnel exit " << name << std::endl;
    }
    UHD_MSG(status) << "e300_recv_tunnel exit " << name << std::endl;
    *running = false;
}

/***********************************************************************
 * Send tunnel - forwards recv socket to send interface
 **********************************************************************/
static void e300_send_tunnel(
    const std::string &name,
    boost::shared_ptr<asio::ip::udp::socket> recver,
    uhd::transport::zero_copy_if::sptr sender,
    asio::ip::udp::endpoint *endpoint,
    bool *running
)
{
    asio::ip::udp::endpoint _rx_endpoint;
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
            const size_t num_bytes = recver->receive_from(asio::buffer(buff->cast<void *>(), buff->size()), _rx_endpoint);
            if (E300_NETWORK_DEBUG) UHD_MSG(status) << name << " got " << num_bytes << std::endl;

            //step 2.5 -- update endpoint
            {
                boost::mutex::scoped_lock l(endpoint_mutex);
                *endpoint = _rx_endpoint;
            }

            //step 3 - commit the buffer
            buff->commit(num_bytes);
        }
    }
    catch(const std::exception &ex)
    {
        UHD_MSG(error) << "e300_send_tunnel exit " << name << " " << ex.what() << std::endl;
    }
    catch(...)
    {
        UHD_MSG(error) << "e300_send_tunnel exit " << name << std::endl;
    }
    UHD_MSG(status) << "e300_send_tunnel exit " << name << std::endl;
    *running = false;
}

static void e300_codec_ctrl_tunnel(
    const std::string &name,
    boost::shared_ptr<asio::ip::udp::socket> socket,
    ad9361_ctrl_transport::sptr _ctrl_xport,
    asio::ip::udp::endpoint *endpoint,
    bool *running
)
{
    asio::ip::udp::endpoint _endpoint;
    try
    {
        while (*running)
        {
            uint8_t in_buff[64] = {};
            uint8_t out_buff[64] = {};

            const size_t num_bytes = socket->receive_from(asio::buffer(in_buff), *endpoint);

            if (num_bytes < 64) {
                std::cout << "Received short packet" << std::endl;
                continue;
            }

            ad9361_transaction_t *in = reinterpret_cast<ad9361_transaction_t*>(in_buff);

            in->handle = _ctrl_xport->get_device_handle();
            in->version = AD9361_TRANSACTION_VERSION;

            _ctrl_xport->ad9361_transact(&in_buff[0], &out_buff[0]);

            socket->send_to(asio::buffer(out_buff, 64), *endpoint);
        }
    }
    catch(const std::exception &ex)
    {
        UHD_MSG(error) << "e300_ctrl_tunnel exit " << name << " " << ex.what() << std::endl;
    }
    catch(...)
    {
        UHD_MSG(error) << "e300_ctrl_tunnel exit " << name << std::endl;
    }
    UHD_MSG(status) << "e300_ctrl_tunnel exit " << name << std::endl;
    *running = false;
}

static void e300_global_regs_tunnel(
    const std::string &name,
    boost::shared_ptr<asio::ip::udp::socket> socket,
    uhd::usrp::e300::global_regs::sptr regs,
    asio::ip::udp::endpoint *endpoint,
    bool *running
)
{
    UHD_ASSERT_THROW(regs);
    asio::ip::udp::endpoint _endpoint;
    try
    {
        while (*running)
        {
            uint8_t in_buff[16] = {};

            const size_t num_bytes = socket->receive_from(asio::buffer(in_buff), *endpoint);

            if (num_bytes < 16) {
                std::cout << "Received short packet: " << num_bytes << std::endl;
                continue;
            }

            uhd::usrp::e300::global_regs_transaction_t *in =
                reinterpret_cast<uhd::usrp::e300::global_regs_transaction_t *>(in_buff);

            if(in->is_poke) {
                regs->poke32(in->addr, in->data);
            }
            else {
                in->data = regs->peek32(in->addr);
                socket->send_to(asio::buffer(in_buff, 16), *endpoint);
            }
        }
    }
    catch(const std::exception &ex)
    {
        UHD_MSG(error) << "e300_gregs_tunnel exit " << name << " " << ex.what() << std::endl;
    }
    catch(...)
    {
        UHD_MSG(error) << "e300_gregs_tunnel exit " << name << std::endl;
    }
    UHD_MSG(status) << "e300_gregs_tunnel exit " << name << std::endl;
    *running = false;
}


/***********************************************************************
 * The TCP server itself
 **********************************************************************/
void e300_impl::run_server(const std::string &port, const std::string &what)
{
    asio::io_service io_service;
    asio::ip::udp::resolver resolver(io_service);
    asio::ip::udp::resolver::query query(asio::ip::udp::v4(), "0.0.0.0", port);
    asio::ip::udp::endpoint endpoint = *resolver.resolve(query);

    //boost::shared_ptr<asio::ip::udp::acceptor> acceptor(new asio::ip::udp::acceptor(io_service, endpoint));
    while (not boost::this_thread::interruption_requested())
    {
        UHD_MSG(status) << "e300 run server on port " << port << " for " << what << std::endl;
        try
        {
            //while (not wait_for_recv_ready(acceptor->native(), 100))
            //{
            //    if (boost::this_thread::interruption_requested()) return;
            //}
            boost::shared_ptr<asio::ip::udp::socket> socket;
            socket.reset(new asio::ip::udp::socket(io_service, endpoint));
            //acceptor->accept(*socket);
            UHD_MSG(status) << "e300 socket accept on port " << port << " for " << what << std::endl;
            //asio::ip::udp::no_delay option(true);
            //socket->set_option(option);
            boost::thread_group tg;
            bool running = true;
            radio_perifs_t &perif = _radio_perifs[0];
            if (what == "RX")
            {
                tg.create_thread(boost::bind(&e300_recv_tunnel, "RX data tunnel", perif.rx_data_xport, socket, &endpoint, &running));
                tg.create_thread(boost::bind(&e300_send_tunnel, "RX flow tunnel", socket, perif.rx_flow_xport, &endpoint, &running));
            }
            if (what == "TX")
            {
                tg.create_thread(boost::bind(&e300_recv_tunnel, "TX flow tunnel", perif.tx_flow_xport, socket, &endpoint, &running));
                tg.create_thread(boost::bind(&e300_send_tunnel, "TX data tunnel", socket, perif.tx_data_xport, &endpoint, &running));
            }
            if (what == "CTRL")
            {
                tg.create_thread(boost::bind(&e300_recv_tunnel, "response tunnel", perif.recv_ctrl_xport, socket, &endpoint, &running));
                tg.create_thread(boost::bind(&e300_send_tunnel, "control tunnel", socket, perif.send_ctrl_xport, &endpoint, &running));
            }
            if (what == "CODEC")
            {
                tg.create_thread(boost::bind(&e300_codec_ctrl_tunnel, "CODEC tunnel", socket, _codec_xport, &endpoint, &running));
            }
            if (what == "GREGS")
            {
                tg.create_thread(boost::bind(&e300_global_regs_tunnel, "GREGS tunnel", socket, _global_regs, &endpoint, &running));
            }


            tg.join_all();
            socket->close();
            socket.reset();
        }
        catch(...){}
    }
}
}}} // namespace
