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

#ifndef INCLUDED_RPC_SERVER_HPP
#define INCLUDED_RPC_SERVER_HPP

#include <string>
#include <iostream>
#include <istream>
#include <ostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <uhd/transport/nirio/rpc/rpc_common.hpp>
#include "rpc_connection.hpp"

namespace usrprio_rpc {

class rpc_server : private boost::noncopyable
{
public:
    static const boost::uint32_t CURRENT_VERSION = 1;
    static const boost::uint32_t OLDEST_COMPATIBLE_VERSION = 1;

    typedef rpc_connection::callback_func_t callback_func_t;

    explicit rpc_server(
        const std::string& address,
        const std::string& port,
        callback_func_t callback_func,
        boost::uint32_t max_pool_size = std::numeric_limits<boost::uint32_t>::max());
    void run();
    void stop();

private:
    bool _accept_new_connection();
    size_t _num_connected();
     void _handle_stop();

     //Services
    boost::asio::io_service             _io_service;
    boost::asio::ip::tcp::acceptor      _acceptor;
    boost::asio::signal_set             _stop_sig_handler;
    //Connections and threads
    std::vector<rpc_connection::sptr>   _connections;
    boost::thread_group                 _thread_pool;
    boost::uint32_t                     _max_thread_pool_size;
    //Callback func
    callback_func_t                     _callback_func;
    //Synchronization
    boost::mutex                        _mutex;
};

}

#endif /* INCLUDED_RPC_SERVER_HPP */
