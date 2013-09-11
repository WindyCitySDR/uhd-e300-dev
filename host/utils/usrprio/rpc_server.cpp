///
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

#include "rpc_server.hpp"
#include "rpc_logger.hpp"
#include <boost/format.hpp>
#include <boost/foreach.hpp>

namespace usrprio_rpc {

using boost::asio::ip::tcp;

rpc_server::rpc_server(
    boost::uint32_t port,
    callback_func_t callback_func,
    boost::uint32_t max_pool_size)
: _io_service(),
  _acceptor(),
  _stop_sig_handler(_io_service),
  _max_thread_pool_size(max_pool_size),
  _callback_func(callback_func)
{
    // Register to handle the signals that indicate when the server should exit.
    // It is safe to register for the same signal multiple times in a program,
    // provided all registration for the specified signal is made through Asio.
    _stop_sig_handler.add(SIGINT);
    _stop_sig_handler.add(SIGTERM);
#if defined(SIGQUIT)
    _stop_sig_handler.add(SIGQUIT);
#endif // defined(SIGQUIT)
    _stop_sig_handler.async_wait(boost::bind(&rpc_server::_handle_stop, this));


    static const bool REUSE_ADDRS = true;
    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), port);
    RPC_LOG(boost::format("RPC Server starting on %s:%d") % endpoint.address() % port, LOG_VERBOSE)

    _acceptor.reset(new boost::asio::ip::tcp::acceptor(_io_service, endpoint, REUSE_ADDRS));

    if (!_acceptor || !_acceptor->is_open())
        _svr_err.assign(boost::asio::error::service_not_found, boost::system::system_category());
}

rpc_server::~rpc_server()
{
    stop();
}

boost::system::error_code& rpc_server::run()
{
    boost::mutex::scoped_lock lock(_mutex);

    if (!_svr_err) {
        RPC_LOG("RPC server started", LOG_STATUS)
        bool success = true;
        while (success) {
            if (_io_service.stopped() || !_acceptor->is_open()) break;

            //Block until a client connects and use the io_service handler
            //to process the connection
            success = _accept_new_connection();
        }

        //Wait for all io_service handlers to finish executing
        _thread_pool.join_all();
        RPC_LOG("RPC server stopped", LOG_STATUS)
    }
    return _svr_err;
}

void rpc_server::stop() {
    _handle_stop();
}

void rpc_server::_handle_stop()
{
    RPC_LOG("RPC server stopping", LOG_VERBOSE)
    _acceptor->close();
    _io_service.stop();
}

bool rpc_server::_accept_new_connection() {
    rpc_connection::sptr new_conn(
        new rpc_connection(_io_service, *_acceptor, _callback_func, CURRENT_VERSION, OLDEST_COMPATIBLE_VERSION));

    if (!new_conn->start()) {
        if (_num_connected() < _thread_pool.size()) {
            //Find an unused connection and replace it
            for (size_t i = 0; i < _connections.size(); i++) {
                if (_connections[i].get() == NULL || !_connections[i]->is_active()) {
                    _connections[i].reset();
                    _connections[i] = new_conn;
                    break;
                }
            }
        } else {
            //All worker threads busy. Create a new one.
            if (_thread_pool.size() < _max_thread_pool_size) {
                _thread_pool.create_thread(boost::bind(&boost::asio::io_service::run, &_io_service));
            }
            _connections.push_back(new_conn);
            RPC_LOG(boost::format("Grew thread pool to %d threads") % _thread_pool.size(), LOG_STATUS)
        }
    }

    return new_conn->is_active();
}

size_t rpc_server::_num_connected() {
    size_t count = 0;
    for (size_t i = 0; i < _connections.size(); i++) {
        if (_connections[i].get() != NULL && _connections[i]->is_active()) count++;
    }
    return count;
}

}
