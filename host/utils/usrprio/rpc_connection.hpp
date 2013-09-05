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

#ifndef INCLUDED_RPC_CONNECTION_HPP
#define INCLUDED_RPC_CONNECTION_HPP

#include <boost/asio.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/function.hpp>
#include <uhd/transport/nirio/rpc/rpc_common.hpp>

namespace usrprio_rpc {

class rpc_connection : private boost::noncopyable
{
public:
    typedef boost::shared_ptr<rpc_connection> sptr;
    typedef boost::function<void (func_id_t, const func_args_reader_t&, func_args_writer_t&, client_id_t)> callback_func_t;

    explicit rpc_connection(
        boost::asio::io_service& io_service,
        boost::asio::ip::tcp::acceptor& acceptor,
        callback_func_t callback_func,
        boost::uint32_t current_version,
        boost::uint32_t oldest_compatible_version);

    boost::system::error_code& start();

    bool is_active() const {
        return _is_active && !_exec_err;
    }

private:
    void _handle_accept(
        const boost::system::error_code& err);
    void _handle_request(
        const boost::system::error_code& err,
        size_t transferred,
        size_t expected);
    void _wait_for_next_request_header();

    //Services
    boost::asio::ip::tcp::socket        _socket;
    boost::asio::ip::tcp::acceptor&     _acceptor;
    //Function info storage
    hshake_args_t                       _hshake_args;
    func_xport_buf_t                    _request;
    func_xport_buf_t                    _response;
    callback_func_t                     _callback_func;
    //State info
    bool                                _is_active;
    boost::uint32_t                     _current_version;
    boost::uint32_t                     _oldest_compatible_version;
    boost::system::error_code           _exec_err;
    //Synchronization
    boost::mutex                        _mutex;
};

} // namespace usrprio_rpc

#endif // INCLUDED_RPC_CONNECTION_HPP
