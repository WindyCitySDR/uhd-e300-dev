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

#include "rpc_connection.hpp"
#include <vector>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include "rpc_logger.hpp"

#define CHAIN_BLOCKING_XFER(func, exp, status) \
    if (status) status = (static_cast<size_t>((func)) == exp);

namespace usrprio_rpc {

rpc_connection::rpc_connection(
    boost::asio::io_service& io_service,
    boost::asio::ip::tcp::acceptor& acceptor,
    callback_func_t callback_func,
    boost::uint32_t current_version,
    boost::uint32_t oldest_compatible_version)
: _socket(io_service),
  _acceptor(acceptor),
  _callback_func(callback_func),
  _is_active(false),
  _current_version(current_version),
  _oldest_compatible_version(oldest_compatible_version)
{
}

boost::system::error_code& rpc_connection::start() {
    boost::mutex::scoped_lock lock(_mutex);

    _is_active = false;
    try {
        //Block until a client connects to the socket.
        //If the server is stopped, the acceptor will be closed and the "accept" call will throw an exception.
        _acceptor.accept(_socket);
        RPC_LOG("Accepted client connection request", LOG_VERBOSE)
    } catch (boost::exception& e) {
        _exec_err.assign(boost::asio::error::connection_aborted, boost::system::system_category());
    }

    if (!_exec_err) {
        try {
            //Perform a synchronous handshake
            //If the server is stopped or the connection is severed,
            //the "read" and "write" calls will throw an exception.
            bool status = true;
            CHAIN_BLOCKING_XFER(
                boost::asio::read(_socket, boost::asio::buffer(&_hshake_args, sizeof(_hshake_args))),
                sizeof(_hshake_args), status);
            _hshake_args.version = _current_version;
            _hshake_args.oldest_comp_version = _oldest_compatible_version;

            if (status && (_hshake_args.version >= _oldest_compatible_version)) {
                boost::asio::write(_socket, boost::asio::buffer(&_hshake_args, sizeof(_hshake_args)));
            } else {
                RPC_LOG(boost::format("Version mismatch for client 0x%x") % _hshake_args.client_id, LOG_WARNING)
                _exec_err.assign(boost::asio::error::connection_refused, boost::system::system_category());
            }
        } catch (boost::exception& e) {
            _exec_err.assign(boost::asio::error::connection_aborted, boost::system::system_category());
        }

        if (!_exec_err) {
            RPC_LOG(boost::format("Connection to client 0x%x established") % _hshake_args.client_id, LOG_STATUS)
            _is_active = true;

            //Asynchronously wait for function call header request.
            _wait_for_next_request_header();
        }
    }

    return _exec_err;
}

void rpc_connection::_wait_for_next_request_header() {
    boost::asio::async_read(
        _socket,
        boost::asio::buffer(&_request.header, sizeof(_request.header)),
        boost::bind(&rpc_connection::_handle_request, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred,
            sizeof(_request.header)));
}

void rpc_connection::_handle_request(const boost::system::error_code& err, size_t transferred, size_t expected) {
    boost::mutex::scoped_lock lock(_mutex);

    _exec_err = err;
    if (transferred != expected) {
        _exec_err.assign(boost::asio::error::operation_aborted, boost::system::system_category());
    }

    if (!_exec_err) {
        //Header successfully received.
        if (_hshake_args.client_id == _request.header.client_id) {
            //Receive data if the signature matches the handshake ID
            try {
                _request.data.resize(_request.header.func_args_size);

                //Synchronously receive function args and respond.
                //If the server is stopped or the connection is severed,
                //the "read" and "write" calls will throw an exception.

                bool status = true;
                CHAIN_BLOCKING_XFER(
                    boost::asio::read(_socket, boost::asio::buffer(&(*_request.data.begin()), _request.data.size())),
                    _request.data.size(), status);

                func_args_reader_t in_args;
                func_args_writer_t out_args;

                in_args.load(_request.data);
                RPC_LOG(boost::format("Calling function %d for client 0x%x") % _request.header.func_id % _hshake_args.client_id, LOG_VERBOSE);
                _callback_func(_request.header.func_id, in_args, out_args, _request.header.client_id);
                out_args.store(_response.data);

                _response.header = _request.header;
                _response.header.func_args_size = _response.data.size();

                //Send the response
                CHAIN_BLOCKING_XFER(
                    boost::asio::write(_socket, boost::asio::buffer(&_response.header, sizeof(_response.header))),
                    sizeof(_response.header), status);
                CHAIN_BLOCKING_XFER(
                    boost::asio::write(_socket, boost::asio::buffer(&(*_response.data.begin()), _response.data.size())),
                    _response.data.size(), status);

                if (!status)
                    _exec_err.assign(boost::asio::error::operation_aborted, boost::system::system_category());

            } catch (boost::exception& e) {
                _exec_err.assign(boost::asio::error::operation_aborted, boost::system::system_category());
            }
        }

        //Asynchronously wait for the next function call header request.
        _wait_for_next_request_header();
    }

    if(_exec_err) {
        RPC_LOG(boost::format("Connection to client 0x%x closed") % _hshake_args.client_id, LOG_STATUS)
        _is_active = false;
        _exec_err.assign(boost::asio::error::eof, boost::system::system_category());
    }
}

} // namespace usrprio_rpc
