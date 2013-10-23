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

#ifndef INCLUDED_RPC_COMMON_HPP
#define INCLUDED_RPC_COMMON_HPP

#define USE_BINARY_ARCHIVE 0

#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#if (USE_BINARY_ARCHIVE)
    #include <boost/archive/binary_oarchive.hpp>
    #include <boost/archive/binary_iarchive.hpp>
#else
    #include <boost/archive/text_oarchive.hpp>
    #include <boost/archive/text_iarchive.hpp>
#endif

namespace usrprio_rpc {

//[Over-the-wire] IDs
typedef boost::int32_t  func_id_t;
typedef boost::uint64_t client_id_t;

//[Over-the-wire] Handshake format
struct hshake_args_t {
    boost::uint32_t version;
    boost::uint32_t oldest_comp_version;
    union {
        client_id_t         client_id;
        struct {
            boost::uint32_t pid;
            boost::uint32_t hid;
        } id;
    };
};

//[Over-the-wire] Header for RPC request and response
class func_args_header_t {
public:
    func_id_t       func_id;
    client_id_t     client_id;
    boost::uint32_t func_args_size;

    static bool match_function(const func_args_header_t& a, const func_args_header_t& b) {
        return ((a.func_id == b.func_id) && (a.client_id == b.client_id));
    }
};

//[Internal] Storage for RPC header and arguments
class func_xport_buf_t {
public:
    func_args_header_t  header;
    std::vector<char>   data;
};

//[Internal] Serializer for RPC input parameters
class func_args_writer_t {
public:
    func_args_writer_t() : _stream(), _archive(_stream) {}

    template<typename data_t>
    void push(const data_t& d) {
        _archive << d;
    }

    template<typename data_t>
    friend func_args_writer_t& operator<< (func_args_writer_t& out, const data_t& data) {
        out.push(data);
        return out;
    }

    void store(std::vector<char>& data) const {
        const std::string& str = _stream.str();
        data.resize(str.length());
        data.assign((char*)str.c_str(), ((char*)str.c_str()) + str.length());
    }

private:
    std::ostringstream              _stream;
#if (USE_BINARY_ARCHIVE)
    boost::archive::binary_oarchive _archive;
#else
    boost::archive::text_oarchive   _archive;
#endif
};

//[Internal] Deserializer for RPC output parameters
class func_args_reader_t {
public:
    func_args_reader_t() : _stream(), _archive() {}

    template<typename data_t>
    void pull(data_t& d) const {
        if (_archive) (*_archive) >> d;
    }

    template<typename data_t>
    friend const func_args_reader_t& operator>> (const func_args_reader_t& in, data_t& data) {
        in.pull(data);
        return in;
    }

    void load(const std::vector<char>& data) {
        _stream.str(std::string(data.begin(), data.end()));
#if (USE_BINARY_ARCHIVE)
        _archive.reset(new boost::archive::binary_iarchive(_stream));
#else
        _archive.reset(new boost::archive::text_iarchive(_stream));
#endif
    }

private:
    std::istringstream                                  _stream;
#if (USE_BINARY_ARCHIVE)
    boost::scoped_ptr<boost::archive::binary_iarchive>  _archive;
#else
    boost::scoped_ptr<boost::archive::text_iarchive>    _archive;
#endif
};

}

#undef USE_BINARY_ARCHIVE

#endif /* INCLUDED_RPC_COMMON_HPP */
