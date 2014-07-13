//
// Copyright 2014 Ettus Research LLC
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

#ifndef INCLUDED_UHD_TYPES_CVITA_SID_HPP
#define INCLUDED_UHD_TYPES_CVITA_SID_HPP

#include <string>
#include <boost/format.hpp>

namespace uhd {

class cvita_sid_t {
public:
    cvita_sid_t(): _src_addr(0), _src_endpoint(0),
                _dest_addr(0), _dest_endpoint(0)
    { }

    cvita_sid_t(boost::uint8_t src_addr, boost::uint8_t src_endpoint,
              boost::uint8_t dest_addr, boost::uint8_t dest_endpoint):
      _src_addr(src_addr), _src_endpoint(src_endpoint),
      _dest_addr(dest_addr), _dest_endpoint(dest_endpoint)
    { }

    inline boost::uint8_t get_src_addr() const      { return _src_addr; }
    inline boost::uint8_t get_src_endpoint() const  { return _src_endpoint; }
    inline boost::uint8_t get_dest_addr() const     { return _dest_addr; }
    inline boost::uint8_t get_dest_endpoint() const { return _dest_endpoint; }

    inline boost::uint32_t get() const {
        return  (((boost::uint32_t)_src_addr)     << 24) |
                (((boost::uint32_t)_src_endpoint) << 16) |
                (((boost::uint32_t)_dest_addr)    << 8) |
                (((boost::uint32_t)_dest_endpoint)<< 0);
    }

    inline void set(boost::uint32_t sid_u32) {
        _src_addr       = (sid_u32 >> 24) & 0xFF;
        _src_endpoint   = (sid_u32 >> 16) & 0xFF;
        _dest_addr      = (sid_u32 >> 8) & 0xFF;
        _dest_endpoint  = (sid_u32 >> 0) & 0xFF;
    }

    inline operator boost::uint32_t() const {
        return get();
    }

    inline cvita_sid_t flip() const {
        return cvita_sid_t(_dest_addr, _dest_endpoint, _src_addr, _src_endpoint);
    }

    inline std::string to_string() const {
        return str(boost::format("[Src=%.2x:%.2x, Dest=%.2x:%.2x]")
            % _src_addr % _src_endpoint % _dest_addr % _dest_endpoint);
    }

private:
    boost::uint8_t  _src_addr;
    boost::uint8_t  _src_endpoint;
    boost::uint8_t  _dest_addr;
    boost::uint8_t  _dest_endpoint;
};

class cvita_sid_pair_t {
public:
    cvita_sid_t in;
    cvita_sid_t out;

    cvita_sid_pair_t()
    {}
    cvita_sid_pair_t(const cvita_sid_t& in_, const cvita_sid_t& out_):
        in(in_), out(out_)
    {}
};

} //namespace uhd

#endif /* INCLUDED_UHD_TYPES_CVITA_SID_HPP */
