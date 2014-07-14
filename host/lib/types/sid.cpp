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

#include <boost/format.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/sid.hpp>

using namespace uhd;

sid_t::sid_t(boost::uint32_t sid)
    : _sid(sid), _set(true)
{
}

sid_t::sid_t()
    : _sid(0x0000), _set(false)
{
}

std::string sid_t::to_pp_string() const
{
    if (not _set) {
        return "x.x.x.x";
    }
    return str(boost::format("%d.%d.%d.%d")
        % get_remote_src_address()
        % get_local_src_address()
        % get_remote_dst_address()
        % get_local_dst_address());
}

std::string sid_t::to_pp_string_hex() const
{
    if (not _set) {
        return "xx:xx:xx:xx";
    }
    return str(boost::format("%02x:%02x:%02x:%02x")
        % get_remote_src_address()
        % get_local_src_address()
        % get_remote_dst_address()
        % get_local_dst_address());
}


void sid_t::set_sid(boost::uint32_t new_sid)
{
    _set = true;
    _sid = new_sid;
}

void sid_t::set_src_address(boost::uint32_t new_addr) {
    set_sid((_sid & 0x0000FFFF) | ((new_addr & 0xFFFF) << 16));
}
//! the destination address of this SID
//  (the last 16 Bits)
void sid_t::set_dst_address(boost::uint32_t new_addr) {
    set_sid((_sid & 0xFFFF0000) | (new_addr & 0xFFFF));
}
//! remote part of source address
void sid_t::set_remote_src_address(boost::uint32_t new_addr) {
    set_sid((_sid & 0x00FFFFFF) | ((new_addr & 0xFF) << 24));
}
//! local part of source address
void sid_t::set_local_src_address(boost::uint32_t new_addr) {
    set_sid((_sid & 0xFF00FFFF) | ((new_addr & 0xFF) << 16));
}
//! remote part of destination address
void sid_t::set_remote_dst_address(boost::uint32_t new_addr) {
    set_sid((_sid & 0xFFFF00FF) | ((new_addr & 0xFF) << 8));
}
//! local part of source address
void sid_t::set_local_dst_address(boost::uint32_t new_addr) {
    set_sid((_sid & 0xFFFFFF00) | ((new_addr & 0xFF) << 0));
}
