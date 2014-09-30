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

#include <iostream>
#include <sstream>
#include <boost/test/unit_test.hpp>
#include <uhd/types/sid.hpp>
#include <uhd/exception.hpp>

using uhd::sid_t;

BOOST_AUTO_TEST_CASE(test_sid_t) {
    boost::uint32_t sid_value = 0x01020310;
    sid_t sid(sid_value);

    BOOST_CHECK_EQUAL(sid.is_set(), true);
    BOOST_CHECK_EQUAL(sid.to_pp_string(), "1.2>3.16");
    BOOST_CHECK_EQUAL(sid.to_pp_string_hex(), "01:02>03:10");
    BOOST_CHECK_EQUAL(sid.get_src_address(), 0x0102);
    BOOST_CHECK_EQUAL(sid.get_dst_address(), 0x0310);
    BOOST_CHECK_EQUAL(sid.get_remote_src_address(), 0x01);
    BOOST_CHECK_EQUAL(sid.get_local_src_address(), 0x02);
    BOOST_CHECK_EQUAL(sid.get_remote_dst_address(), 0x03);
    BOOST_CHECK_EQUAL(sid.get_local_dst_address(), 0x10);
    BOOST_CHECK_EQUAL(sid == sid, true);
    BOOST_CHECK_EQUAL(sid == sid_value, true);

    boost::uint32_t check_sid_val = (boost::uint32_t) sid;
    BOOST_CHECK_EQUAL(check_sid_val, sid_value);

    std::stringstream ss_dec;
    ss_dec << sid;
    BOOST_CHECK_EQUAL(ss_dec.str(), "1.2>3.16");

    std::stringstream ss_hex;
    ss_hex << std::hex << sid;
    BOOST_CHECK_EQUAL(ss_hex.str(), "01:02>03:10");

    sid_t empty_sid;
    BOOST_CHECK_EQUAL(empty_sid.is_set(), false);
    BOOST_CHECK_EQUAL(empty_sid.to_pp_string(), "x.x>x.x");
    BOOST_CHECK_EQUAL(empty_sid.to_pp_string_hex(), "xx:xx>xx:xx");
    BOOST_CHECK_EQUAL(empty_sid == sid, false);
    BOOST_CHECK_EQUAL(empty_sid == sid_value, false);
    BOOST_CHECK_EQUAL((bool) empty_sid, false);

    empty_sid = sid_value; // No longer empty
    BOOST_CHECK_EQUAL(empty_sid.is_set(), true);
    BOOST_CHECK_EQUAL(empty_sid == sid, true);
}

BOOST_AUTO_TEST_CASE(test_sid_t_set) {
    boost::uint32_t sid_value = 0x0;
    sid_t sid(sid_value);

    sid.set(0x01020304);
    BOOST_CHECK_EQUAL(sid.get(), 0x01020304);
    BOOST_CHECK_EQUAL(sid.get_remote_src_address(), 0x01);
    BOOST_CHECK_EQUAL(sid.get_local_src_address(), 0x02);
    BOOST_CHECK_EQUAL(sid.get_remote_dst_address(), 0x03);
    BOOST_CHECK_EQUAL(sid.get_local_dst_address(), 0x04);

    sid.set_remote_src_address(0x0a);
    BOOST_CHECK_EQUAL(sid.get(), 0x0a020304);
    BOOST_CHECK_EQUAL(sid.get_remote_src_address(), 0x0a);
    BOOST_CHECK_EQUAL(sid.get_local_src_address(), 0x02);
    BOOST_CHECK_EQUAL(sid.get_remote_dst_address(), 0x03);
    BOOST_CHECK_EQUAL(sid.get_local_dst_address(), 0x04);

    sid.set_local_src_address(0x0b);
    BOOST_CHECK_EQUAL(sid.get(), 0x0a0b0304);
    BOOST_CHECK_EQUAL(sid.get_remote_src_address(), 0x0a);
    BOOST_CHECK_EQUAL(sid.get_local_src_address(), 0x0b);
    BOOST_CHECK_EQUAL(sid.get_remote_dst_address(), 0x03);
    BOOST_CHECK_EQUAL(sid.get_local_dst_address(), 0x04);

    sid.set_remote_dst_address(0x0c);
    BOOST_CHECK_EQUAL(sid.get(), 0x0a0b0c04);
    BOOST_CHECK_EQUAL(sid.get_remote_src_address(), 0x0a);
    BOOST_CHECK_EQUAL(sid.get_local_src_address(), 0x0b);
    BOOST_CHECK_EQUAL(sid.get_remote_dst_address(), 0x0c);
    BOOST_CHECK_EQUAL(sid.get_local_dst_address(), 0x04);

    sid.set_local_dst_address(0x0d);
    BOOST_CHECK_EQUAL(sid.get(), 0x0a0b0c0d);
    BOOST_CHECK_EQUAL(sid.get_remote_src_address(), 0x0a);
    BOOST_CHECK_EQUAL(sid.get_local_src_address(), 0x0b);
    BOOST_CHECK_EQUAL(sid.get_remote_dst_address(), 0x0c);
    BOOST_CHECK_EQUAL(sid.get_local_dst_address(), 0x0d);

    sid_t flipped_sid = sid.reversed();
    BOOST_CHECK_EQUAL(flipped_sid.get(), 0x0c0d0a0b);
}

BOOST_AUTO_TEST_CASE(test_sid_t_from_str) {
    sid_t sid("1.2>3.4");
    BOOST_CHECK_EQUAL(sid.get_remote_src_address(), 1);
    BOOST_CHECK_EQUAL(sid.get_local_src_address(), 2);
    BOOST_CHECK_EQUAL(sid.get_remote_dst_address(), 3);
    BOOST_CHECK_EQUAL(sid.get_local_dst_address(), 4);

    sid = "01:02>03:10";
    BOOST_CHECK_EQUAL(sid.get_remote_src_address(), 1);
    BOOST_CHECK_EQUAL(sid.get_local_src_address(), 2);
    BOOST_CHECK_EQUAL(sid.get_remote_dst_address(), 3);
    BOOST_CHECK_EQUAL(sid.get_local_dst_address(), 16);

    sid = "01:06/03:10";
    BOOST_CHECK_EQUAL(sid.get_remote_src_address(), 1);
    BOOST_CHECK_EQUAL(sid.get_local_src_address(), 6);
    BOOST_CHECK_EQUAL(sid.get_remote_dst_address(), 3);
    BOOST_CHECK_EQUAL(sid.get_local_dst_address(), 16);

    sid = "01:02:04:10";
    BOOST_CHECK_EQUAL(sid.get_remote_src_address(), 1);
    BOOST_CHECK_EQUAL(sid.get_local_src_address(), 2);
    BOOST_CHECK_EQUAL(sid.get_remote_dst_address(), 4);
    BOOST_CHECK_EQUAL(sid.get_local_dst_address(), 16);

    BOOST_REQUIRE_THROW(sid_t fail_sid("foobar"), uhd::value_error);
    BOOST_REQUIRE_THROW(sid_t fail_sid("01:02:03:4"), uhd::value_error);
    BOOST_REQUIRE_THROW(sid_t fail_sid("01:02:03:004"), uhd::value_error);
    BOOST_REQUIRE_THROW(sid_t fail_sid("1.2.3.0004"), uhd::value_error);
}