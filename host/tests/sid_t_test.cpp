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
#include <boost/test/unit_test.hpp>
#include <uhd/types/sid.hpp>

BOOST_AUTO_TEST_CASE(test_sid_t) {
    boost::uint32_t sid_value = 0x01020310;
    uhd::sid_t sid(sid_value);

    BOOST_CHECK_EQUAL(sid.is_set(), true);
    BOOST_CHECK_EQUAL(sid.to_pp_string(), "1.2.3.16");
    BOOST_CHECK_EQUAL(sid.to_pp_string_hex(), "01:02:03:10");
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
    std::cout
        << "Testing pretty-printing: "
        << sid.to_pp_string()
        << " /  "
        << sid.to_pp_string_hex()
        << std::endl;
    std::cout << "Testing ostream: " << sid << std::endl;

    uhd::sid_t empty_sid;
    BOOST_CHECK_EQUAL(empty_sid.is_set(), false);
    BOOST_CHECK_EQUAL(empty_sid.to_pp_string(), "x.x.x.x");
    BOOST_CHECK_EQUAL(empty_sid.to_pp_string_hex(), "xx:xx:xx:xx");
    BOOST_CHECK_EQUAL(empty_sid == sid, false);
    BOOST_CHECK_EQUAL(empty_sid == sid_value, false);
    BOOST_CHECK_EQUAL((bool) empty_sid, false);
    std::cout
        << "Testing pretty-printing: "
        << empty_sid.to_pp_string()
        << " /  "
        << empty_sid.to_pp_string_hex()
        << std::endl;
    empty_sid = sid_value; // No longer empty
    BOOST_CHECK_EQUAL(empty_sid.is_set(), true);
    BOOST_CHECK_EQUAL(empty_sid == sid, true);

}

BOOST_AUTO_TEST_CASE(test_sid_t_set) {
    boost::uint32_t sid_value = 0x0;
    uhd::sid_t sid(sid_value);

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
}
