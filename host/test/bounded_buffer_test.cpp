//
// Copyright 2010 Ettus Research LLC
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

#include <boost/test/unit_test.hpp>
#include <uhd/transport/bounded_buffer.hpp>

//test #Include
#include <uhd/transport/alignment_buffer.hpp>

using namespace uhd::transport;

static const boost::posix_time::milliseconds timeout(10);

BOOST_AUTO_TEST_CASE(test_bounded_buffer){
    bounded_buffer<int>::sptr bb(new bounded_buffer<int>(3));

    //push elements, check for timeout
    BOOST_CHECK(bb->push_with_timed_wait(0, timeout));
    BOOST_CHECK(bb->push_with_timed_wait(1, timeout));
    BOOST_CHECK(bb->push_with_timed_wait(2, timeout));
    BOOST_CHECK(not bb->push_with_timed_wait(3, timeout));

    int val;
    //pop elements, check for timeout and check values
    BOOST_CHECK(bb->pop_with_timed_wait(val, timeout));
    BOOST_CHECK_EQUAL(val, 0);
    BOOST_CHECK(bb->pop_with_timed_wait(val, timeout));
    BOOST_CHECK_EQUAL(val, 1);
    BOOST_CHECK(bb->pop_with_timed_wait(val, timeout));
    BOOST_CHECK_EQUAL(val, 2);
    BOOST_CHECK(not bb->pop_with_timed_wait(val, timeout));
}