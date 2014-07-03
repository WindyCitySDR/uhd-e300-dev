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

#ifndef INCLUDED_E300_I2C_HPP
#define INCLUDED_E300_I2C_HPP

#include <boost/noncopyable.hpp>
#include <boost/cstdint.hpp>
#include <boost/shared_ptr.hpp>

#include <uhd/transport/zero_copy.hpp>

namespace uhd { namespace usrp { namespace e300 {

struct i2c_transaction_t {
    boost::uint8_t is_write;
    boost::uint8_t addr;
    boost::uint8_t reg;
    boost::uint8_t data;
};

class i2c : public boost::noncopyable
{
public:
    typedef boost::shared_ptr<i2c> sptr;

    static sptr make_i2cdev(const std::string &device);
    static sptr make_zc(uhd::transport::zero_copy_if::sptr xport);

    virtual boost::uint8_t get_i2c_reg(
        const boost::uint8_t addr,
        const boost::uint8_t reg) = 0;

    virtual void set_i2c_reg(
        const boost::uint8_t addr,
        const boost::uint8_t reg,
        const boost::uint8_t value) = 0;

    static const boost::uint8_t DB_EEPROM_ADDR = 0x50;
    static const boost::uint8_t MB_EEPROM_ADDR = 0x51;
};

}}};

#endif // INCLUDED_E300_I2C_HPP
