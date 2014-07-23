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

#include <boost/noncopyable.hpp>
#include <boost/cstdint.hpp>

#include <uhd/transport/zero_copy.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/utils/byteswap.hpp>

#ifndef INCLUDED_E300_SENSOR_MANAGER_HPP
#define INCLUDED_E300_SENSOR_MANAGER_HPP

namespace uhd { namespace usrp { namespace e300 {

struct sensor_transaction_t {
    boost::uint32_t which;
    union {
        boost::uint32_t value;
        char nmea[83];
        bool lock;
    };
};



enum temp_sensor {ZYNQ_TEMP=0, GPRMC=1,
                  GPGGA=2, NMEA=3, GPS_LOCK=4};

class e300_sensor_manager : boost::noncopyable
{
public:
    typedef boost::shared_ptr<e300_sensor_manager> sptr;
    virtual uhd::sensor_value_t get_mb_temp(void) = 0;

    static sptr make_proxy(uhd::transport::zero_copy_if::sptr xport);
    static sptr make_local(void);

    // Note: This is a hack
    static boost::uint32_t pack_float_in_uint32_t(const float &v)
    {
        const boost::uint32_t *cast = reinterpret_cast<const uint32_t*>(&v);
        return *cast;
    }

    static float unpack_float_from_uint32_t(const boost::uint32_t &v)
    {
        const float *cast = reinterpret_cast<const float*>(&v);
        return *cast;
    }
};


}}} // namespace

#endif // INCLUDED_E300_SENSOR_MANAGER_HPP
