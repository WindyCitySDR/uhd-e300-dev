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

#include "convert_common.hpp"
#include <uhd/utils/byteswap.hpp>

//convert 2 host streams into an interleaved item32 stream and vice-versa

#define __DECLARE_ITEM32_CONVERTER_2x(cpu_type, wire_type, xe, htoxx, xxtoh) \
    DECLARE_CONVERTER(cpu_type, 2, wire_type ## _item32_ ## xe, 1, PRIORITY_GENERAL){ \
        const cpu_type ## _t *input0 = reinterpret_cast<const cpu_type ## _t *>(inputs[0]); \
        const cpu_type ## _t *input1 = reinterpret_cast<const cpu_type ## _t *>(inputs[1]); \
        item32_t *output = reinterpret_cast<item32_t *>(outputs[0]); \
        for (size_t i = 0; i < nsamps; i++) { \
            output[2*i+0] = xx_to_item32_sc16_x1(input0[i], scale_factor); \
            output[2*i+1] = xx_to_item32_sc16_x1(input1[i], scale_factor); \
        } \
    } \
    DECLARE_CONVERTER(wire_type ## _item32_ ## xe, 1, cpu_type, 2, PRIORITY_GENERAL){ \
        const item32_t *input = reinterpret_cast<const item32_t *>(inputs[0]); \
        cpu_type ## _t *output0 = reinterpret_cast<cpu_type ## _t *>(outputs[0]); \
        cpu_type ## _t *output1 = reinterpret_cast<cpu_type ## _t *>(outputs[1]); \
        for (size_t i = 0; i < nsamps; i++) { \
            output0[i] = item32_sc16_x1_to_xx<cpu_type ## _t::value_type>(input[2*i+0], scale_factor); \
            output1[i] = item32_sc16_x1_to_xx<cpu_type ## _t::value_type>(input[2*i+1], scale_factor); \
        } \
    }

//all major host types on little endian device
__DECLARE_ITEM32_CONVERTER_2x(fc64, sc16, le, uhd::htowx, uhd::wtohx)
__DECLARE_ITEM32_CONVERTER_2x(fc32, sc16, le, uhd::htowx, uhd::wtohx)
__DECLARE_ITEM32_CONVERTER_2x(sc16, sc16, le, uhd::htowx, uhd::wtohx)

//all major host types on big endian device
__DECLARE_ITEM32_CONVERTER_2x(fc64, sc16, be, uhd::htonx, uhd::ntohx)
__DECLARE_ITEM32_CONVERTER_2x(fc32, sc16, be, uhd::htonx, uhd::ntohx)
__DECLARE_ITEM32_CONVERTER_2x(sc16, sc16, be, uhd::htonx, uhd::ntohx)
