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
#include <boost/math/special_functions/round.hpp>
#include <vector>

using namespace uhd::convert;

typedef boost::uint32_t (*tohost32_type)(boost::uint32_t);

struct item32_sc12_3x
{
    item32_t line0;
    item32_t line1;
    item32_t line2;
};

template <typename type, tohost32_type towire>
void convert_star_4_to_sc12_item32_3
(
    std::complex<type> &in0,
    std::complex<type> &in1,
    std::complex<type> &in2,
    std::complex<type> &in3,
    const item32_sc12_3x &output,
    const double scalar
)
{
    const item32_t i0 = item32_t(type(in0.real()*scalar) & 0xfff);
    const item32_t q0 = item32_t(type(in0.imag()*scalar) & 0xfff);

    const item32_t i1 = item32_t(type(in1.real()*scalar) & 0xfff);
    const item32_t q1 = item32_t(type(in1.imag()*scalar) & 0xfff);

    const item32_t i2 = item32_t(type(in2.real()*scalar) & 0xfff);
    const item32_t q2 = item32_t(type(in2.imag()*scalar) & 0xfff);

    const item32_t i3 = item32_t(type(in3.real()*scalar) & 0xfff);
    const item32_t q3 = item32_t(type(in3.imag()*scalar) & 0xfff);

    const item32_t line0 = (i0 << 20) | (q0 << 8) | (i1 >> 4);
    const item32_t line1 = (i1 << 28) | (q1 << 16) | (i2 << 4) | (q2 >> 8);
    const item32_t line2 = (q2 << 24) | (i3 << 12) | (q3);

    output.line0 = towire(line0);
    output.line1 = towire(line1);
    output.line2 = towire(line2);
}

template <typename type, tohost32_type towire>
struct convert_star_1_to_sc12_item32_1 : public converter
{

    convert_star_1_to_sc12_item32_1(void)
    {
        //NOP
    }

    void set_scalar(const double scalar)
    {
        _scalar = scalar;
    }

    void operator()(const input_type &inputs, const output_type &outputs, const size_t nsamps)
    {
        const std::complex<type> *input = reinterpret_cast<const std::complex<type> *>(inputs[0]);
        item32_sc12_3x *output = reinterpret_cast<item32_sc12_3x *>(size_t(outputs[0]) & ~0x3);

        //helper variables
        size_t i = 0, o = 0;
        item32_sc12_3x dummy;

        //handle the head case
        const size_t head_samps = size_t(outputs[0]) & 0x3;
        switch (head_samps)
        {
        case 0: break; //no head
        case 1: convert_star_4_to_sc12_item32_3<type, towire>(0, 0, 0, input[0], dummy, _scalar); break;
        case 2: convert_star_4_to_sc12_item32_3<type, towire>(0, 0, input[0], input[1], dummy, _scalar); break;
        case 3: convert_star_4_to_sc12_item32_3<type, towire>(0, input[0], input[1], input[2], dummy, _scalar); break;
        }
        if (head_samps != 0) std::memcpy(outputs[o++], (reinterpret_cast<char *>(&dummy) + 12 - head_samps*3), head_samps*3);
        i += head_samps;

        //convert the body
        while (i+3 < nsamps)
        {
            convert_star_4_to_sc12_item32_3<type, towire>(input[i+0], input[i+1], input[i+2], input[i+3], output[o], _scalar);
            o++; i += 4;
        }

        //handle the tail case
        const size_t tail_samps = nsamps - i;
        switch (tail_samps)
        {
        case 0: break; //no tail
        case 1: convert_star_4_to_sc12_item32_3<type, towire>(input[i+0], 0, 0, 0, dummy, _scalar); break;
        case 2: convert_star_4_to_sc12_item32_3<type, towire>(input[i+0], input[i+1], 0, 0, dummy, _scalar); break;
        case 3: convert_star_4_to_sc12_item32_3<type, towire>(input[i+0], input[i+1], input[i+2], 0, dummy, _scalar); break;
        }
        if (tail_samps != 0) std::memcpy(outputs[o], &dummy, tail_samps*3);
    }

    double _scalar;
};
