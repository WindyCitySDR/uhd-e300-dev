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

#ifndef INCLUDED_AD9361_TRANSACTION_H
#define INCLUDED_AD9361_TRANSACTION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AD9361_TRANSACTION_VERSION 0x1

#define AD9361_ACTION_ECHO 0
#define AD9361_ACTION_INIT 1
#define AD9361_ACTION_SET_RX_GAIN 2
#define AD9361_ACTION_SET_TX_GAIN 3
#define AD9361_ACTION_SET_RX_FREQ 4
#define AD9361_ACTION_SET_TX_FREQ 5
#define AD9361_ACTION_SET_CODEC_LOOP 6
#define AD9361_ACTION_SET_CLOCK_RATE 7
#define AD9361_ACTION_SET_ACTIVE_CHAINS 9

//! do the endianess conversion, define for your platform
#ifndef AD9361_TRANS_END32
    #define AD9361_TRANS_END32(x) (x)
#endif

inline void ad9361_trans_double_pack(const double input, uint32_t *output)
{
    const uint32_t *p = (const uint32_t *)&input;
    output[0] = AD9361_TRANS_END32(p[0]);
    output[1] = AD9361_TRANS_END32(p[1]);
}

inline void ad9361_trans_double_unpack(const uint32_t *input, double *output)
{
    uint32_t *p = (uint32_t *)&output;
    p[0] = AD9361_TRANS_END32(input[0]);
    p[1] = AD9361_TRANS_END32(input[1]);
}

typedef struct
{
    //version is expected to be AD9361_TRANSACTION_VERSION
    //check otherwise for compatibility
    uint32_t version;

    //sequence number - increment every call for sanity
    uint32_t sequence;

    //action tells us what to do, see AD9361_ACTION_*
    uint32_t action;

    union
    {
        //enable mask for chains
        uint32_t enable_mask;

        //true to enable codec internal loopback
        uint32_t codec_loop;

        //freq holds request LO freq and result from tune
        uint32_t freq[2];

        //gain holds request gain and result from action
        uint32_t gain[2];

        //rate holds request clock rate and result from action
        uint32_t rate[2];

    } value;

    //error message comes back as a reply -
    //set to null string for no error \0
    char error_msg[];

} ad9361_transaction_t;


#ifdef __cplusplus
}
#endif

#endif /* INCLUDED_AD9361_TRANSACTION_H */
