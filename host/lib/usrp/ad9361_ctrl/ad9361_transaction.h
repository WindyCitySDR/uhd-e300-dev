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
#define AD9361_ACTION_SET_RX_GAIN 1
#define AD9361_ACTION_SET_TX_GAIN 2
#define AD9361_ACTION_SET_RX_FREQ 3
#define AD9361_ACTION_SET_TX_FREQ 4
#define AD9361_ACTION_SET_CLOCK_RATE 7
#define AD9361_ACTION_SET_ACTIVE_CHAINS 9

typedef struct
{
    //version is expected to be AD9361_TRANSACTION_VERSION
    //check otherwise for compatibility
    uint32_t version;

    //sequence number - increment every call for sanity
    uint32_t sequence;

    //action tells us what to do, see AD9361_ACTION_*
    uint32_t action;

    //enable mask for chains
    uint32_t enable_mask;

    //value holds rates, gains, freqs for request
    //and value holds the result actual value in reply
    double value;

    //error message comes back as a reply -
    //set to null string for no error \0
    char error_msg[40];

} ad9361_transaction_t;


#ifdef __cplusplus
}
#endif

#endif /* INCLUDED_AD9361_TRANSACTION_H */
