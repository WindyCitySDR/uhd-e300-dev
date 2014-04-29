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

#ifndef INCLUDED_AD9361_CHIP_H
#define INCLUDED_AD9361_CHIP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AD9361_GENERIC, AD9361_B200, AD9361_E300
} ad9361_product_t;

////////////////////////////////////////////////////////////
// shadow registers
typedef struct {
    uint8_t vcodivs;
    uint8_t inputsel;
    uint8_t rxfilt;
    uint8_t txfilt;
    uint8_t bbpll;
    uint8_t bbftune_config;
    uint8_t bbftune_mode;
} ad9361_chip_regs_t;

////////////////////////////////////////////////////////////
// other private data fields for VRQ handler
typedef struct {
    //Product
    ad9361_product_t    product;
    //Intermediate state
    double      rx_freq, tx_freq, req_rx_freq, req_tx_freq;
    double      baseband_bw, bbpll_freq, adcclock_freq;
    double      req_clock_rate, req_coreclk;
    uint16_t    rx_bbf_tunediv;
    uint8_t     curr_gain_table;
    uint32_t    rx1_gain, rx2_gain, tx1_gain, tx2_gain;
    int32_t     tfir_factor;
    //Register soft-copies
    ad9361_chip_regs_t regs;
    //Transact Interface
    void*       xact_iface;
} ad9361_device_t;

#ifdef __cplusplus
}
#endif

#endif /* INCLUDED_AD9361_CHIP_H */
