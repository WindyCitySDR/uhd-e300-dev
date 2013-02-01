//
// Copyright 2012-2013 Ettus Research LLC
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

#ifndef INCLUDED_B200_REGS_HPP
#define INCLUDED_B200_REGS_HPP

#include <boost/cstdint.hpp>

#define TOREG(x) ((x)*4)

#define localparam static const int

localparam SR_SPI       = 8;
localparam SR_ATR0      = 12;
localparam SR_ATR1      = 16;
localparam SR_LEDS      = 20;
localparam SR_TEST      = 21;
localparam SR_MISC_OUTS = 24;
localparam SR_READBACK  = 32;
localparam SR_TX_CTRL   = 64;
localparam SR_RX_CTRL   = 96;
localparam SR_TIME      = 128;

localparam RB32_TEST = 0;
localparam RB32_SPI = 4;
localparam RB64_TIME_NOW = 8;
localparam RB64_TIME_PPS = 16;

/* ATR Control Bits */
static const boost::uint32_t TX_ENABLE1 = (1 << 7);
static const boost::uint32_t SFDX1_RX = (1 << 6);
static const boost::uint32_t SFDX1_TX = (1 << 5);
static const boost::uint32_t SRX1_RX = (1 << 4);
static const boost::uint32_t SRX1_TX = (1 << 3);
static const boost::uint32_t LED_RX1 = (1 << 2);
static const boost::uint32_t LED_TXRX_RX1 = (1 << 1);
static const boost::uint32_t LED_TXRX_TX1 = (1 << 0);

static const boost::uint32_t TX_ENABLE2 = (1 << 7);
static const boost::uint32_t SFDX2_RX = (1 << 6);
static const boost::uint32_t SFDX2_TX = (1 << 5);
static const boost::uint32_t SRX2_RX = (1 << 4);
static const boost::uint32_t SRX2_TX = (1 << 3);
static const boost::uint32_t LED_RX2 = (1 << 2);
static const boost::uint32_t LED_TXRX_RX2 = (1 << 1);
static const boost::uint32_t LED_TXRX_TX2 = (1 << 0);


/* ATR State Definitions. */
/* Note that Side 2 is DIFFERENT than FDX on side one, on rev 2,
 * because of a couple of crossed switch control lines. */
static const boost::uint32_t STATE_OFF = 0x00;
static const boost::uint32_t STATE_FDX_TXRX1 = (SFDX1_RX
                                                | SFDX1_TX
                                                | LED_TXRX_TX2
                                                | LED_RX1);
static const boost::uint32_t STATE_FDX_TXRX2 = (SFDX2_RX
                                                | SRX2_TX
                                                | LED_TXRX_TX2
                                                | LED_RX2);

#endif /* INCLUDED_B200_REGS_HPP */
