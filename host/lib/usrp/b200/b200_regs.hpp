//
// Copyright 2012 Ettus Research LLC
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

localparam SR_MISC         = 0;      // 5
localparam SR_USER_REGS    = 5;      // 2

localparam SR_TX_CTRL0     = 32;     // 6
localparam SR_TX_DSP0      = 40;     // 5

localparam SR_TX_CTRL1     = 64;      // 6
localparam SR_TX_DSP1      = 72;      // 5

localparam SR_RX_CTRL0     = 96;      // 9
localparam SR_RX_DSP0      = 106;     // 7

localparam SR_RX_CTRL1     = 128;     // 9
localparam SR_RX_DSP1      = 138;     // 7

localparam SR_TIME64       = 192;     // 6
localparam SR_SPI          = 208;     // 3
localparam SR_GPIO0        = 224;     // 5
localparam SR_GPIO1        = 232;     // 5

#define SR_RX_DSP(which) (SR_RX_DSP0 + which*32)
#define SR_TX_DSP(which) (SR_TX_DSP0 + which*32)

#define SR_RX_CTRL(which) (SR_RX_CTRL0 + which*32)
#define SR_TX_CTRL(which) (SR_TX_CTRL0 + which*32)

#define REG_RB_TIME_NOW_HI TOREG(10)
#define REG_RB_TIME_NOW_LO TOREG(11)
#define REG_RB_TIME_PPS_HI TOREG(14)
#define REG_RB_TIME_PPS_LO TOREG(15)
#define REG_RB_SPI         TOREG(0)
#define REG_RB_COMPAT      TOREG(1)

/* ATR GPIO TX Output Settings 
 * N.B. The LED_RX and LED_TXRX_RX names are switched in the schematic and the
 * FPGA nets. */
static const boost::uint32_t LED_TXRX_TX = (1 << 16);
static const boost::uint32_t LED_RX = (1 << 17);
static const boost::uint32_t LED_TXRX_RX = (1 << 18);
static const boost::uint32_t SRX_TX = (1 << 19);
static const boost::uint32_t SRX_RX = (1 << 20);
static const boost::uint32_t SFDX_TX = (1 << 21);
static const boost::uint32_t SFDX_RX = (1 << 22);
static const boost::uint32_t TX_ENABLE = (1 << 23);

static const boost::uint32_t STATE_OFF = 0x00;
static const boost::uint32_t STATE_TX = (LED_TXRX_TX | SFDX_TX | TX_ENABLE);
static const boost::uint32_t STATE_RX_ON_TXRX = (LED_TXRX_RX | SRX_TX | SRX_RX);
static const boost::uint32_t STATE_RX_ON_RX2 = (LED_RX | SFDX_RX);
static const boost::uint32_t STATE_FDX = (LED_TXRX_TX | LED_RX | SFDX_TX 
                                  | SFDX_RX | TX_ENABLE);

/* ATR GPIO RX Output Settings */
//FIXME -- What do these do?
static const boost::uint32_t CODEC_CTRL_IN = 0x0F;
static const boost::uint32_t CODEC_EN_AGC = (1 << 4);
static const boost::uint32_t CODEC_TXRX = (1 << 5);

#endif /* INCLUDED_B200_REGS_HPP */
