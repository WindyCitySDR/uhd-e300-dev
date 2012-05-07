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

#endif /* INCLUDED_B200_REGS_HPP */
