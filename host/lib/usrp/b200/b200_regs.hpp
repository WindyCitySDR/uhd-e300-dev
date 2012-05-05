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

#define localparam static const int

localparam SR_MISC         = 0;      // 5
localparam SR_USER_REGS    = 5;      // 2

localparam SR_TX_CTRL0     = 32;     // 6
localparam SR_TX_DSP0      = 40;     // 5
localparam SR_TX_FE0       = 48;     // 5

localparam SR_TX_CTRL1     = 64;      // 6
localparam SR_TX_DSP1      = 72;      // 5
localparam SR_TX_FE1       = 80;      // 5

localparam SR_RX_CTRL0     = 96;      // 9
localparam SR_RX_DSP0      = 106;     // 7
localparam SR_RX_FE0       = 116;     // 5

localparam SR_RX_CTRL1     = 128;     // 9
localparam SR_RX_DSP1      = 138;     // 7
localparam SR_RX_FE1       = 148;     // 5

localparam SR_TIME64       = 192;     // 6
localparam SR_SPI          = 208;     // 3
localparam SR_GPIO0        = 224;     // 5
localparam SR_GPIO1        = 232;     // 5

#define SR_RX_FRONTEND(which) (SR_RX_FE0 + which*32)*4
#define SR_TX_FRONTEND(which) (SR_TX_FE0 + which*32)*4

#define SR_RX_DSP(which) (SR_RX_DSP0 + which*32)*4
#define SR_TX_DSP(which) (SR_TX_DSP0 + which*32)*4

#define SR_RX_CTRL(which) (SR_RX_CTRL0 + which*32)*4
#define SR_TX_CTRL(which) (SR_TX_CTRL0 + which*32)*4

#define SR_RX_GPIO(which) (SR_GPIO0 + which*8)*4

#define SR_TIME64_CORE (SR_TIME64)*4
#define SR_SPI_CORE (SR_SPI)*4

#define REG_RB_TIME_NOW_HI 10
#define REG_RB_TIME_NOW_LO 11
#define REG_RB_TIME_PPS_HI 14
#define REG_RB_TIME_PPS_LO 15
#define REG_RB_SPI         0
#define REG_RB_COMPAT      1

#endif /* INCLUDED_B200_REGS_HPP */
