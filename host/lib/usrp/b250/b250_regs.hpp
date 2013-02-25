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

#ifndef INCLUDED_B250_REGS_HPP
#define INCLUDED_B250_REGS_HPP

#include <boost/cstdint.hpp>

#define TOREG(x) ((x)*4)

#define localparam static const int

localparam SR_TEST      = 7;
localparam SR_SPI       = 8;
localparam SR_GPIO      = 16;
localparam SR_LEDS      = 20;
localparam SR_MISC_OUTS = 24;
localparam SR_READBACK  = 32;
localparam SR_TX_CTRL   = 64;
localparam SR_RX_CTRL   = 96;

localparam RB32_GPIO            = 0;
localparam RB32_SPI             = 4;
localparam RB64_TIME_NOW        = 8;
localparam RB64_TIME_PPS        = 16;
localparam RB32_TEST            = 24;

//wishbone settings map - relevant to host code
#define SET0_BASE 0xa000
#define SR_ADDR(base, offset) ((base) + (offset)*4)

localparam ZPU_SR_LEDS       = 00;
localparam ZPU_SR_PHY_RST    = 01;
localparam ZPU_SR_CLOCK_CTRL = 02;
localparam ZPU_SR_XB_LOCAL   = 03;
localparam ZPU_SR_SPI        = 32;
localparam ZPU_SR_ETHINT0    = 40;
localparam ZPU_SR_ETHINT1    = 56;

localparam ZPU_RB_SPI = 2;

#endif /* INCLUDED_B250_REGS_HPP */
