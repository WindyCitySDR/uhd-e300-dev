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
localparam SR_MISC_OUTS = 24;
localparam SR_READBACK  = 32;
localparam SR_TX_CTRL   = 64;
localparam SR_RX_CTRL   = 96;
localparam SR_TIME      = 128;
localparam SR_RX_DSP    = 144;
localparam SR_TX_DSP    = 184;
localparam SR_LEDS      = 196;
localparam SR_FP_GPIO   = 200;

localparam RB32_GPIO            = 0;
localparam RB32_SPI             = 4;
localparam RB64_TIME_NOW        = 8;
localparam RB64_TIME_PPS        = 16;
localparam RB32_TEST            = 24;
localparam RB32_RX              = 28;

localparam BL_ADDRESS     = 0;
localparam BL_DATA        = 1;

//wishbone settings map - relevant to host code
#define SET0_BASE 0xa000
#define SETXB_BASE 0xb000
#define BOOT_LDR_BASE 0xFA00
#define I2C0_BASE 0xfe00
#define I2C1_BASE 0xff00
#define SR_ADDR(base, offset) ((base) + (offset)*4)

localparam ZPU_SR_LEDS       = 00;
localparam ZPU_SR_PHY_RST    = 01;
localparam ZPU_SR_CLOCK_CTRL = 02;
localparam ZPU_SR_XB_LOCAL   = 03;
localparam ZPU_SR_SPI        = 32;
localparam ZPU_SR_ETHINT0    = 40;
localparam ZPU_SR_ETHINT1    = 56;

localparam ZPU_RB_SPI = 2;
localparam ZPU_RB_CLK_STATUS = 3;

//spi slaves on radio
#define DB_DAC_SEN (1 << 7)
#define DB_ADC_SEN (1 << 6)
#define DB_RX_LSADC_SEN (1 << 5)
#define DB_RX_LSDAC_SEN (1 << 4)
#define DB_TX_LSADC_SEN (1 << 3)
#define DB_TX_LSDAC_SEN (1 << 2)
#define DB_RX_SEN (1 << 1)
#define DB_TX_SEN (1 << 0)

#endif /* INCLUDED_B250_REGS_HPP */
