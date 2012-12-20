
// Copyright 2012 Ettus Research LLC
/*
 * Copyright 2007 Free Software Foundation, Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <wb_i2c.h>
#include <wb_utils.h>

#define REG_I2C_PRESCALER_LO 0
#define REG_I2C_PRESCALER_HI 4
#define REG_I2C_CTRL         8
#define REG_I2C_DATA         12
#define REG_I2C_CMD_ST       16

//
// STA, STO, RD, WR, and IACK bits are cleared automatically
//

#define	I2C_CTRL_EN	(1 << 7)	// core enable
#define	I2C_CTRL_IE	(1 << 6)	// interrupt enable

#define	I2C_CMD_START	(1 << 7)	// generate (repeated) start condition
#define I2C_CMD_STOP	(1 << 6)	// generate stop condition
#define	I2C_CMD_RD	(1 << 5)	// read from slave
#define I2C_CMD_WR	(1 << 4)	// write to slave
#define	I2C_CMD_NACK	(1 << 3)	// when a rcvr, send ACK (ACK=0) or NACK (ACK=1)
#define I2C_CMD_RSVD_2	(1 << 2)	// reserved
#define	I2C_CMD_RSVD_1	(1 << 1)	// reserved
#define I2C_CMD_IACK	(1 << 0)	// set to clear pending interrupt

#define I2C_ST_RXACK	(1 << 7)	// Received acknowledgement from slave (1 = NAK, 0 = ACK)
#define	I2C_ST_BUSY	(1 << 6)	// 1 after START signal detected; 0 after STOP signal detected
#define	I2C_ST_AL	(1 << 5)	// Arbitration lost.  1 when core lost arbitration
#define	I2C_ST_RSVD_4	(1 << 4)	// reserved
#define	I2C_ST_RSVD_3	(1 << 3)	// reserved
#define	I2C_ST_RSVD_2	(1 << 2)	// reserved
#define I2C_ST_TIP	(1 << 1)	// Transfer-in-progress
#define	I2C_ST_IP	(1 << 0)	// Interrupt pending

void wb_i2c_init(const uint32_t base, const size_t clk_rate)
{
    // prescaler divisor values for 100 kHz I2C [uses 5 * SCLK internally]
    const uint16_t prescaler = (clk_rate/(5 * 400000)) - 1;
    wb_poke32(base + REG_I2C_PRESCALER_LO, (prescaler >> 0) & 0xff);
    wb_poke32(base + REG_I2C_PRESCALER_HI, (prescaler >> 8) & 0xff);
    wb_poke32(base + REG_I2C_CTRL, I2C_CTRL_EN);
}

static inline void
wait_for_xfer(const uint32_t base)
{
  while (wb_peek32(base + REG_I2C_CMD_ST) & I2C_ST_TIP)	// wait for xfer to complete
    ;
}

static inline bool
wait_chk_ack(const uint32_t base)
{
  wait_for_xfer(base);

  if ((wb_peek32(base + REG_I2C_CMD_ST) & I2C_ST_RXACK) != 0){	// target NAK'd
    return false;
  }
  return true;
}

bool wb_i2c_read (const uint32_t base, const uint8_t i2c_addr, uint8_t *buf, size_t len)
{
  if (len == 0)			// reading zero bytes always works
    return true;

  while (wb_peek32(base + REG_I2C_CMD_ST) & I2C_ST_BUSY)
    ;

  wb_poke32(base + REG_I2C_DATA, (i2c_addr << 1) | 1);	 // 7 bit address and read bit (1)
  // generate START and write addr
  wb_poke32(base + REG_I2C_CMD_ST, I2C_CMD_WR | I2C_CMD_START);
  if (!wait_chk_ack(base))
    goto fail;

  for (; len > 0; buf++, len--){
    wb_poke32(base + REG_I2C_CMD_ST, I2C_CMD_RD | (len == 1 ? (I2C_CMD_NACK | I2C_CMD_STOP) : 0));
    wait_for_xfer(base);
    *buf = wb_peek32(base + REG_I2C_DATA);
  }
  return true;

 fail:
  wb_poke32(base + REG_I2C_CMD_ST, I2C_CMD_STOP);  // generate STOP
  return false;
}


bool wb_i2c_write(const uint32_t base, const uint8_t i2c_addr, const uint8_t *buf, size_t len)
{
  while (wb_peek32(base + REG_I2C_CMD_ST) & I2C_ST_BUSY)
    ;

  wb_poke32(base + REG_I2C_DATA, (i2c_addr << 1) | 0);	 // 7 bit address and write bit (0)

  // generate START and write addr (and maybe STOP)
  wb_poke32(base + REG_I2C_CMD_ST,  I2C_CMD_WR | I2C_CMD_START | (len == 0 ? I2C_CMD_STOP : 0));
  if (!wait_chk_ack(base))
    goto fail;

  for (; len > 0; buf++, len--){
    wb_poke32(base + REG_I2C_DATA, *buf);
    wb_poke32(base + REG_I2C_CMD_ST, I2C_CMD_WR | (len == 1 ? I2C_CMD_STOP : 0));
    if (!wait_chk_ack(base))
      goto fail;
  }
  return true;

 fail:
  wb_poke32(base + REG_I2C_CMD_ST, I2C_CMD_STOP);  // generate STOP
  return false;
}
