
// Copyright 2012 Ettus Research LLC

#ifndef INCLUDED_B250_DEFS_H
#define INCLUDED_B250_DEFS_H

#define CPU_CLOCK 125000000
#define MAIN_RAM_BASE 0x0000
#define BOOT_RAM_BASE 0x4000
#define PKT_RAM0_BASE 0x8000
#define UART0_BASE 0xfd00
#define UART0_BAUD 115200
#define I2C0_BASE 0xfe00
#define I2C1_BASE 0xff00
#define SET0_BASE 0xa000
#define RB0_BASE 0xa000 //same as set

#define SR_PHY_RST ((SET0_BASE) + 1*4)

#endif /* INCLUDED_B250_DEFS_H */
