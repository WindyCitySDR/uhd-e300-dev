
// Copyright 2012 Ettus Research LLC

#ifndef INCLUDED_B250_DEFS_H
#define INCLUDED_B250_DEFS_H

#define CPU_CLOCK 200000000
#define MAIN_RAM_BASE 0x0000
#define BOOT_RAM_BASE 0x4000
#define PKT_RAM0_BASE 0x8000
#define XGE0_BASE 0xC000
#define XGE1_BASE 0xD000
#define UART0_BASE 0xfd00
#define UART0_BAUD 115200
#define I2C0_BASE 0xfe00
#define I2C1_BASE 0xff00
#define SET0_BASE 0xa000
#define RB0_BASE 0xa000 //same as set

//#define ETH1G
#define ETH10G 

// Setting Regs Memeory Map
static const int SR_LEDS       = 0;
static const int SR_PHY_RST    = 1;
static const int SR_CLOCK_CTRL = 2;
static const int SR_XB_LOCAL   = 3;
static const int SR_SFPP_CTRL  = 4;
static const int SR_SPI        = 32;
static const int SR_ETHINT0    = 40;
static const int SR_ETHINT1    = 56;

// Readback Memory Map
static const int RB_COUNTER     = 0;
static const int RB_SPI_RDY     = 1;
static const int RB_SPI_DATA    = 2;
static const int RB_SFPP_STATUS = 3;


#endif /* INCLUDED_B250_DEFS_H */
