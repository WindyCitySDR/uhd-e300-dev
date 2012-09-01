/*
 * Copyright 2012 Ettus Research LLC
 */

/*
    ? STOP condition after writing address on read
    - Default buck/boost register values are OK
*/

#include "ltc3675.h"

//#include <stdio.h>
//#include <util/delay.h>

#include "io.h"
#include "i2c.h"
#include "debug.h"

//#define HARDWIRE_ENABLE	// Use hardware enable pins instead of I2C on regulators that support it

#ifdef ATTINY88_DIP

#ifdef HARDWIRE_ENABLE
static io_pin_t PWR_EN1     = IO_PC(7);	// Not routed by card
static io_pin_t PWR_EN2     = IO_PA(0);	// Not available on DIP
static io_pin_t PWR_EN3     = IO_PA(1);	// Not available on DIP
static io_pin_t PWR_EN4     = IO_PB(6);	// Instead of FTDI_BCD
static io_pin_t PWR_EN5     = IO_PB(7);	// Instead of FTDI_PWREN2
#endif // HARDWIRE_ENABLE

#else

#ifdef HARDWIRE_ENABLE
static io_pin_t PWR_EN1     = IO_PC(1);
//static io_pin_t PWR_EN2     = IO_PC(2);	// Now used by I2C for charge controller
//static io_pin_t PWR_EN3     = IO_PC(3);	// Now used by I2C for charge controller
static io_pin_t PWR_EN4     = IO_PA(1);
static io_pin_t PWR_EN5     = IO_PA(2);
#endif // HARDWIRE_ENABLE

#endif // ATTINY88_DIP

static io_pin_t PWR_SDA     = IO_PC(4);
static io_pin_t PWR_SCL     = IO_PC(5);

static io_pin_t PWR_IRQ     = IO_PD(0);
static io_pin_t WAKEUP      = IO_PD(2);
static io_pin_t ONSWITCH_DB = IO_PD(3);
static io_pin_t PWR_RESET   = IO_PD(4);

bool ltc3675_init(void)
{
#ifdef HARDWIRE_ENABLE
    io_output_pin(PWR_EN1);
    io_output_pin(PWR_EN2);
    io_output_pin(PWR_EN3);
    io_output_pin(PWR_EN4);
    io_output_pin(PWR_EN5);
#endif // HARDWIRE_ENABLE

 /*	io_output_pin(PWR_SDA);
    io_output_pin(PWR_SCL);

    // Must remain HIGH when idle
    io_set_pin(PWR_SDA);
    io_set_pin(PWR_SCL);
*/
	i2c_init(PWR_SDA, PWR_SCL);
    //i2c_init_ex(PWR_SDA, PWR_SCL, true);

    io_input_pin(PWR_IRQ);
	io_set_pin(PWR_IRQ);	// Enable pull-up for Open Drain
	
    io_input_pin(WAKEUP);
	io_set_pin(WAKEUP);	// Enable pull-up for Open Drain
	
    io_input_pin(ONSWITCH_DB);
	io_set_pin(ONSWITCH_DB);	// Enable pull-up for Open Drain
	
    io_input_pin(PWR_RESET);
	io_set_pin(PWR_RESET);	// Enable pull-up for Open Drain

    // FIXME: Make 'power_init' bool if any I2C

    return true;
}

#define LTC3675_BASE_ADDRESS    0x12
#define LTC3675_WRITE_ADDRESS   (LTC3675_BASE_ADDRESS + 0)
#define LTC3675_READ_ADDRESS    (LTC3675_BASE_ADDRESS + 1)

#define LTC3675_RETRY_DELAY     1   // us MAGIC
#define LTC3675_MAX_ACK_RETRIES 10  // * LTC3675_RETRY_DELAY us

#define LTC3675_SCL_LOW_PERIOD  2   // 1.3 us
#define LTC3675_SCL_HIGH_PERIOD 1   // 0.6 us
#define LTC3675_BUS_FREE_TIME   2   // 1.3 us
#define LTC3675_STOP_TIME       1   // 0.6 us

// Max I2C rate = 400kHz

bool ltc3675_enable_reg(ltc3675_regulator_t reg, bool on)
{
	//debug_blink2(reg + 1);
	
	// Sub-address: index of regulator
	// Data: <default reg contents> | <enable>
	
	const bool pull_up = false;
	
    switch (reg)
    {
        case LTC3675_REG_1: // Master
        case LTC3675_REG_2: // Slave
#ifdef HARDWIRE_ENABLE
            io_enable_pin(PWR_EN1, on);
			break;
#else
			return i2c_write_ex(PWR_SDA, PWR_SCL, LTC3675_WRITE_ADDRESS, 0x01, 0x6F | (on ? 0x80 : 0x00), pull_up);
#endif // HARDWIRE_ENABLE
        case LTC3675_REG_3: // Master
        case LTC3675_REG_4: // Slave
#ifdef HARDWIRE_ENABLE
            io_enable_pin(PWR_EN3, on);
            break;
#else
			return i2c_write_ex(PWR_SDA, PWR_SCL, LTC3675_WRITE_ADDRESS, 0x03, 0x6F | (on ? 0x80 : 0x00), pull_up);
#endif // HARDWIRE_ENABLE
        case LTC3675_REG_5: // I2C only
            return i2c_write_ex(PWR_SDA, PWR_SCL, LTC3675_WRITE_ADDRESS, 0x05, 0x0F | (on ? 0x80 : 0x00), pull_up);    // (Boost address, Default reg contents | Enable)
        case LTC3675_REG_6: // Single
#ifdef HARDWIRE_ENABLE
            io_enable_pin(PWR_EN5, on);
            break;
#else
			return i2c_write_ex(PWR_SDA, PWR_SCL, LTC3675_WRITE_ADDRESS, 0x06, 0x6F | (on ? 0x80 : 0x00), pull_up);
#endif // HARDWIRE_ENABLE
        default:
            return false;
    }

    return true;
}

bool ltc3675_set_voltage(ltc3675_regulator_t reg, uint16_t voltage)
{
    // Not necessary due to R-bridges and default DAC registers

    // VRAM will be 1.3579 - a little high? (re-program DAC reference)
    //  No: minimum FB step will put Vout < 1.35

    return true;
}

bool ltc3675_is_power_button_depressed(void)
{
	return (io_test_pin(ONSWITCH_DB) == false);
}
