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

static io_pin_t PWR_EN1     = IO_PC(1);
static io_pin_t PWR_EN2     = IO_PC(2);
static io_pin_t PWR_EN3     = IO_PC(3);
static io_pin_t PWR_EN4     = IO_PA(1);
static io_pin_t PWR_EN5     = IO_PA(2);

static io_pin_t PWR_SDA     = IO_PC(4);
static io_pin_t PWR_SCL     = IO_PC(5);

static io_pin_t PWR_IRQ     = IO_PD(0);
static io_pin_t WAKEUP      = IO_PD(2);
static io_pin_t ONSWITCH_DB = IO_PD(3);
static io_pin_t PWR_RESET   = IO_PD(4);

bool ltc3675_init(void)
{
    io_output_pin(PWR_EN1);
    io_output_pin(PWR_EN2);
    io_output_pin(PWR_EN3);
    io_output_pin(PWR_EN4);
    io_output_pin(PWR_EN5);

 /*   io_output_pin(PWR_SDA);
    io_output_pin(PWR_SCL);

    // Must remain HIGH when idle
    io_set_pin(PWR_SDA);
    io_set_pin(PWR_SCL);
*/
    i2c_init(PWR_SDA, PWR_SCL);

    io_input_pin(PWR_IRQ);
    io_input_pin(WAKEUP);
    io_input_pin(ONSWITCH_DB);
    io_input_pin(PWR_RESET);

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
/*
static bool _ltc3675_write_byte(uint8_t value)
{
    // Assumes:
    //  io_output_pin(PWR_SDA);
    //  PWR_SCL is LOW

    for (uint8_t i = 0; i < 8; ++i)
    {
        io_enable_pin(PWR_SDA, (value & (1 << (7 - i))));   // MSB first

        io_set_pin(PWR_SCL);
        _delay_us(LTC3675_SCL_HIGH_PERIOD);

        io_clear_pin(PWR_SCL);
        _delay_us(LTC3675_SCL_LOW_PERIOD);
    }

    io_input_pin(PWR_SDA);

    _delay_us(LTC3675_SCL_HIGH_PERIOD);

    uint8_t retries = 0;
    while (io_test_pin(PWR_SDA))
    {
        if (retries == LTC3675_MAX_ACK_RETRIES) {
            io_output_pin(PWR_SDA);
            return false;
        }

        ++retries;
        _delay_us(LTC3675_RETRY_DELAY);
    }

    // Clock away acknowledge
    io_set_pin(PWR_SCL);
    _delay_us(LTC3675_SCL_HIGH_PERIOD);

    io_clear_pin(PWR_SCL);
    _delay_us(LTC3675_SCL_LOW_PERIOD);

    io_output_pin(PWR_SDA);

    io_clear_pin(PWD_SDA);

    return true;
}

static bool _ltc3675_read_byte(uint8_t* value)
{
    // Assumes:
    //  io_output_pin(PWR_SDA);
    //  PWR_SCL is LOW

    (*value) = 0x00;

    io_input_pin(PWD_SDA);

    for (uint8_t i = 0; i < 8; ++i)
    {
        io_set_pin(PWR_SCL);
        _delay_us(LTC3675_SCL_HIGH_PERIOD);

        (*value) |= ((io_test_pin(PWR_SDA) ? 0x1 : 0x0) << (7 - i));   // MSB first

        io_clear_pin(PWR_SCL);
        _delay_us(LTC3675_SCL_LOW_PERIOD);
    }

    // Not necessary to ACK since it's only this one byte

    io_output_pin(PWR_SDA);

    io_clear_pin(PWD_SDA);

    return true;
}

static void _ltc3675_i2c_start(void)
{
    // START condition
    io_clear_pin(PWR_SDA);
    _delay_us(LTC3675_SCL_LOW_PERIOD);  // Thd, sta

    io_clear_pin(PWR_SCL);
    _delay_us(LTC3675_SCL_LOW_PERIOD / 2);   // MAGIC
}

static void _ltc3675_i2c_stop(void)
{
    // STOP condition
    io_set_pin(PWR_SCL);
    _delay_us(LTC3675_STOP_TIME);

    io_set_pin(PWD_SDA);
    _delay_us(LTC3675_BUS_FREE_TIME);
}

static bool _ltc3675_write(uint8_t subaddr, uint8_t value)
{
    // Assumes:
    //  PWR_SCL is HIGH
    //  io_output_pin(PWR_SDA);

    _ltc3675_i2c_start();

    if (_ltc3675_write_byte(LTC3675_WRITE_ADDRESS) == false)
        return false;

    if (_ltc3675_write_byte(subaddr) == false)
        return false;

    if (_ltc3675_write_byte(value) == false)
        return false;

    _ltc3675_i2c_stop();

    return true;
}

static bool _ltc3675_read(uint8_t subaddr, uint8_t* value)
{
    // Assumes:
    //  PWR_SCL is HIGH
    //  io_output_pin(PWR_SDA);

    _ltc3675_i2c_start();

    if (_ltc3675_write_byte(LTC3675_READ_ADDRESS) == false)
        return false;

    if (_ltc3675_write_byte(subaddr) == false)
        return false;

    if (_ltc3675_read_byte(value) == false)
        return false;

    _ltc3675_i2c_stop();

    return true;
}
*/
bool ltc3675_enable_reg(ltc3675_regulator_t reg, bool on)
{
    switch (reg)
    {
        case LTC3675_REG_1: // Master
        case LTC3675_REG_2: // Slave
            io_enable_pin(PWR_EN1, on);
            break;
        case LTC3675_REG_3: // Master
        case LTC3675_REG_4: // Slave
            io_enable_pin(PWR_EN3, on);
            break;
        case LTC3675_REG_5: // I2C only
            return i2c_write(PWR_SDA, PWR_SCL, LTC3675_WRITE_ADDRESS, 0x05, 0x0F | (on ? 0x80 : 0x00));    // (Boost address, Default reg contents | Enable)
        case LTC3675_REG_6: // Single
            io_enable_pin(PWR_EN5, on);
            break;
        default:
            return false;
    }

    return true;
}

bool ltc3675_set_voltage(ltc3675_regulator_t reg, uint16_t voltage)
{
    // TODO: Not necessary due to R-bridges and default DAC registers

    // VRAM will be 1.3579 - a little high? (re-program DAC reference)
    //  No: minimum FB step will put Vout < 1.35

    return true;
}
