#include "config.h"
#include "i2c.h"

#include <util/delay.h>

#include "io.h"

#define I2C_DEFAULT_RETRY_DELAY     1   // us MAGIC
#define I2C_DEFAULT_MAX_ACK_RETRIES 10  // * I2C_DEFAULT_RETRY_DELAY us

#define I2C_DEFAULT_SCL_LOW_PERIOD  2   // 1.3 us
#define I2C_DEFAULT_SCL_HIGH_PERIOD 1   // 0.6 us
#define I2C_DEFAULT_BUS_FREE_TIME   2   // 1.3 us
#define I2C_DEFAULT_STOP_TIME       1   // 0.6 us

static bool _i2c_write_byte(io_pin_t sda, io_pin_t scl, uint8_t value)
{
    // Assumes:
    //  io_output_pin(sda);
    //  scl is LOW

    for (uint8_t i = 0; i < 8; ++i)
    {
        io_enable_pin(sda, (value & (1 << (7 - i))));   // MSB first

        io_set_pin(scl);
        _delay_us(I2C_DEFAULT_SCL_HIGH_PERIOD);

        io_clear_pin(scl);
        _delay_us(I2C_DEFAULT_SCL_LOW_PERIOD);
    }

    io_input_pin(sda);

    _delay_us(I2C_DEFAULT_SCL_HIGH_PERIOD);

    uint8_t retries = 0;
    while (io_test_pin(sda))
    {
        if (retries == I2C_DEFAULT_MAX_ACK_RETRIES) {
            io_output_pin(sda);
            return false;
        }

        ++retries;
        _delay_us(I2C_DEFAULT_RETRY_DELAY);
    }

    // Clock away acknowledge
    io_set_pin(scl);
    _delay_us(I2C_DEFAULT_SCL_HIGH_PERIOD);

    io_clear_pin(scl);
    _delay_us(I2C_DEFAULT_SCL_LOW_PERIOD);

    io_output_pin(sda);

    io_clear_pin(sda);

    return true;
}

static bool _i2c_read_byte(io_pin_t sda, io_pin_t scl, uint8_t* value)
{
    // Assumes:
    //  io_output_pin(sda);
    //  scl is LOW

    (*value) = 0x00;

    io_input_pin(sda);

    for (uint8_t i = 0; i < 8; ++i)
    {
        io_set_pin(scl);
        _delay_us(I2C_DEFAULT_SCL_HIGH_PERIOD);

        (*value) |= ((io_test_pin(sda) ? 0x1 : 0x0) << (7 - i));   // MSB first

        io_clear_pin(scl);
        _delay_us(I2C_DEFAULT_SCL_LOW_PERIOD);
    }

    // Not necessary to ACK since it's only this one byte

    io_output_pin(sda);

    io_clear_pin(sda);

    return true;
}

static void _i2c_start(io_pin_t sda, io_pin_t scl)
{
    // START condition
    io_clear_pin(sda);
    _delay_us(I2C_DEFAULT_SCL_LOW_PERIOD);  // Thd, sta

    io_clear_pin(sda);
    _delay_us(I2C_DEFAULT_SCL_LOW_PERIOD / 2);   // MAGIC
}

static void _i2c_stop(io_pin_t sda, io_pin_t scl)
{
    // STOP condition
    io_set_pin(scl);
    _delay_us(I2C_DEFAULT_STOP_TIME);

    io_set_pin(sda);
    _delay_us(I2C_DEFAULT_BUS_FREE_TIME);
}

bool i2c_write(io_pin_t sda, io_pin_t scl, uint8_t addr, uint8_t subaddr, uint8_t value)
{
    // Assumes:
    //  SCL is HIGH
    //  io_output_pin(SDA);

    _i2c_start(sda, scl);

    if (_i2c_write_byte(sda, scl, addr) == false)
        return false;

    if (_i2c_write_byte(sda, scl, subaddr) == false)
        return false;

    if (_i2c_write_byte(sda, scl, value) == false)
        return false;

    _i2c_stop(sda, scl);

    return true;
}

bool i2c_read(io_pin_t sda, io_pin_t scl, uint8_t addr, uint8_t subaddr, uint8_t* value)
{
    // Assumes:
    //  SCL is HIGH
    //  io_output_pin(SDA);

    _i2c_start(sda, scl);

    if (_i2c_write_byte(sda, scl, addr) == false)
        return false;

    if (_i2c_write_byte(sda, scl, subaddr) == false)
        return false;

    if (_i2c_read_byte(sda, scl, value) == false)
        return false;

    _i2c_stop(sda, scl);

    return true;
}

void i2c_init(io_pin_t sda, io_pin_t scl)
{
    io_output_pin(sda);
    io_output_pin(scl);

    // Must remain HIGH when idle
    io_set_pin(sda);
    io_set_pin(scl);
}
