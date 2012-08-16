#include "config.h"
#include "power.h"

#include <string.h>
#include <util/delay.h>
#include <avr/io.h>

#include "io.h"
#include "ltc3675.h"

#define BLINK_ERROR_DELAY   250  // ms

#define POWER_DEFAULT_DELAY     50  // ms
#define POWER_DEFAULT_RETRIES   10

#define ARRAY_SIZE(a)      (sizeof(a)/sizeof(a[0]))
#define ZERO_MEMORY(s)      memset(&s, 0x00, sizeof(s))

struct reg_config {
    int16_t voltage;    // mV
    uint8_t address;    // Device specific
    bool configured;    // Device specific
} default_reg_config[] = {        // Index maps to 'power_subsystem_t'
    { 0000, 0, true },          // PS_UNKNOWN
    { 1000, 0, true },          // PS_FPGA
    { 1350, LTC3675_REG_1 },    // PS_VDRAM
    { 1800, LTC3675_REG_3 },    // PS_PERIPHERALS_1_8
    { 3300, LTC3675_REG_6 },    // PS_PERIPHERALS_3_3
    { 5000, LTC3675_REG_5 }     // PS_TX
};

static io_pin_t USBPM_IRQ   = IO_PB(1);

static io_pin_t AVR_CS      = IO_PB(2);
static io_pin_t AVR_MOSI    = IO_PB(3);
static io_pin_t AVR_MISO    = IO_PB(4);
static io_pin_t AVR_SCK     = IO_PB(5);

static io_pin_t FTDI_BCD    = IO_PB(6);
static io_pin_t FTDI_PWREN2 = IO_PB(7);

static io_pin_t AVR_RESET   = IO_PC(6);
static io_pin_t AVR_IRQ     = IO_PD(5);

///////////////////////////////////////////////////////////////////////////////

static io_pin_t CORE_PWR_EN = IO_PA(3);
static io_pin_t CORE_PGOOD = IO_PB(0);

void tps54478_init(void)
{
    io_output_pin(CORE_PWR_EN);
    io_input_pin(CORE_PGOOD);
}

void tps54478_set_power(bool on)
{
    io_enable_pin(CORE_PWR_EN, on);
}

bool tps54478_is_power_good(void)
{
    return io_test_pin(CORE_PGOOD);
}

///////////////////////////////////////////////////////////////////////////////

static io_pin_t CHARGE      = IO_PD(1);

void charge_set_led(bool on)
{
    io_enable_pin(CHARGE, on);
}

///////////////////////////////////////////////////////////////////////////////

void power_signal_interrupt(void)
{
    // FIXME
}

///////////////////////////////////////////////////////////////////////////////

static io_pin_t PS_POR      = IO_PD(6);
static io_pin_t PS_SRST     = IO_PD(7);

#define FPGA_RESET_DELAY    10  // ms   // MAGIC

void fpga_reset(bool delay)
{
    io_clear_pin(PS_POR);
    io_clear_pin(PS_SRST);

    if (delay)
        _delay_ms(FPGA_RESET_DELAY);

    io_enable_pin(PS_POR, true);
    io_enable_pin(PS_SRST, true);
}

///////////////////////////////////////////////////////////////////////////////

static io_pin_t VBAT        = IO_PC(0);

void battery_init(void)
{
    //io_input_pin(VBAT);
    DIDR0 |= 0x1;           // Digital input disable PC0 (ADC0)

    ADMUX = (1 << REFS0)    // AVcc reference
          | (0 << ADLAR)    // Left-aligned result
          | (0 << MUX0);    // ADC0

    ADCSRA = (0x7 << ADPS0);// Prescale clock by 128
}

uint16_t battery_get_voltage(void)
{
    // Vout = (357k / (274k + 357k)) * Vbat
    // Vbat = (Vout * (274k + 357k)) / 357k

    // ADC = (Vin * 1024) / Vref
    // Vin = (ADC * Vref) / 1024
    // Vref = 3.3

    // Vbat(mV) = 1000 * (((ADC * 3.3) / 1024) * (274k + 357k)) / 357k
    // Vbat(mV) ~= ADC * 6 (=5.70)

    ADCSRA |= (1 << ADEN);        // FIXME: Turn on ADC (or leave on all the time)

    ADCSRA |= (1 << ADSC);  // Start conversion

    while (ADCSRA & (1 << ADSC));   // Wait for End of Conversion

    uint16_t voltage = (ADCH << 8) | (ADCL << 0);

    voltage *= 6;

    ADCSRA &= ~(1 << ADEN);         // FIXME: Turn off ADC (or leave on all the time?)

    return voltage;
}

///////////////////////////////////////////////////////////////////////////////

static void blink_error_sequence(uint8_t len)
{
    charge_set_led(false);
    _delay_ms(BLINK_ERROR_DELAY);

    for (; len > 0; len--) {
        charge_set_led(true);
        _delay_ms(BLINK_ERROR_DELAY);
        charge_set_led(false);
        _delay_ms(BLINK_ERROR_DELAY);
    }

    //for (len = 2; len > 0; len--)   // Could have *2 on delay, but so as never to overflow 8-bit argument
    //    _delay_ms(BLINK_ERROR_DELAY);
}

typedef struct power_params {
    power_subsystem_t subsys;
    bool enable;
    uint8_t retry;
    //uint16_t opaque;
} power_params_t;

static bool _power_up_fpga(power_params_t* params)
{
    if (params->subsys != PS_FPGA)
        return false;

    if (params->enable == false)
    {
        if (tps54478_is_power_good() == false)  // Already off
            return true;

        if (params->retry == 0)
            tps54478_set_power(false);

        return (tps54478_is_power_good() == false);
    }

    //bool fpga_power_good = tps54478_is_power_good();  // TODO: Can it ever already be good?

    if (params->retry == 0)
        tps54478_set_power(true);

    return tps54478_is_power_good();
}

static bool _power_up_reg(power_params_t* params)
{
    if ((params->subsys >= ARRAY_SIZE(default_reg_config)) || (params->subsys < 2))
        return false;

    struct reg_config* cfg = default_reg_config + params->subsys;

    if (params->enable == false) {
        return ltc3675_enable_reg(cfg->address, false);
    }

    if (ltc3675_set_voltage(cfg->address, cfg->voltage) == false)
        return false;

    return ltc3675_enable_reg(cfg->address, true);
}

static bool _power_enable_subsys(power_params_t* params)
{
    switch (params->subsys)
    {
        case PS_FPGA:
            return _power_up_fpga(params);
        //case PS_:
        //    break;
        default:
            return _power_up_reg(params);
    }

    return false;   // Should never get here
}

bool power_enable(power_subsystem_t subsys, bool on)
{
    power_params_t params;
    ZERO_MEMORY(params);
    params.subsys = subsys;
    params.enable = on;

    return _power_enable_subsys(&params);
}

typedef bool (*boot_function_t)(power_params_t*);

struct boot_step {
    power_subsystem_t subsys;
    boot_function_t fn;
    uint8_t delay;
    uint8_t retries;
    //uint16_t opaque;
} boot_steps[] = {  // MAGIC: Retries/delays
    { PS_FPGA,              NULL, POWER_DEFAULT_DELAY, POWER_DEFAULT_RETRIES },
    { PS_VDRAM,             NULL, POWER_DEFAULT_DELAY, POWER_DEFAULT_RETRIES },
    { PS_PERIPHERALS_1_8,   NULL, POWER_DEFAULT_DELAY, POWER_DEFAULT_RETRIES },
    { PS_PERIPHERALS_3_3,   NULL, POWER_DEFAULT_DELAY, POWER_DEFAULT_RETRIES },
    //{ PS_TX,              NULL, POWER_DEFAULT_DELAY, POWER_DEFAULT_RETRIES }  // CHECK: Leaving TX off
};

void power_init(void)
{
    tps54478_init();

    ltc3675_init();

    battery_init();

    io_output_pin(CHARGE);

    io_output_pin(PS_POR);
    io_output_pin(PS_SRST);

    // Hold low until power is stable
    io_clear_pin(PS_POR);
    io_clear_pin(PS_SRST);
/*
    USBPM_IRQ

    AVR_CS
    AVR_MOSI
    AVR_MISO
    AVR_SCK

    FTDI_BCD
    FTDI_PWREN2

    AVR_RESET

    AVR_IRQ
*/
}

bool power_on(void)
{
    charge_set_led(false);

	uint8_t step_count, retry;
	for (step_count = 0; step_count < ARRAY_SIZE(boot_steps); step_count++) {
	    struct boot_step* step = boot_steps + step_count;
	    if ((step->fn == NULL) && (step->subsys == PS_UNKNOWN))
            continue;

        power_params_t params;

	    for (retry = 0; retry <= step->retries; retry++) {
	        ZERO_MEMORY(params);
            params.subsys = step->subsys;
            params.enable = true;
	        params.retry = retry;

	        if (step->fn != NULL) {
                if (step->fn(&params))
                    break;
	        }
	        else {
	            if (_power_enable_subsys(&params))
                    break;
	        }

            if ((retry < step->retries) && (step->delay > 0))
                _delay_ms(step->delay);
	    }

	    if (retry == step->retries)
	        break;
    }

    if (step_count != ARRAY_SIZE(boot_steps)) {
        while (true) {
            blink_error_sequence(step_count);
        }

        return false;
    }

    return true;
}
