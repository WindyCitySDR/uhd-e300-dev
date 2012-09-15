/*
	* Test battery voltage code
	* Charger error blinking uses busy wait - will drain battery if encounters error while unattended? Surely rest of H/W will pull more current.
*/
#include "config.h"
#include "power.h"

#include <string.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "io.h"
#include "ltc3675.h"
#include "ltc4155.h"
#include "debug.h"
#include "global.h"
#include "error.h"

#define BLINK_ERROR_DELAY   250  // ms

#define POWER_DEFAULT_DELAY     50  // ms
#define POWER_DEFAULT_RETRIES   10

#define ARRAY_SIZE(a)      (sizeof(a)/sizeof(a[0]))
#define ZERO_MEMORY(s)      memset(&s, 0x00, sizeof(s))

#ifndef I2C_REWORK
io_pin_t PWR_SDA     = IO_PC(4);
io_pin_t PWR_SCL     = IO_PC(5);
#endif // I2C_REWORK

//volatile bool powered = false;

struct reg_config {
    int16_t voltage;    // mV
	uint8_t device;
    uint8_t address;    // Device specific
    bool powered;
} default_reg_config[] = {        // Index maps to 'power_subsystem_t'
    { 0000, REG_UNKNOWN, 0/*, true*/ },          // PS_UNKNOWN
    { 1000, REG_TPS54478, 0/*, true*/ },          // PS_FPGA
    { 1350, REG_LTC3675, LTC3675_REG_1 },    // PS_VDRAM
    { 1800, REG_LTC3675, LTC3675_REG_3 },    // PS_PERIPHERALS_1_8
    { 3300, REG_LTC3675, LTC3675_REG_6 },    // PS_PERIPHERALS_3_3
    { 5000, REG_LTC3675, LTC3675_REG_5 }     // PS_TX
};
/*
int8_t power_get_regulator_index(uint8_t device, uint8_t address)
{
	for (int8_t i = 0; i < ARRAY_SIZE(default_reg_config); ++i)
	{
		struct reg_config* reg = default_reg_config + i;
		if ((reg->device == device) && (reg->address == address))
			return i;
	}
	
	return -1;
}
*/
static bool ltc3675_reg_helper(uint8_t address)
{
	for (int8_t i = 0; i < ARRAY_SIZE(default_reg_config); ++i)
	{
		struct reg_config* reg = default_reg_config + i;
		if ((reg->device == REG_LTC3675) && (reg->address == address))
			return reg->powered;
	}
#ifdef DEBUG_SAFETY
	debug_log_ex("!3675HLP ", false);
	debug_log_hex(address);
#endif // DEBUG_SAFETY
	return false;
	//return power_is_subsys_on(power_get_regulator_index(REG_LTC3675, address) - 1);
}

static io_pin_t AVR_CS      = IO_PB(2);
static io_pin_t AVR_MOSI    = IO_PB(3);
static io_pin_t AVR_MISO    = IO_PB(4);
static io_pin_t AVR_SCK     = IO_PB(5);

#ifndef ATTINY88_DIP
static io_pin_t FTDI_BCD    = IO_PB(6);
static io_pin_t FTDI_PWREN2 = IO_PB(7);
#endif // ATTINY88_DIP

static io_pin_t AVR_RESET   = IO_PC(6);
static io_pin_t AVR_IRQ     = IO_PD(5);

///////////////////////////////////////////////////////////////////////////////

#define TPS54478_START_DELAY	3	// ms

#ifdef ATTINY88_DIP
static io_pin_t CORE_PWR_EN = IO_PC(1);	// IO_PC(7) not routed by card, using PWER_EN1 instead
#else
static io_pin_t CORE_PWR_EN = IO_PA(3);
#endif // ATTINY88_DIP
static io_pin_t CORE_PGOOD = IO_PB(0);

void tps54478_init(bool enable)
{
	tps54478_set_power(enable);
	io_clear_pin(CORE_PWR_EN);
	
    io_input_pin(CORE_PGOOD);
#ifndef DEBUG	// Don't enable pull-up when connected to a pulled-up switch
	io_set_pin(CORE_PGOOD);	// Enable pull-up for Open Drain
#endif // DEBUG
//#ifdef DEBUG
//	io_enable_pin(CORE_PWR_EN, false);
//#endif // DEBUG
//_delay_ms(2500);
}

void tps54478_set_power(bool on)
{
	debug_log_ex("54478", false);
	
	// Assumes: Hi-Z input/LOW output
	
	if (on)
	{
		io_input_pin(CORE_PWR_EN);
		_delay_ms(TPS54478_START_DELAY);
		
		debug_log("+");
	}		
	else
	{
		io_output_pin(CORE_PWR_EN);
		// Don't delay here as we can't detect its state anyway
		
		debug_log("-");
	}		
	
	//io_enable_pin(CORE_PWR_EN, on);
}

bool tps54478_is_power_good(void)
{
    return io_test_pin(CORE_PGOOD);	// This doesn't necessarily mean it's good - the chip might be malfunctioning (or switched off)
}

///////////////////////////////////////////////////////////////////////////////

static io_pin_t CHARGE      = IO_PD(1);

void charge_set_led(bool on)
{
#ifdef DEBUG
	on = !on;
#endif // DEBUG
    io_enable_pin(CHARGE, on);
}

///////////////////////////////////////////////////////////////////////////////

void power_signal_interrupt(void)
{
    io_set_pin(AVR_IRQ);	// FIXME: Active low?
}

///////////////////////////////////////////////////////////////////////////////

#ifndef DEBUG
static io_pin_t PS_POR      = IO_PD(6);
#define PS_POR_AVAILABLE
#endif // DEBUG
static io_pin_t PS_SRST     = IO_PD(7);

#define FPGA_RESET_DELAY    10  // ms   // MAGIC

void fpga_reset(bool delay)
{
#ifdef PS_POR_AVAILABLE
    io_clear_pin(PS_POR);
#endif // PS_POR_AVAILABLE
    io_clear_pin(PS_SRST);

    if (delay)
        _delay_ms(FPGA_RESET_DELAY);
#ifdef PS_POR_AVAILABLE
    io_enable_pin(PS_POR, true);
#endif // PS_POR_AVAILABLE
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

    ADCSRA |= (1 << ADEN);        // FIXME: Turn on ADC (or leave on all the time?)

    ADCSRA |= (1 << ADSC);  // Start conversion

    while (ADCSRA & (1 << ADSC));   // Wait for End of Conversion

    uint16_t voltage = (ADCH << 8) | (ADCL << 0);

    voltage *= 6;

    ADCSRA &= ~(1 << ADEN);         // FIXME: Turn off ADC (or leave on all the time?)

    return voltage;
}

///////////////////////////////////////////////////////////////////////////////

void blink_error_sequence(uint8_t len)
{
    charge_set_led(false);
    _delay_ms(BLINK_ERROR_DELAY * 2);

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
        //if (tps54478_is_power_good() == false)  // Already off
		//	return true;

        if (params->retry == 0)
		{
			io_clear_pin(PS_SRST);	// FIXME: Hold it low to stop
#ifdef PS_POR_AVAILABLE
			io_clear_pin(PS_POR);	// Prepare it for shutdown, and then the potential next power cycle
#endif // PS_POR_AVAILABLE			
            tps54478_set_power(false);
		}

        //return (tps54478_is_power_good() == false);
		return true;
    }

    //bool fpga_power_good = tps54478_is_power_good();  // TODO: Can it ever already be good?

    if (params->retry == 0)
        tps54478_set_power(true);

    return tps54478_is_power_good();
}

static bool _power_up_reg(power_params_t* params)
{
    if ((params->subsys > PS_TX) || (params->subsys < PS_VDRAM))
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
	//boot_function_t fn;
	//uint8_t delay;
	//uint8_t retries;
    //uint16_t opaque;
	//bool powered;
} boot_steps[] = {  // MAGIC: Retries/delays
    { PS_FPGA,              /*NULL, POWER_DEFAULT_DELAY, POWER_DEFAULT_RETRIES*/ },	// 7..8						// 3..4
    { PS_VDRAM,             /*NULL, POWER_DEFAULT_DELAY, POWER_DEFAULT_RETRIES*/ },	// 9..10					// 5..6
    { PS_PERIPHERALS_1_8,   /*NULL, POWER_DEFAULT_DELAY, POWER_DEFAULT_RETRIES*/ },	// 11..12					// 7..8
    { PS_PERIPHERALS_3_3,   /*NULL, POWER_DEFAULT_DELAY, POWER_DEFAULT_RETRIES*/ },	// 13..14					// 9..10
    //{ PS_TX,              /*NULL, POWER_DEFAULT_DELAY, POWER_DEFAULT_RETRIES*/ }  // CHECK: Leaving TX off
};
/*
bool power_is_subsys_on(int8_t index)
{
	if ((index < 0) || (index >= ARRAY_SIZE(boot_steps)))
		return false;
	
	struct boot_step* step = boot_steps + index;
	
	return step->powered;
}
*/
void power_init(void)
{
    io_output_pin(CHARGE);
	
	charge_set_led(true);

    tps54478_init(true);	// Will keep EN float (keep power on)
#ifndef I2C_REWORK
	i2c_init(PWR_SDA, PWR_SCL);
#endif // I2C_REWORK

	ltc4155_init();

    ltc3675_init(ltc3675_reg_helper);

    battery_init();
#ifdef PS_POR_AVAILABLE
	io_output_pin(PS_POR);
#endif // PS_POR_AVAILABLE
    io_output_pin(PS_SRST);
    // Hold low until power is stable
#ifdef PS_POR_AVAILABLE
    io_clear_pin(PS_POR);
#endif // PS_POR_AVAILABLE
    io_clear_pin(PS_SRST);
/*
    AVR_CS
    AVR_MOSI
    AVR_MISO
    AVR_SCK

    FTDI_BCD
    FTDI_PWREN2
*/
	io_input_pin(AVR_RESET);	// Has external pull-up (won't do anything because this is configured at the hardware RESET pin)
	
	//io_output_pin(AVR_IRQ);		// Output here, input to FPGA
    io_input_pin(AVR_IRQ);
	//io_set_pin(AVR_IRQ);	// FIXME: Active low?
	
	///////////////
	
	EICRA = _BV(ISC01) | _BV(ISC00) | _BV(ISC10)/* | _BV(ISC11)*/;	// Rising edge for INT0 (WAKEUP). [Falling for INT1.] Any logical change INT1 (ONSWITCH_DB)
	//EIMSK = _BV(INT0);	// [Turn on WAKEUP interrupt] Don't do this, as unit will turn on anyway
	EIMSK = _BV(INT1) | _BV(INT0);	// Turn on ONSWITCH_DB and WAKEUP
	
	PCMSK0 = _BV(PCINT1) | _BV(PCINT0);	// USBPM_IRQ | CORE_PGOOD
	PCMSK2 = _BV(PCINT16)/* | _BV(PCINT20)*/;	// PWR_IRQ/* | PWR_RESET*/
	PCICR = _BV(PCIE2) | _BV(PCIE0);

	///////////////
/*
	TCNT0;
	OCR0A = 0x;
	TCCR0A = _BV(CTC0);
	TIFR0;
	TIMSK0;
	TCCR0A |= 0x05;	// Switch on with 1024 prescaler
*/
	TCCR1B = _BV(WGM12);	// CTC mode
	OCR1A = 15624 * 2;		// Hold button for 2 seconds to switch off
	TIMSK1 = _BV(OCIE1A);	// Enable CTC on Timer 1
	
	charge_set_led(false);
}

bool power_on(void)
{
	pmc_mask_irqs(true);
	
    //charge_set_led(false);

	uint8_t step_count, retry;
	for (step_count = 0; step_count < ARRAY_SIZE(boot_steps); step_count++)
	{
//		debug_blink(step_count);
//		debug_blink(3 + (step_count * 2) + 0);
//		debug_blink_rev(7 + (step_count * 2) + 0);
		
	    struct boot_step* step = boot_steps + step_count;
	    if (/*(step->fn == NULL) && */(step->subsys == PS_UNKNOWN))
            continue;
		
		debug_log_ex("PWR ", false);
		debug_log_byte_ex(step->subsys, true);

        power_params_t params;

	    for (retry = 0; retry < /*step->retries*/POWER_DEFAULT_RETRIES; retry++)
		{
	        ZERO_MEMORY(params);
            params.subsys = step->subsys;
            params.enable = true;
	        params.retry = retry;

	        if ((/*(step->fn != NULL) && (step->fn(&params))) ||
				((step->fn == NULL) && */(_power_enable_subsys(&params))))
			{
				//step->powered = true;
				default_reg_config[step->subsys].powered = true;
				
				debug_log("+");
//				debug_blink(3 + (step_count * 2) + 1);
//				debug_blink_rev(7 + (step_count * 2) + 1);

                break;
	        }
			
			debug_log("?");

            if ((retry < /*step->retries*/POWER_DEFAULT_RETRIES)/* && (step->delay > 0)*/)
                _delay_ms(/*step->delay*/POWER_DEFAULT_DELAY);
	    }
		
//		debug_blink(step_count);

	    if (retry == /*step->retries*/POWER_DEFAULT_RETRIES)
	        break;
    }

    if (step_count != ARRAY_SIZE(boot_steps))
	{
		debug_log("x");
		
		//sei();	// For button press detection

        /*while (_state.powered == false) {
            blink_error_sequence(step_count + BlinkError_FPGA_Power);
        }*/
		pmc_set_blink_error(step_count + BlinkError_FPGA_Power);

		pmc_mask_irqs(false);

        return false;
    }
	
	///////////////////////////////////
	
	fpga_reset(false);  // Power has been brought up, so let FPGA run
	
	///////////////////////////////////
	
	// Turn off WAKEUP interrupt, enable ONSWITCH_DB
	//EIMSK = _BV(INT1);
	
	_state.powered = true;
//debug_blink_rev(1);
//_delay_ms(1000);	// Wait for FPGA PGOOD to stabilise
	pmc_mask_irqs(false);

    return true;
}

uint8_t power_off(void)
{
	pmc_mask_irqs(true);
	
	io_clear_pin(PS_SRST);	// FIXME: Hold it low to stop FPGA running
	
	///////////////////////////////////
	
	int8_t step_count, retry;
	for (step_count = ARRAY_SIZE(boot_steps) - 1; step_count >= 0; step_count--)
	{
//		debug_blink(step_count);
		
	    struct boot_step* step = boot_steps + step_count;
	    if (/*(step->fn == NULL) && */(step->subsys == PS_UNKNOWN))
            continue;

        power_params_t params;

	    for (retry = 0; retry < /*step->retries*/POWER_DEFAULT_RETRIES; retry++)
		{
	        ZERO_MEMORY(params);
            params.subsys = step->subsys;
            params.enable = false;
	        params.retry = retry;
			
			if ((/*(step->fn != NULL) && (step->fn(&params))) ||
				((step->fn == NULL) && */(_power_enable_subsys(&params))))
			{
				//step->powered = false;
				default_reg_config[step->subsys].powered = false;
				break;
			}
			
            if ((retry < /*step->retries*/POWER_DEFAULT_RETRIES)/* && (step->delay > 0)*/)
                _delay_ms(/*step->delay*/POWER_DEFAULT_DELAY);
	    }
		
//		debug_blink(step_count);

	    if (retry == /*step->retries*/POWER_DEFAULT_RETRIES)
	        break;
    }

    if (step_count != -1)
	{
		/*pmc_mask_irqs(false);
		
        while (_state.powered) {
            blink_error_sequence(step_count + BlinkError_FPGA_Power);
        }*/
		if (pmc_get_blink_error() == BlinkError_None)	// Only set blink error if no existing error
			pmc_set_blink_error(step_count + BlinkError_FPGA_Power);
		
		pmc_mask_irqs(false);

        return (step_count + 1);
    }
	
	///////////////////////////////////
	
	// Turn off WAKEUP interrupt, enable ONSWITCH_DB
	//EIMSK = _BV(INT1);

	_state.powered = false;
	
	pmc_mask_irqs(false);
	
	return 0;
}

///////////////////////////////////////////////////////////////////////////////

#ifdef DEBUG

#ifdef ATTINY88_DIP
static io_pin_t DEBUG_1 = IO_PB(6);
static io_pin_t DEBUG_2	= IO_PB(7);
#endif // ATTINY88_DIP

#endif // DEBUG

ISR(INT0_vect)	// PD(2) WAKEUP: Rising edge
{
	//cli();
	
	//power_on();
	debug_log("\nINT0\n");
	_state.wake_up = true;
	
	//sei();
}

ISR(INT1_vect)	// PD(3) ONSWITCH_DB (PB_STAT): Any change
{
	//cli();
	
	if (ltc3675_is_power_button_depressed())
	{
		TCNT1 = 0;
		if ((TCCR1B & 0x07) == 0x00)
			_state.active_timers++;
		TCCR1B |= /*0x5*/0x3;	// [1024] 64 prescaler
		//_state.timers_running = true;
		
		//debug_set(DEBUG_1, true);
		//debug_set(DEBUG_2, false);
	}
	else
	{
		//if (TIMSK1 & _BV(OCIE1A))	// If letting go of button and still running, stop timer
		{
			//TIMSK1 &= ~_BV(OCIE1A);
			if ((TCCR1B & 0x07) != 0x00)
				_state.active_timers--;
			TCCR1B &= ~0x7;	// Disable timer
			//_state.timers_running = false;
			
			//debug_set(DEBUG_1, false);
		}
	}
	
	//sei();
}

ISR(TIMER1_COMPA_vect)
{
	//cli();

	//TIMSK1 &= ~_BV(OCIE1A);	// Turn off timer
	TCCR1B &= ~0x7;	// Disable timer
	//_state.timers_running = false;
	_state.active_timers--;
	_state.power_off = true;
	
	//debug_set(DEBUG_2, true);
	
	//power_off();
	
	//sei();
	
	//sleep_mode();
}

ISR(PCINT0_vect)
{
	//cli();
	
	// CORE_PGOOD
	//	Assert low: power problem -> shutdown
	// USBPM_IRQ
	//	Charge status change? -> update LED
	//	Power problem:	battery -> blink charge LED
	//					major -> shutdown
	
	if (/*(_state.powered) && */(/*io_test_pin(CORE_PGOOD)*/tps54478_is_power_good() == false))
	{
		_state.core_power_bad = true;
	}
	
	if (ltc4155_has_interrupt())
	{
		_state.ltc4155_irq = true;
	}
	
	//sei();
}

ISR(PCINT2_vect)
{
	//cli();
	
	// PWR_IRQ
	//	Regulator problem: shutdown
	// PWR_RESET
	//	Ignored
	
	if (ltc3675_has_interrupt())
	{
		//debug_set(IO_PB(6), true);
		_state.ltc3675_irq = true;
	}
	
	//sei();
}
