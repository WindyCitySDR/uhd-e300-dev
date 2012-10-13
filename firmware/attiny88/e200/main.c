/*
 * Copyright 2009 Ettus Research LLC
 */

#include "config.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

#include "global.h"
#include "power.h"
#include "debug.h"
#include "error.h"
#include "ltc3675.h"
#include "ltc4155.h"

#define INITIAL_DELAY	250	// ms

FUSES = {	// FIXME: & FUSE_CKSEL1 for low power 128 kHz clock
	.low = (FUSE_CKSEL0 & FUSE_SUT0 & FUSE_CKDIV8),	// Internal 8MHz Oscillator, Slowly rising power (start-up time), Divide Clock by 8
	.high = (FUSE_EESAVE & FUSE_SPIEN),	// Save EEPROM between flashes	// FIXME: Leave SPIEN for programming enabled?
};	// Not using watchdog as it runs during sleep and consumes power

volatile STATE _state;

/*
    - Main/shared variables must be volatile
	- Port pins are tri-stated on reset
	* AVR_IRQ PD(5)
	- Enable pull-ups on all O.D. outputs from regulator chip
	* PS_POR/SRST should be driven HIGH by ATTiny?
	- AVR_RESET -> RESET pin - don't configure fuse (this would disable this functionality and prohibit serial programming)
	* Ship-and-store mode for charge controller?
	* cli before I2C calls
	* PS_TX
	- en5-clk, en2-data
	* Instruction following SEI is executed before interrupts
	* LTC3675 real-time status doesn't contain UV/OT
	* LTC3675 PGOOD -> power down (no point in checking blink state)
	* On WALL, use TX, on battery use OTG switcher
	* PRR - Power Reduction Register (p40)
	* 100% -> 50% battery charge limit
*/

bool pmc_mask_irqs(bool mask)
{
	if (_state.interrupts_enabled == false)
		return false;
	
	if (mask)
		cli();
	else
		sei();
	
	return true;
}

int main(void)
{
	_delay_ms(INITIAL_DELAY);
	
	///////////////////////////////////////////////////////////////////////////
	
	memset((void*)&_state, 0x00, sizeof(STATE));
	
	debug_init();
	debug_blink(1);
	
	//debug_log("#");	// Will not boot if this is 21 chars long?!
	debug_log("Hello world");
	
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // SLEEP_MODE_PWR_SAVE combination is documented as Reserved

    // FIXME: Init as SPI slave (FPGA is master)
	
	// 8-bit timer for blinking errors on charge LED
	TCCR0A = _BV(CTC0);		// CTC mode
	OCR0A = 244;			// 250ms with 1024 prescale
	TIMSK0 = _BV(OCIE0A);	// Enable CTC on Timer 0

	bool init_result = power_init();
	debug_log_ex("Init", false);
	debug_log(init_result ? "+" : "-");
	debug_blink(2);
	//debug_blink_rev(6);
	
	///////////////////////////////////
	
	power_on(); // Turn on immediately. Need to de-press power button to turn off.
	debug_log("Power");
	debug_blink(3);
	//debug_blink_rev(10);
	
	//debug_wait();
	
	_state.interrupts_enabled = true;
	sei();	// Enable interrupts
	
	asm("nop");
	
	_state.wake_up = false;	// This will fire the first time the regs are turned on
	
	bool one_more = false;
	
	while (true)
	{
		one_more = false;
		
		if ((_state.ltc4155_irq)/* || ltc4155_has_interrupt()*/)	// [Don't know why PCINT ISR misses LTC4155 IRQ on power up, so double-check state of line]
		{
			ltc4155_handle_irq();
			
			_state.ltc4155_irq = false;
		}
		
		if (_state.core_power_bad)	// FIXME: Check whether it's supposed to be on
		{
			if (power_is_subsys_on(PS_FPGA))
			{
				_delay_ms(1);	// Seeing weird 120us drop in PGOOD during boot from flash (no apparent drop in 1.0V though)
				
				if (tps54478_is_power_good() == false)
				{
					debug_log("ML:FPGA!");
			
					//power_off();
					_state.power_off = true;
			
					/*while (_state.wake_up == false)
					{
						blink_error_sequence(1);
					}*/
					pmc_set_blink_error(BlinkError_FPGA_Power);	// [If a blink error was set in power_off, this will supercede it]
				}
			}			
			
			_state.core_power_bad = false;
		}
		
		if ((_state.ltc3675_irq)/* || ltc3675_has_interrupt()*/)	// This is fired on initial power up
		{
			debug_log("ML:3675+");
			
			ltc3675_handle_irq();
			
			if (ltc3675_is_power_good(ltc3675_get_last_status()) == false)
			{
				debug_log("ML:3675!");
				
				//power_off();
				_state.power_off = true;
			}
			
			_state.ltc3675_irq = false;
		}
		
		if (_state.power_off)
		{
			debug_log("ML:Off..");
			
			power_off();
			
			_state.power_off = false;
			_state.wake_up = false;
		}
		else if (_state.wake_up)
		{			
			//if (_state.powered == false)	// Don't check in case button is held long enough to force LTC3675 shutdown (will not change 'powered' value)
			{
				debug_log("ML:On..");
				
				power_on();
			}
			
			_state.wake_up = false;
		}
		
		// Check to see if the error state has resolved itself at the end of each sequence of the current blink error
		
		if ((_state.blink_error != BlinkError_None) && (_state.blink_last_loop != _state.blink_loops))
		{
			// [Check IRQs periodically]
			
			bool ltc3675_use_last_status = false;
			/*if (ltc3675_has_interrupt())
			{
				//debug_set(IO_PB(6), ((_state.blink_loops % 2) == 0));
				ltc3675_use_last_status = true;
				ltc3675_handle_irq();
			}*/
			
			///////////////////////////
			
			switch (_state.blink_error)
			{
				case BlinkError_LTC3675_UnderVoltage:
				case BlinkError_LTC3675_OverTemperature:
				case BlinkError_DRAM_Power:
				case BlinkError_3_3V_Peripherals_Power:
				case BlinkError_1_8V_Peripherals_Power:
				case BlinkError_TX_Power:
					if (((ltc3675_use_last_status) && (ltc3675_status_to_error(ltc3675_get_last_status()) != BlinkError_None)) || 
						((ltc3675_use_last_status == false) && (ltc3675_check_status() != BlinkError_None)))
						break;
					debug_log("BE:3675-");
					goto cancel_blink_error;
				case BlinkError_FPGA_Power:
					if (tps54478_is_power_good() == false)
						break;
					debug_log("BE:FPGA-");
					goto cancel_blink_error;
				default:
cancel_blink_error:				
					//debug_set(IO_PB(7), true);
					pmc_set_blink_error(BlinkError_None);
			}
			
			////////////////////////////////////
			
			// More periodic checks
			// Need to do this has some interrupts are on PCINT, and while GIE is disabled, might change & change back
			//	E.g. LTC3675 IRQ due to UV, reset IRQ, re-asserts UV
			
			if (ltc4155_has_interrupt())
			{
				debug_log("BE:4155");
				
				_state.ltc4155_irq = true;
				one_more = true;
			}
			
			if (ltc3675_has_interrupt())
			{
				debug_log("BE:3675");
				
				_state.ltc3675_irq = true;
				one_more = true;
			}
			
			if (tps54478_is_power_good() == false)
			{
				debug_log("BE:FPGA!");
				
				_state.core_power_bad = true;
				one_more = true;
			}
			
			////////////////////////////////////
			
			_state.blink_last_loop = _state.blink_loops;
		}
		
		//if (_state.timers_running == false)
		if ((_state.active_timers == 0) && (one_more == false))
		{
			debug_log("^");
			sleep_mode();
			debug_log("$");
		}			
	}

	return 0;
}

uint8_t pmc_get_blink_error(void)
{
	return _state.blink_error;
}

void pmc_set_blink_error(uint8_t count)
{
	if ((_state.blink_error != BlinkError_None)/* && (count > _state.blink_error)*/)	// [Prioritise]
		return;
	else if (_state.blink_error == count)	// Don't restart if the same
		return;
	
	if (count == BlinkError_None)
	{
		debug_log("BLNK-");
		_state.blink_stop = true;
		return;
	}
	
	//char msg[25];
	//sprintf(msg, "Blink code = %i\n", count);
	//debug_log(msg);
	debug_log_ex("BLNK ", false);
	debug_log_byte(count);
	
	_state.blink_error = count;
	_state.blink_loops = 0;
	_state.blink_last_loop = 0;
	_state.blinker_state = 0;
	_state.blink_stop = false;

	charge_set_led(false);
	
	TCNT0 = 0;
	if ((TCCR0A & 0x07) == 0x00)	// Might already be active with existing error
		_state.active_timers++;
	TCCR0A |= 0x05;	// Start with 1024 prescale
}

ISR(TIMER0_COMPA_vect)	// Blink the sequence, and leave one slot at the beginning and end where the LED is off so one can get a sense of how many blinks occurred
{
	if (_state.blinker_state < (2 * _state.blink_error + 1))
		charge_set_led((_state.blinker_state % 2) == 1);
	
	_state.blinker_state++;
	
	if (_state.blinker_state == (2 * _state.blink_error + 1 + 1))
	{
		_state.blinker_state = 0;
		
		if (_state.blink_stop)
		{
			if ((TCCR0A & 0x07) != 0x00)
				_state.active_timers--;
			TCCR0A &= ~0x07;
			
			_state.blink_error = BlinkError_None;
			
			debug_log("BLNK.");
		}
		else
		{
			_state.blink_loops++;
		}
	}		
}