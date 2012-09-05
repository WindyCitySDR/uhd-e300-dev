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

FUSES = {	// FIXME: & FUSE_CKSEL1 for low power 128 kHz clock
	.low = (FUSE_CKSEL0 & FUSE_SUT0 & FUSE_CKDIV8),	// Internal 8MHz Oscillator, Slowly rising power (start-up time), Divide Clock by 8
	.high = (FUSE_EESAVE & FUSE_SPIEN),	// Save EEPROM between flashes	// FIXME: Leave SPIEN for programming enabled?
};	// Not using watchdog as it runs during sleep and consumes power

volatile STATE _state;

/*
    - Main/shared variables must be volatile
	- Port pins are tri-stated on reset
	* AVR_IRQ PD(5)
	* Enable pull-ups on all O.D. outputs from regulator chip
	* PS_POR/SRST should be driven HIGH by ATTiny?
	- AVR_RESET -> RESET pin - don't configure fuse (this would disable this functionality and prohibit serial programming)
	* Ship-and-store mode for charge controller?
	* cli before I2C calls
	* PS_TX
	* en5-clk, en2-data
*/

int main(void)
{
	memset(&_state, 0x00, sizeof(STATE));
	
	debug_init();
	//debug_blink(1);
	
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // SLEEP_MODE_PWR_SAVE combination is documented as Reserved

    // FIXME: Init as SPI slave (FPGA is master)
	
	TCCR0A = _BV(CTC0);	// CTC mode
	OCR0A = 244;		// 250ms with 1024 prescale
	TIMSK0 = _BV(OCIE0A);	// Enable CTC on Timer 0

	power_init();
	//debug_blink(2);
	
	///////////////////////////////////

	power_on(); // Turn on immediately. Need to de-press power button to turn off.
	//debug_blink(3);

	/* main loop */
	// Check CORE_PGOOD
	// Check battery_get_voltage
	// Interrupts:
	//  PWD_IRQ
	//  WAKEUP
	//  ONSWITCH_DB
	//  -PWR_RESET
	//  -AVR_CS
	//  ?AVR_RESET
	
	//debug_wait();
	
	sei();	// Enable interrupts
	
	bool one_more = false;

	while (true)
	{
		one_more = false;
		
		if (_state.core_power_bad)
		{
			power_off();
			
			/*while (_state.wake_up == false)
			{
				blink_error_sequence(1);
			}*/
			pmc_set_blink_error(BlinkError_FPGA_Power);
			
			_state.core_power_bad = false;
		}
		
		if ((_state.ltc3675_irq)/* || (ltc3675_has_interrupt())*/)
		{
			ltc3675_handle_irq();
			
			if (ltc3675_is_power_good(ltc3675_get_last_status()) == false)
				power_off();
			
			_state.ltc3675_irq = false;
		}
		
		if (_state.power_off)
		{
			power_off();
			_state.power_off = false;
		}
		
		if (_state.wake_up)
		{
			power_on();
			_state.wake_up = false;
		}
		
		if ((_state.blink_error != BlinkError_None) && (_state.blink_last_loop != _state.blink_loops)/* && (_state.blinker_state == 0)*/)
		{
			// Check IRQs periodically
			
			bool ltc3675_use_last_status = false;
			if (ltc3675_has_interrupt())
			{
				//debug_set(IO_PB(6), ((_state.blink_loops % 2) == 0));
				ltc3675_use_last_status = true;
				ltc3675_handle_irq();
			}
			
			///////////////////////////
			
			switch (_state.blink_error)
			{
				case BlinkError_LTC3675_UnderVoltage:
				case BlinkError_LTC3675_OverTemperature:
				case BlinkError_DRAM_Power:
				case BlinkError_3_3V_Peripherals_Power:
				case BlinkError_1_8V_Peripherals_Power:
				case BlinkError_TX_Power:
					if (((ltc3675_use_last_status) && (ltc3675_status_to_error(ltc3675_get_last_status()) != 0)) || 
						((ltc3675_use_last_status == false) && (ltc3675_check_status() != 0)))
						break;
					goto cancel_blink_error;
				case BlinkError_FPGA_Power:
					if (tps54478_is_power_good() == false)
						break;
					goto cancel_blink_error;
				default:
cancel_blink_error:				
					//debug_set(IO_PB(7), true);
					pmc_set_blink_error(BlinkError_None);
			}
			
			////////////////////////////////////
			
			// More periodic checks
			
			if (tps54478_is_power_good() == false)
			{
				_state.core_power_bad = true;
				one_more = true;
			}				
				
			////////////////////////////////////
			
			_state.blink_last_loop = _state.blink_loops;
		}
		
		//if (_state.timers_running == false)
		if ((_state.active_timers == 0) && (one_more == false))
			sleep_mode();
	}

	return 0;
}

void pmc_set_blink_error(uint8_t count)
{
	if ((_state.blink_error != BlinkError_None) && (count > _state.blink_error))	// Prioritise
		return;
	
	charge_set_led(false);
	_state.blinker_state = 0;
	
	if (count == BlinkError_None)
	{
		_state.active_timers--;
		TCCR0A &= ~0x07;
	}
	else
	{
		_state.active_timers++;
		TCNT0 = 0;
		TCCR0A |= 0x05;	// 1024 prescale
	}
	
	_state.blink_error = count;
	_state.blink_loops = 0;
	_state.blink_last_loop = 0;
}

ISR(TIMER0_COMPA_vect)
{
	if (_state.blinker_state < (2 * _state.blink_error + 1))
		charge_set_led((_state.blinker_state % 2) == 1);
	
	_state.blinker_state++;
	
	if (_state.blinker_state == (2 * _state.blink_error + 1 + 1))
	{
		_state.blinker_state = 0;
		_state.blink_loops++;
	}		
}
