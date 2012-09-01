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
*/

int main(void)
{
	memset(&_state, 0x00, sizeof(STATE));
	
	debug_init();
	//debug_blink(1);
	
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // SLEEP_MODE_PWR_SAVE combination is documented as Reserved

    // FIXME: Init as SPI slave (FPGA is master)

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

	while (true)
	{
		if (_state.timers_running == false)
			sleep_mode();
			
		if (_state.core_power_bad)
		{
			power_off();
			
			while (_state.wake_up == false)
			{
				blink_error_sequence(1);
			}
			
			_state.core_power_bad = false;
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
	}		

	return 0;
}
