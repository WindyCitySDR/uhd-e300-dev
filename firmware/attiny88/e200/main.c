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

#include "power.h"
#include "debug.h"

FUSES = {
	.low = (FUSE_CKSEL0 & FUSE_SUT0 & FUSE_CKDIV8),
	.high = (FUSE_EESAVE & FUSE_SPIEN),	// FIXME: SPIEN for programming?
};

/*
    - Main/shared variables must be volatile
	* AVR_IRQ PD(5)
*/

int main(void){
	debug_init();
	
	//debug_blink(1);
	
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // SLEEP_MODE_PWR_SAVE combination is documented as Reserved

    // FIXME: Init as SPI slave (FPGA is master)

	power_init();
	
	//debug_blink(2);

	power_on(); // Turn on immediately. Need to de-press power button to turn off.
	
	//debug_blink(3);

	fpga_reset(false);  // Power has been brought up, so let FPGA run

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

	// FIXME: Go to sleep, wait for interrupts
	
	debug_wait();

	while (true)
        sleep_mode();

	return 0;
}
