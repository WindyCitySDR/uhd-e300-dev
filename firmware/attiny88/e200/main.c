/*
 * Copyright 2009 Ettus Research LLC
 */

#include "config.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "power.h"

FUSES = {
	.low = (FUSE_CKSEL0 & FUSE_SUT0),
	.high = (FUSE_EESAVE/* & FUSE_SPIEN*/),
};

/*
    Main/shared variables must be volatile
*/

int main(void){
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // SLEEP_MODE_PWR_SAVE combination is documented as Reserved

    // FIXME: Init as SPI slave (FPGA is master)

	power_init();

	power_on(); // Turn on immediately. Need to de-press power button to turn off.

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

	while (true)
        sleep_mode();

	return 0;
}
