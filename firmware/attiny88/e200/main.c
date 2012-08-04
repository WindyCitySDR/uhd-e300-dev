/*
 * Copyright 2009 Ettus Research LLC
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <util/delay.h>

/*
    Main/shared variables must be volatile
*/

int main(void){
	/* initialize */
	power_init();

	power_on(); // Turn on immediately. Need to de-press power button to turn off.

	/* main loop */
	// Check CORE_PGOOD
	// Check battery_get_voltage
	// Interrupts:
	//  PWD_IRQ
	//  WAKEUP
	//  ONSWITCH_DB
	//  PWR_RESET
	//  AVR_CS
	//  AVR_RESET
	while (1);

	return 0;
}
