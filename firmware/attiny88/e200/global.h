/*
 * global.h
 *
 * Created: 31/08/2012 8:47:14 PM
 *  Author: Balint Seeber
 */ 

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct State
{
	//bool timers_running;
	uint8_t active_timers;
	bool powered;
	bool wake_up;
	bool power_off;
	bool core_power_bad;
	bool ltc3675_irq;
	//bool low_battery;
	uint8_t blink_error;
	uint8_t blinker_state;
	uint8_t blink_loops;
	uint8_t blink_last_loop;
} STATE;

//extern volatile bool _timers_running;
extern volatile STATE _state;

void pmc_set_blink_error(uint8_t count);

#endif /* GLOBAL_H_ */
