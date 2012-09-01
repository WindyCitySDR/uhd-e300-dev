/*
 * global.h
 *
 * Created: 31/08/2012 8:47:14 PM
 *  Author: Balint Seeber
 */ 

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <stdbool.h>

typedef struct State
{
	bool timers_running;
	bool powered;
	bool wake_up;
	bool power_off;
	bool core_power_bad;
} STATE;

//extern volatile bool _timers_running;
extern volatile STATE _state;

#endif /* GLOBAL_H_ */
