/*
 * debug.h
 */ 

#ifndef DEBUG_H_
#define DEBUG_H_

#include <stdint.h>

#ifdef DEBUG
#define DEBUG_NOOP	;
#define LED_ON		false
#define LED_OFF		true
#else
#define DEBUG_NOOP	{}
#define LED_ON		true
#define LED_OFF		false
#endif // DEBUG

void debug_init(void) DEBUG_NOOP
void debug_blink(uint8_t count) DEBUG_NOOP
void debug_blink2(uint8_t count) DEBUG_NOOP
void debug_wait(void) DEBUG_NOOP

#endif /* DEBUG_H_ */
