/*
 * debug.h
 */ 

#ifndef DEBUG_H_
#define DEBUG_H_

#include <stdint.h>
#include <stdbool.h>

#include "io.h"

#ifdef DEBUG
#define DEBUG_NOOP	;
#define LED_ON		false
#define LED_OFF		true
#else
#define DEBUG_NOOP	{}
#define LED_ON		true
#define LED_OFF		false
#endif // DEBUG

inline void debug_init(void) DEBUG_NOOP
inline void debug_set(io_pin_t pin, bool enable);
inline void debug_blink(uint8_t count) DEBUG_NOOP
inline void debug_blink_rev(uint8_t count) DEBUG_NOOP
inline void debug_blink2(uint8_t count) DEBUG_NOOP
inline void debug_wait(void) DEBUG_NOOP

#endif /* DEBUG_H_ */
