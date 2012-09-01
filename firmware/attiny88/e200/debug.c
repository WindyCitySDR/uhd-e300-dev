/*
 * debug.c
 */ 

#include "config.h"
#include "debug.h"

#include <util/delay.h>

#include "io.h"

#define DEBUG_BLINK_DELAY	250	// ms

#ifdef DEBUG

#ifdef ATTINY88_DIP
static io_pin_t DEBUG_1 = IO_PB(6);
static io_pin_t DEBUG_2	= IO_PB(7);
#endif // ATTINY88_DIP

void debug_init()
{
	io_output_pin(DEBUG_1);
	io_output_pin(DEBUG_2);
	
	io_enable_pin(DEBUG_1, true);
	io_enable_pin(DEBUG_2, true);
}

void debug_set(io_pin_t pin, bool enable)
{
	io_enable_pin(pin, !enable);
}

void debug_blink(uint8_t count)
{
	io_enable_pin(DEBUG_1, false);
	io_enable_pin(DEBUG_2, true);
	_delay_ms(DEBUG_BLINK_DELAY * 2);

	for (; count > 0; count--) {
		io_enable_pin(DEBUG_2, false);
		_delay_ms(DEBUG_BLINK_DELAY);
		io_enable_pin(DEBUG_2, true);
		_delay_ms(DEBUG_BLINK_DELAY);
	}

	io_enable_pin(DEBUG_1, true);
	io_enable_pin(DEBUG_2, true);
	_delay_ms(DEBUG_BLINK_DELAY * 2);
}

void debug_blink_rev(uint8_t count)
{
	io_enable_pin(DEBUG_2, false);
	io_enable_pin(DEBUG_1, true);
	_delay_ms(DEBUG_BLINK_DELAY * 2);

	for (; count > 0; count--) {
		io_enable_pin(DEBUG_1, false);
		_delay_ms(DEBUG_BLINK_DELAY);
		io_enable_pin(DEBUG_1, true);
		_delay_ms(DEBUG_BLINK_DELAY);
	}

	io_enable_pin(DEBUG_2, true);
	io_enable_pin(DEBUG_1, true);
	_delay_ms(DEBUG_BLINK_DELAY * 2);
}

void debug_blink2(uint8_t count)
{
	io_enable_pin(DEBUG_1, true);
	io_enable_pin(DEBUG_2, true);
	_delay_ms(DEBUG_BLINK_DELAY * 2);

	bool b = false;
	for (; count > 0; count--) {
		io_enable_pin(DEBUG_1, b);
		io_enable_pin(DEBUG_2, b);
		_delay_ms(DEBUG_BLINK_DELAY);
		b = !b;
	}

	io_enable_pin(DEBUG_1, true);
	io_enable_pin(DEBUG_2, true);
	_delay_ms(DEBUG_BLINK_DELAY * 2);
}

void debug_wait(void)
{
	io_enable_pin(DEBUG_1, true);
	io_enable_pin(DEBUG_2, true);
	
	bool b = false;
	while (true)
	{
		io_enable_pin(DEBUG_1, b);
		io_enable_pin(DEBUG_2, !b);
		
		_delay_ms(DEBUG_BLINK_DELAY);
		
		b = !b;
	}
	
	io_enable_pin(DEBUG_1, true);
	io_enable_pin(DEBUG_2, true);
}

#endif // DEBUG
