/*
 * debug.c
 */ 

#include "config.h"
#include "debug.h"

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "io.h"
#include "power.h"

#define DEBUG_BLINK_DELAY	250	// ms

#ifdef DEBUG
static io_pin_t SERIAL_DEBUG      = IO_PD(6);
#else
//static io_pin_t SERIAL_DEBUG      = EN4;
#endif // DEBUG

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
	
	io_set_pin(SERIAL_DEBUG);
	io_output_pin(SERIAL_DEBUG);
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

#else

void debug_blink_rev(uint8_t count)
{
	charge_set_led(true);
	_delay_ms(DEBUG_BLINK_DELAY * 4);

	for (; count > 0; count--) {
		charge_set_led(false);
		_delay_ms(DEBUG_BLINK_DELAY);
		charge_set_led(true);
		_delay_ms(DEBUG_BLINK_DELAY * 2);
	}

	_delay_ms(DEBUG_BLINK_DELAY * 2);
	charge_set_led(false);
	_delay_ms(DEBUG_BLINK_DELAY * 4);
}

#endif // DEBUG

void debug_log(/*const */char* message)
{
	cli();
	
	const uint8_t max_buffer_length = 25;
	if (strlen(message) > (max_buffer_length - 1))
		message[max_buffer_length-1] = '\0';
	
	uint8_t buffer[max_buffer_length*10];	// START+8+STOP
	uint8_t idx = 0, i = 0;
	//buffer[i++] = 1;
	buffer[i++] = 0;	// START
	while (true)
	{
		if (*message == '\0')
		{
			buffer[i] = -1;
			break;
		}
		
		buffer[i++] = (((uint8_t)(*message) & ((uint8_t)1<<(/*7-*/(idx++)))) ? 0x01 : 0x00);
		
		if (idx == 8)
		{
			buffer[i++] = 1;	// STOP
			idx = 0;
			message++;
			if (*message != '\0')
				buffer[i++] = 0;	// START
		}
	}
	
	uint8_t time_fix = 0;
	const uint16_t delay = /*3333/2 - 10*/650-2;
	uint16_t countdown;
	
	for (uint8_t j = 0; j < i; ++j)
	{
		if (buffer[j])
			PORTD |= _BV(6);
		else
			PORTD &= ~_BV(6);
		
		countdown = delay;
		while (--countdown)
			__asm("nop");
	}
	
	io_set_pin(SERIAL_DEBUG);
	
	//sei();
}
