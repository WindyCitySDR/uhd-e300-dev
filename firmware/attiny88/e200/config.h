#define __DELAY_BACKWARD_COMPATIBLE__   // Avoid compile-time arg error to '__builtin_avr_delay_cycles'
#define F_CPU   1000000UL // 1 MHz (8MHz / 8)

#define TARGET_ATTINY88_DIP // Re-map ports due to lack of PORTA
