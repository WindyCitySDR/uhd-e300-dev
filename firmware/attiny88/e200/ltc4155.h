/*
 * ltc4155.h
 *
 * Created: 17/08/2012 8:09:43 PM
 *  Author: Balint Seeber
 */ 


#ifndef LTC4155_H_
#define LTC4155_H_

#include <stdbool.h>

bool ltc4155_init(void);
bool ltc4155_has_interrupt(void);
bool ltc4155_handle_irq(void);

#endif /* LTC4155_H_ */
