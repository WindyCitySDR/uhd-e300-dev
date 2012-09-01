/*
 * ltc4155.c
 */ 

#include "config.h"
#include "ltc4155.h"

#include "i2c.h"

static io_pin_t CHRG_SDA     = IO_PC(2);
static io_pin_t CHRG_SCL     = IO_PC(3);

void ltc4155_init(void)
{
	i2c_init_ex(CHRG_SDA, CHRG_SCL, true);	// FIXME: Disable pull-ups once board is re-worked
}
