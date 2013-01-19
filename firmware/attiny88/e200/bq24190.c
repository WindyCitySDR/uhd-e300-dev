/*
 * bq24190.c
 *
 * Created: 11/12/2012 4:58:12 PM
 *  Author: Balint Seeber
 */

#ifdef CHARGER_TI

#include "config.h"
#include "bq24190.h"

#include <util/delay.h>

#include "io.h"
#include "i2c.h"
#include "power.h"
#include "debug.h"
#include "global.h"
#include "error.h"

static io_pin_t USBPM_IRQ	= IO_PB(1);

#ifdef ATTINY88_DIP

static io_pin_t CHRG_SDA     = IO_PC(2);
static io_pin_t CHRG_SCL     = IO_PC(3);

#else

#ifdef I2C_REWORK

static io_pin_t CHRG_SDA     = IO_PC(4);
static io_pin_t CHRG_SCL     = IO_PC(5);

#else

#define CHRG_SDA	PWR_SDA
#define CHRG_SCL	PWR_SCL

#endif // I2C_REWORK

#endif // ATTINY88_DIP

const bool _bq24190_pull_up = false;

#define BQ24190_BASE_ADDRESS    0x6B
#define BQ24190_WRITE_ADDRESS   (BQ24190_BASE_ADDRESS + 0)
#define BQ24190_READ_ADDRESS    (BQ24190_BASE_ADDRESS + 1)

enum BQ24190Registers
{
	BQ24190_REG_INPUT_SOURCE_CTL= 0,
	BQ24190_REG_PWR_ON_CONFIG	= 1,
	BQ24190_REG_TIMER_CONTROL	= 5
};
/*
enum BQ24190TimerControl
{
	
};
*/
enum BQ24190Shifts
{
	BQ24190_SHIFTS_CHARGER_CONFIG	= 4,
	BQ24190_SHIFTS_I2C_WATCHDOG		= 4
};

bool _bq24190_toggle_charger(bool on)
{
	uint8_t config = 0;
	if (i2c_read2_ex(CHRG_SDA, CHRG_SCL, BQ24190_READ_ADDRESS, BQ24190_REG_PWR_ON_CONFIG, &config, _bq24190_pull_up) == false)
		return false;
	
	config &= ~(0x3 << BQ24190_SHIFTS_CHARGER_CONFIG);
	if (on)
		config |= (0x01 << BQ24190_SHIFTS_CHARGER_CONFIG);	// Enable charger
	
	if (i2c_write_ex(CHRG_SDA, CHRG_SCL, BQ24190_WRITE_ADDRESS, BQ24190_REG_PWR_ON_CONFIG, config, _bq24190_pull_up) == false)
		return false;
	
	return true;
}

bool bq24190_init(bool disable_charger)
{
	if (disable_charger)
	_bq24190_toggle_charger(false);
	
	///////////////////////////////////
	
	uint8_t timer_control = 0;
	if (i2c_read2_ex(CHRG_SDA, CHRG_SCL, BQ24190_READ_ADDRESS, BQ24190_REG_TIMER_CONTROL, &timer_control, _bq24190_pull_up) == false)
		return false;
	
	timer_control &= ~(0x3 << BQ24190_SHIFTS_I2C_WATCHDOG);
	timer_control |= (0x00 << BQ24190_SHIFTS_I2C_WATCHDOG);	// Disable I2C watch dog
	
	if (i2c_write_ex(CHRG_SDA, CHRG_SCL, BQ24190_WRITE_ADDRESS, BQ24190_REG_TIMER_CONTROL, timer_control, _bq24190_pull_up) == false)
		return false;
	
	///////////////////////////////////
	
	uint8_t input_src_ctl = 0;
	if (i2c_read2_ex(CHRG_SDA, CHRG_SCL, BQ24190_READ_ADDRESS, BQ24190_REG_INPUT_SOURCE_CTL, &input_src_ctl, _bq24190_pull_up) == false)
		return false;
	
	//input_src_ctl &= ~(0x07);
	input_src_ctl |= (0x07);	// Set 3A limit
	
	if (i2c_write_ex(CHRG_SDA, CHRG_SCL, BQ24190_WRITE_ADDRESS, BQ24190_REG_INPUT_SOURCE_CTL, input_src_ctl, _bq24190_pull_up) == false)
		return false;
	
	return true;
}

bool bq24190_has_interrupt(void)
{
	//return (io_test_pin(USBPM_IRQ) == false);
	return false;
}

bool bq24190_handle_irq(void)	// IRQ is pulsed (not held)
{
	return true;
}

//void bq24190_dump(void)
/*
bool bq24190_set_charge_current_limit(uint8_t deciamps)
{
	return true;
}
*/
#endif // CHARGER_TI
