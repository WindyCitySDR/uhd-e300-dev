/*
 * ltc4155.c
 */ 

#include "config.h"
#include "ltc4155.h"

#include "io.h"
#include "i2c.h"
#include "power.h"

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

const bool _ltc4155_pull_up = false;

#define LTC4155_BASE_ADDRESS    0x12
#define LTC4155_WRITE_ADDRESS   (LTC4155_BASE_ADDRESS + 0)
#define LTC4155_READ_ADDRESS    (LTC4155_BASE_ADDRESS + 1)

#define LTC4155_RETRY_DELAY     1   // us MAGIC
#define LTC4155_MAX_ACK_RETRIES 10  // * LTC4155_RETRY_DELAY us

#define LTC4155_SCL_LOW_PERIOD  2   // 1.3 us
#define LTC4155_SCL_HIGH_PERIOD 1   // 0.6 us
#define LTC4155_BUS_FREE_TIME   2   // 1.3 us
#define LTC4155_STOP_TIME       1   // 0.6 us

enum LTC4155Registers
{
	LTC4155_REG_USB			= 0x00,	// W/R
	LTC4155_REG_WALL		= 0x01,	// W/R
	LTC4155_REG_CHARGE		= 0x02,	// W/R
	LTC4155_REG_STATUS		= 0x03,	// R
	LTC4155_REG_GOOD		= 0x04,	// R
	LTC4155_REG_THERMISTOR	= 0x05,	// R
	LTC4155_REG_ENABLE		= 0x06,	// W/R
	LTC4155_REG_ARM_AND_SHIP= 0x07	// W
};

enum LTC4155InterruptMasks	// LTC4155_REG_ENABLE
{
	LTC4155_ENABLE_USB_OTG	= 1 << 1,
	
	LTC4155_INT_UVCL	= 1 << 2,
	LTC4155_INT_ILIMIT	= 1 << 3,
	LTC4155_INT_USB_OTG	= 1 << 4,
	LTC4155_INT_EXT_PWR	= 1 << 5,
	LTC4155_INT_FAULT	= 1 << 6,
	LTC4155_INT_CHARGER	= 1 << 7
};

enum LTC4155Options	// LTC4155_REG_USB
{
	LTC4155_USB_OTG_LOCKOUT				= 1 << 5,
	LTC4155_ENABLE_BATTERY_CONDITIONER	= 1 << 6,
	LTC4155_DISABLE_INPUT_UVCL			= 1 << 7
};

enum LTC4155Statuses	// LTC4155_REG_STATUS
{
	LTC4155_LOW_BATTERY		= 1 << 0,
	LTC4155_BOOST_ENABLE	= 1 << 3,
	LTC4155_ID_PIN_DETECT	= 1 << 4,
};

enum LTC4155Goods	// LTC4155_REG_GOOD
{
	LTC4155_BAD_CELL_FAULT		= 1 << 0,
	LTC4155_OTG_FAULT			= 1 << 1,
	LTC4155_OVP_ACTIVE			= 1 << 2,
	LTC4155_INPUT_UVCL_ACTIVE	= 1 << 3,
	LTC4155_INPUT_CURRENT_LIMIT_ACTIVE = 1 << 4,
	LTC4155_WALLSNS_GOOD		= 1 << 5,
	LTC4155_USBSNS_GOOD			= 1 << 6,
	LTC4155_EXTERNAL_POWER_GOOD	= 1 << 7
};

enum LTC4155BatteryChargerStatues
{
	LTC4155_CHARGER_OFF,
	LTC4155_CHARGER_LOW_BATTERY_VOLTAGE,
	LTC4155_CHARGER_CONSTANT_CURRENT,
	LTC4155_CHARGER_CONSTANT_VOLTAGE_VPROG_GT_VCX,
	LTC4155_CHARGER_CONSTANT_VOLTAGE_VPROG_LT_VCX,
	LTC4155_CHARGER_NTC_TOO_WARM,
	LTC4155_CHARGER_NTC_TOO_COLD,
	LTC4155_CHARGER_NTC_HOT
};

enum LTC4155ThermistorStatuses
{
	LTC4155_NTC_NORMAL,
	LTC4155_NTC_TOO_COLD,
	LTC4155_NTC_TOO_WARM,
	LTC4155_NTC_FAULT
};

static uint8_t _ltc4155_interrupt_mask =
	LTC4155_ENABLE_USB_OTG |// Enable +5V on USB connector
	//LTC4155_INT_UVCL |
	//LTC4155_INT_ILIMIT |
	//LTC4155_INT_USB_OTG |
	LTC4155_INT_EXT_PWR |	// Turn up current limit
	LTC4155_INT_FAULT |		// Blink error
	LTC4155_INT_CHARGER;	// Illuminate charge LED

static void _ltc4155_clear_irq(void)
{
	i2c_write_ex(CHRG_SDA, CHRG_SCL, LTC4155_WRITE_ADDRESS, LTC4155_REG_ENABLE, _ltc4155_interrupt_mask, _ltc4155_pull_up);
}

bool ltc4155_init(void)
{
	io_input_pin(USBPM_IRQ);
	io_set_pin(USBPM_IRQ);	// Enable pull-up for Open Drain
#ifdef I2C_REWORK
	i2c_init_ex(CHRG_SDA, CHRG_SCL, _ltc4155_pull_up);
#endif // I2C_REWORK
	_ltc4155_clear_irq();	// Will set interrupt masks
	
	i2c_write_ex(CHRG_SDA, CHRG_SCL, LTC4155_WRITE_ADDRESS, LTC4155_REG_WALL, 0x1F, _ltc4155_pull_up);	// Without this, fails when bringing up 1.8V
	
	// Vbatt float = 4.05V
	// Charge safety timer = 4hr
	// Battery charger I limit = 100%
	// Full capacity charge threshold = 10%
	
	// FIXME:
	// Disable ID pin detection & autonomous startup
	// Enable OTG
	//i2c_write_ex(CHRG_SDA, CHRG_SCL, LTC4155_WRITE_ADDRESS, LTC4155_REG_USB, LTC4155_USB_OTG_LOCKOUT, _ltc4155_pull_up);	// Disable autonomous startup
	//i2c_write_ex(CHRG_SDA, CHRG_SCL, LTC4155_WRITE_ADDRESS, LTC4155_REG_ENABLE, LTC4155_ENABLE_USB_OTG, _ltc4155_pull_up);	// Enable OTG
	
	return true;
}

bool ltc4155_has_interrupt(void)
{
	return (io_test_pin(USBPM_IRQ) == false);
}

bool ltc4155_arm_ship_and_store(void)
{
	return true;
}

bool ltc4155_get_thermistor(uint8_t* val, bool* warning)
{
	return true;
}
