#ifndef POWER_H
#define POWER_H

#include <stdbool.h>
#include <stdint.h>

void tps54478_init(void);
void tps54478_set_power(bool on);   // Zynq core power (1.0V for FPGA)
bool tps54478_is_power_good(void);

void charge_set_led(bool on);

void power_signal_interrupt(void);

void fpga_reset(bool delay);

typedef enum power_subsystems {
    PS_UNKNOWN,
    PS_FPGA,
    PS_VDRAM,
    PS_PERIPHERALS_1_8,
    PS_PERIPHERALS_3_3,
    PS_TX
} power_subsystem_t;

bool power_enable(power_subsystem_t subsys, bool on);

void battery_init(void);
uint16_t battery_get_voltage(void);  // mV

void power_init(void);
bool power_on(void);

#endif // POWER_H
