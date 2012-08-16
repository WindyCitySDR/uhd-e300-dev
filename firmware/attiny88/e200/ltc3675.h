/*
 * Copyright 2012 Ettus Research LLC
 */

#ifndef LTC3675_H
#define LTC3675_H

//#include "types.h"
#include <stdbool.h>
#include <stdint.h>

bool ltc3675_init(void);

typedef enum ltc3675_regulators {
    LTC3675_REG_1,  // 1A Buck
    LTC3675_REG_2,  // 1A Buck
    LTC3675_REG_3,  // 500mA Buck
    LTC3675_REG_4,  // 500mA Buck
    LTC3675_REG_5,  // 1A Boost
    LTC3675_REG_6   // 1A Buck-Boost
    // LED Boost
} ltc3675_regulator_t;

bool ltc3675_enable_reg(ltc3675_regulator_t reg, bool on);
bool ltc3675_set_voltage(ltc3675_regulator_t reg, uint16_t voltage);

#endif /* LTC3675_H */
