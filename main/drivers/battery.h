#pragma once

#include <stdlib.h>

void battery_init(void);
uint16_t battery_voltage_mv(void);
uint8_t battery_level_percent(uint16_t mv);
