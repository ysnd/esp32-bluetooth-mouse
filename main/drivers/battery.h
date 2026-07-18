#pragma once

#include <stdlib.h>

typedef struct {
    uint16_t voltage_mv;
    uint8_t percent;
} battery_info_t;

void battery_init(void);
battery_info_t battery_get_info(void);
