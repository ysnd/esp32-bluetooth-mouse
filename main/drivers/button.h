#pragma once

#include "driver/gpio.h"
#include <stdint.h>

void btn_init(void);
uint8_t read_btn(void);
int8_t read_tilt(void);
