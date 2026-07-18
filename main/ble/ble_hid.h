#pragma once

#include "stdlib.h"
#include "stdbool.h"

void send_mouse_report(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel, int8_t hwheel);
void ble_hid_init(void);
bool ble_is_connected(void);
void ble_bas_update(void);

