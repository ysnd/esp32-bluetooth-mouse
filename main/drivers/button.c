#include "button.h"
#include "config.h"
#include "esp_timer.h"

//Button
void btn_init(void) {
    gpio_config_t btn_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BTN_L) | (1ULL << BTN_R) | (1ULL << BTN_M) | (1ULL << BTN_ML) | (1ULL << BTN_MR) | (1ULL << BTN_TF) | (1ULL << BTN_TB),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_conf);
}

uint8_t read_btn(void) {
    static uint8_t stable_state = 0x00;
    static uint8_t last_raw = 0x00;
    static uint32_t last_change_time = 0;

    uint8_t raw = 0;
    if (!gpio_get_level(BTN_L)) raw |= 0x01;
    if (!gpio_get_level(BTN_R)) raw |= 0x02;
    if (!gpio_get_level(BTN_M)) raw |= 0x04;
    if (!gpio_get_level(BTN_TF)) raw |= 0x08;
    if (!gpio_get_level(BTN_TB)) raw |= 0x10;

    uint32_t now = esp_timer_get_time() / 1000; //ms 
    if (raw != last_raw) {
        last_raw = raw;
        last_change_time = now;
    }
    if ((now - last_change_time) >= 10) {
        stable_state = raw;
    }
    return stable_state;
}

int8_t read_tilt(void) {
    static uint8_t stable = 0;
    static uint8_t last_raw = 0;
    static uint32_t last_change_time = 0;

    uint8_t raw = 0;

    if (!gpio_get_level(BTN_ML)) raw |= 0x01;
    if (!gpio_get_level(BTN_MR)) raw |= 0x02;

    uint32_t now = esp_timer_get_time() / 1000;

    if (raw != last_raw) {
        last_raw = raw;
        last_change_time = now;
    }

    if ((now - last_change_time) >= 5) {
        stable = raw;
    }

    switch (stable) {
        case 0x01: 
            return -1;
        case 0x02: 
            return 1;
        case 0x03:
            return 0;//imposible bisa 2 on
        default: 
            return 0;
    }
}
