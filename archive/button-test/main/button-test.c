#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

const uint8_t BTN_L = 21;
const uint8_t BTN_R = 33;
const uint8_t BTN_M = 32;
const uint8_t BTN_ML = 22;
const uint8_t BTN_MR = 27;
const uint8_t BTN_TF = 19;
const uint8_t BTN_TB = 5;

static const char *TAG = "BUTTON";

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

    if (!gpio_get_level(BTN_ML)) raw = 1;
    if (!gpio_get_level(BTN_MR)) raw = 2;

    uint32_t now = esp_timer_get_time() / 1000;

    if (raw != last_raw) {
        last_raw = raw;
        last_change_time = now;
    }

    if ((now - last_change_time) >= 5) {
        stable = raw;
    }

    switch (stable) {
        case 1: return 1;
        case 2: return -1;
        default: return 0;
    }
}

void app_main(void)
{
    btn_init();
    ESP_LOGI(TAG, "BUTTON TEST READY!");
    
    while (1) {
        int8_t hwheel = read_tilt();
        uint8_t btn = read_btn();
        if (btn & 0x01) ESP_LOGI(TAG, "LEFT CLICK");
        if (btn & 0x02) ESP_LOGI(TAG, "RIGHT CLICK");
        if (btn & 0x04) ESP_LOGI(TAG, "MIDDLE CLICK");
        if (btn & 0x08) ESP_LOGI(TAG, "THUMB FORWARD");
        if (btn & 0x10) ESP_LOGI(TAG, "THUMB BACK");
        if (hwheel == 1) ESP_LOGI(TAG, "TILT LEFT : %d", hwheel);
        if (hwheel == -1) ESP_LOGI(TAG, "TILT RIGHT: %d",hwheel);
       
        vTaskDelay(1);
    }
}
