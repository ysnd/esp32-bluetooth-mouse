#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

const uint8_t ENC_A = 25;
const uint8_t ENC_B = 26;

static const int8_t enc_table[16] = {
    0, -1, +1, 0, 
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0
};

typedef struct {
    uint8_t last_state;
    int32_t count;
} encoder_t;

static encoder_t enc = {0};

static const char *TAG = "ENCODER_TEST";

void enc_init(void) {
    gpio_config_t enc_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ENC_A) | (1ULL << ENC_B),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&enc_conf);
    uint8_t a = gpio_get_level(ENC_A);
    uint8_t b = gpio_get_level(ENC_B);

    enc.last_state = (a << 1) | b;
}

int8_t encoder_read(void) {
    uint8_t a = gpio_get_level(ENC_A);
    uint8_t b = gpio_get_level(ENC_B);

    uint8_t enc_curr = (a << 1) |b;

    if (enc_curr != enc.last_state) {
        uint8_t idx = (enc.last_state << 2) | enc_curr;
        enc.count += enc_table[idx];
        enc.last_state = enc_curr;
    }

    if (enc.count >= 2) {
        enc.count -= 2;
        return 1;
    }

    if (enc.count <= -2) {
        enc.count += 2;
        return -1;
    }
    return 0;
}

void app_main(void) {
    enc_init();
    ESP_LOGI(TAG, "ENCODER TESTING READY");

    while (1) {
        int8_t wheel = encoder_read();
        if (wheel != 0) {
            ESP_LOGI(TAG, "SCROLL: %d", wheel);
        }
        vTaskDelay(1);
    }
}
