#include <stdio.h>
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "xtensa/hal.h"

const uint8_t SCLK = 18;
const uint8_t SDIO = 23;
const uint8_t BTN_L = 21;
const uint8_t BTN_R = 33;
const uint8_t BTN_M = 32;
const uint8_t BTN_ML = 22;
const uint8_t BTN_MR = 27;
const uint8_t BTN_TF = 19;
const uint8_t BTN_TB = 5;
const uint8_t ENC_A = 25;
const uint8_t ENC_B = 26;

static const int8_t table[16] = {
    0, -1, +1, 0, 
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0
};

#define PRODUCT_ID_REG 0x00
#define MOTION_STATUS_REG 0x02
#define DELTA_X_REG 0x03
#define DELTA_Y_REG 0x04
#define OPERATION_MODE_REG 0x05

int8_t dy,dx;

static const char *TAG = "MX8650_Mouse_Test";

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

//Encoder
void enc_init(void) {
    gpio_config_t enc_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ENC_A) | (1ULL << ENC_B),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&enc_conf);
}

//MX8650 Sensor
static inline void cmd_send_bit(uint8_t bit) {
    gpio_set_level(SCLK, 0);
    esp_rom_delay_us(2);
    gpio_set_level(SDIO, bit ? 1 : 0);
    gpio_set_level(SCLK, 1);
    esp_rom_delay_us(2);
}

static uint8_t cmd_read_bit(void) {
    gpio_set_level(SCLK, 0);
    esp_rom_delay_us(2);
    gpio_set_level(SCLK, 1);
    esp_rom_delay_us(2);
    return gpio_get_level(SDIO);
}

void mx8650_write_reg(uint8_t addr, uint8_t data) {
    uint8_t cmd = 0x80 | (addr & 0x7F);
    gpio_set_direction(SDIO, GPIO_MODE_OUTPUT);
    for (int i = 7; i >= 0; i--) {
        cmd_send_bit((cmd >> i) & 1);
    }
    for (int i = 7; i >= 0; i--) {
        cmd_send_bit((data >> i) & 1);
    }
    gpio_set_level(SCLK, 1);
}

uint8_t mx8650_read_reg(uint8_t addr) {
    uint8_t cmd = addr & 0x7F;
    uint8_t data = 0;
    gpio_set_direction(SDIO, GPIO_MODE_OUTPUT);
    for (int i = 7; i >= 0; i--) {
        cmd_send_bit((cmd >> i) & 1);
    }
    gpio_set_direction(SDIO, GPIO_MODE_INPUT);
    esp_rom_delay_us(5);
    for (int i = 7; i >= 0; i--) {
        if (cmd_read_bit()) {
            data |= (1 << i);
        }
    }
    gpio_set_direction(SDIO, GPIO_MODE_OUTPUT);

    return data;
}

void mx8650_resync(void) {
    gpio_set_direction(SDIO, GPIO_MODE_OUTPUT);
    gpio_set_level(SCLK, 1);
    esp_rom_delay_us(100);

    gpio_set_level(SCLK, 0); //tRESYNC min 1us

    esp_rom_delay_us(2);
    gpio_set_level(SCLK, 1);
    esp_rom_delay_us(100); //tSIWTT
}

void mx8650_init(void) {
    gpio_reset_pin(SCLK);
    gpio_set_direction(SCLK, GPIO_MODE_OUTPUT);
    gpio_set_level(SCLK, 1);

    gpio_reset_pin(SDIO);
    gpio_set_direction(SDIO, GPIO_MODE_OUTPUT);

    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t pid = mx8650_read_reg(PRODUCT_ID_REG);
    ESP_LOGI(TAG, "Product ID: 0x%02X (harus 0x30)", pid);
    if (pid != 0x30) {
        ESP_LOGW(TAG, "MX8650 teu baleg konek na cek wairing maneh! nyobian resync");
        mx8650_resync();
        if (pid != 0x30) {
            ESP_LOGE(TAG, "MX650 teu aya cek dei wairing kehed.");
        }
        return;
    }
    mx8650_write_reg(OPERATION_MODE_REG, 0xB8); //force normal mode led on sleep disable
    ESP_LOGI(TAG, "MX8650 SENSOR READY");
}

bool mx8650_has_motion(void) {
    return (mx8650_read_reg(MOTION_STATUS_REG) & 0x80);
}

int8_t mx8650_get_dx(void) {
    return (int8_t)mx8650_read_reg(DELTA_X_REG);
}

int8_t mx8650_get_dy(void) {
    return -(int8_t)mx8650_read_reg(DELTA_Y_REG);
}

void app_main(void){
    ESP_LOGI(TAG, "MX8650 MOUSE PoC TEST");
    btn_init();
    enc_init();
    mx8650_init();

    uint8_t a = gpio_get_level(ENC_A);
    uint8_t b = gpio_get_level(ENC_B);

    uint8_t enc_last_state = (a << 1) |b;
    int32_t enc_count = 0;
   
    while (1) {
        
        if (mx8650_has_motion()) {
            dx = mx8650_get_dx();
            dy = mx8650_get_dy();
            ESP_LOGI(TAG, "MOVE -> X: %d, Y: %d", dx, dy);
        }
        uint8_t btn = read_btn();
        if (btn & 0x01) ESP_LOGI(TAG, "LEFT CLICK");
        if (btn & 0x02) ESP_LOGI(TAG, "RIGHT CLICK");
        if (btn & 0x04) ESP_LOGI(TAG, "MIDDLE CLICK");
        if (btn & 0x08) ESP_LOGI(TAG, "THUMB FORWARD");
        if (btn & 0x10) ESP_LOGI(TAG, "THUMB BACK");

        a = gpio_get_level(ENC_A);
        b = gpio_get_level(ENC_B);
        uint8_t curr = (a << 1) | b;

        if (curr != enc_last_state) {
            uint8_t idx = (enc_last_state << 2) | curr;
            int8_t delta = table[idx];
            enc_count += delta;
            enc_last_state = curr;
        }
        //report per 2 counts = 1 detent
        int8_t wheel = 0;
        if (enc_count >= 2) {
            enc_count -= 2; 
            wheel = 1;  
            ESP_LOGI(TAG, "SCROLL:%d", wheel);
        }
        if (enc_count <= -2) {
            enc_count += 2;
            wheel = -1;
            ESP_LOGI(TAG, "SCROLL:%d", wheel);
        }

        vTaskDelay(1);
    }
}

