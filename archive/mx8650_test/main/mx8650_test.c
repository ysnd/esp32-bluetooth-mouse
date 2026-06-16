#include <stdint.h>
#include <stdio.h>
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

const uint8_t SCLK = 18;
const uint8_t SDIO = 23;

#define PRODUCT_ID_REG 0x00
#define MOTION_STATUS_REG 0x02
#define DELTA_X_REG 0x03
#define DELTA_Y_REG 0x04
#define OPERATION_MODE_REG 0x05
#define IMAGE_QUALITY_REG 0x07

int8_t dy,dx;

static const char *TAG = "MX8650_test";

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
        pid = mx8650_read_reg(PRODUCT_ID_REG);
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

uint8_t mx8650_get_image_quality(void) {
    return mx8650_read_reg(IMAGE_QUALITY_REG);
}

void app_main(void){
    ESP_LOGI(TAG, "MX8650 SENSOR TEST");
    mx8650_init();
   
    while (1) {
        
        if (mx8650_has_motion()) {
            dx = mx8650_get_dx();
            dy = mx8650_get_dy();
            ESP_LOGI(TAG, "MOVE -> X: %d, Y: %d", dx, dy);
        }
        vTaskDelay(1);
    }
}
