#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

uint8_t SCLK = 18;
uint8_t SDIO = 23;
uint8_t BTN_L = 32;
uint8_t BTN_R = 33;

static const char *TAG = "TESTING MX mouse";

static void spi_delay(void) {
    for (volatile int i =0; i <10; i++);    
}
//kirim 1 bit via sdio 
static inline void spi_send_bit(uint8_t bit) {
    gpio_set_level(SCLK, 0);
    spi_delay();

    //send data
    gpio_set_level(SDIO, bit? 1: 0);

    //clk rising edge(sensor baca data)
    gpio_set_level(SCLK, 1);
    spi_delay();
}

//terima 1 bit via sdio 
static uint8_t spi_read_bit(void) {
    gpio_set_level(SCLK, 0);
    spi_delay();

    //clk rising edge reading from senseor
    gpio_set_level(SCLK, 1);
    spi_delay();
    return gpio_get_level(SDIO);
}

// mx8650 protocol
//write register
void mx8650_write(uint8_t addr, uint8_t data) {
    //CMD: bit 7=1 (write), bit 6-0=addr 
    uint8_t cmd=0x80 | (addr & 0x7F);

    //set sdio output
    gpio_set_direction(SDIO, GPIO_MODE_OUTPUT);

    //send 8 nit cmd(MSB first)
    for (int i = 7; i>=0; i--) {
        spi_send_bit((cmd >> i) & 1);
    }
    // kirim 8bit data 
    for (int i =7 ; i>= 0; i--) {
        spi_send_bit((data >> i) & 1);
    }
}
//read register 
uint8_t mx8650_read(uint8_t addr) {
    uint8_t cmd= addr & 0x7F;
    uint8_t data = 0;

    // set sdio ke output untuk kirim address
    gpio_set_direction(SDIO, GPIO_MODE_OUTPUT);

    //kirim cmd 
    for (int i =7; i>=0; i--) {
        spi_send_bit((cmd >> i) & 1);
    }

    // set sdio input (High-z) sensor drive jalur
    gpio_set_direction(SDIO, GPIO_MODE_INPUT);

    //read 8 bit from sensor 
    for (int i=7; i >=0 ; i--) {
        if (spi_read_bit()) {
            data |= (1 << i);
        }
    }
    //kembalikan stpio ke output untuk transaction berikutnya
    gpio_set_direction(SDIO, GPIO_MODE_OUTPUT);
    return data;
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-IDF MX8650 Driver Starting...");
    //init gpio
    gpio_reset_pin(SCLK);
    gpio_set_direction(SCLK, GPIO_MODE_OUTPUT);
    gpio_set_level(SCLK, 1); // idle high spi mode 0

    gpio_reset_pin(SDIO);
    gpio_set_direction(SDIO, GPIO_MODE_OUTPUT);

    vTaskDelay(pdMS_TO_TICKS(500));

    //TESTING
    uint8_t pid= mx8650_read(0x00);
    ESP_LOGI(TAG, "Product ID : 0x%02X (Should be 0x30)", pid);
    if (pid != 0x30) {
        ESP_LOGE(TAG, "Connection Failed! check your wiring");
        return;
    }

    //setup: force normal mode(disable sleep for debug)
    //write 0x05 (opration mode) = 0xB8 default: LED ON Sleep disable
    mx8650_write(0x05, 0xB8);

    ESP_LOGI(TAG, "Sensor Ready. Polling x jeng y");

    while (1) {
        // baca motion status
        uint8_t motion = mx8650_read(0x02);
        if (motion & 0x80) {
            // ada gerakan read delta x dan y 
            //int8 karena supaya nilai negatif antara -128 127
            int8_t dx=(int8_t)mx8650_read(0x03);
            int8_t dy=-(int8_t)mx8650_read(0x04);

            ESP_LOGI(TAG, "MOVE -> X: %d, Y:%d", dx, dy);
        } else {
           // ESP_LOGI(TAG, "Idle");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
