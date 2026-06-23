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
#define CONFIGURATION_REG 0x06
#define OPERATION_STATE_REG 0x08
#define WRITE_PROTECT_REG 0x09

#define MX8650_LED_CTR (1 << 7)
#define MX8650_BIT6_MUST0 (0 << 6)
#define MX8650_BIT5_MUST1 (1 << 5)

#define MX8650_SLP_EN (1 << 4)
#define MX8650_SLP2_EN (1 << 3)
#define MX8650_SLP2_MU (1 << 2)
#define MX8650_SLP1_MU (1 << 1)
#define MX8650_WAKEUP (1 << 0)

#define MX8650_MOTSWK_BIT (1 << 6)

#define STATE_OPSTATE_MASK 0x07
#define STATE_SLP_STATE (1 << 3)

#define STATE_NORMAL 0x00
#define STATE_ENTRY_SLP1 0x01
#define STATE_ENTRY_SLP2 0x02
#define STATE_SLEEP 0x04

typedef enum {
    CPI_800 = 0,
    CPI_1000,
    CPI_1200,
    CPI_1600
} cpi_t;

static cpi_t current_cpi = CPI_800;

static const uint16_t cpi_table[] = {800, 1000, 1200, 1600};

typedef enum {
    DPI_IDLE,
    DPI_WAIT_HOLD,
    DPI_WAIT_RELEASE
} dpi_state_t;

static dpi_state_t dpi_state = DPI_IDLE;

typedef enum {
    MX8650_SLEEP_DISABLED,
    MX8650_SLEEP1_ONLY,
    MX8650_SLEEP1_SLEEP2
} mx8650_sleep_mode_t;

typedef enum {
    MX8650_MOTSWK_MOTION = 0,
    MX8650_MOTSWK_SWKINT
} mx8650_motswk_mode_t;

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
    mx8650_write_reg(OPERATION_MODE_REG, MX8650_LED_CTR | MX8650_BIT5_MUST1);
    
    ESP_LOGI(TAG, "MX8650 SENSOR READY");
}

bool mx8650_read_motion(int8_t *dx, int8_t *dy) {
    uint8_t motion = mx8650_read_reg(MOTION_STATUS_REG);

    if (!(motion & 0x80)) {
        return false;
    }
    *dx = (int8_t)mx8650_read_reg(DELTA_X_REG);
    *dy = (int8_t)mx8650_read_reg(DELTA_Y_REG);
    return true;
}

//cpi 
void mx8650_set_cpi(cpi_t cpi) {
    //disable write protect 
    mx8650_write_reg(WRITE_PROTECT_REG, 0x5A);
    uint8_t cfg;
    cfg = mx8650_read_reg(CONFIGURATION_REG);
    cfg &= ~0x03;
    cfg |= (uint8_t)cpi;

    mx8650_write_reg(CONFIGURATION_REG, cfg);

    //enable write protect lagi
    mx8650_write_reg(WRITE_PROTECT_REG, 0x00);

    current_cpi = cpi;
    ESP_LOGI(TAG, "CPI ganti -> %d", cpi_table[current_cpi]);
}

cpi_t mx8650_get_cpi(void) {
    return current_cpi;
}

static void mx8650_set_sleep_mode(mx8650_sleep_mode_t mode) {
    uint8_t om = MX8650_LED_CTR | MX8650_BIT5_MUST1;
    switch (mode) {
        case MX8650_SLEEP_DISABLED:
            break;
        case MX8650_SLEEP1_ONLY:
            om |= MX8650_SLP_EN;
            break;
        case MX8650_SLEEP1_SLEEP2:
            om |= MX8650_SLP_EN | MX8650_SLP2_EN;
    }
    mx8650_write_reg(OPERATION_MODE_REG, om);
}

void mx8650_set_motswk(mx8650_motswk_mode_t mode) {
    uint8_t cfg = mx8650_read_reg(CONFIGURATION_REG);

    if (mode == MX8650_MOTSWK_SWKINT) {
        cfg |= MX8650_MOTSWK_BIT;
    } else {
        cfg &= ~MX8650_MOTSWK_BIT;
    }
    mx8650_write_reg(CONFIGURATION_REG, cfg);
}

void mx8650_print_state(void) {
    uint8_t s = mx8650_read_reg(OPERATION_STATE_REG);

    uint8_t state = s & STATE_OPSTATE_MASK;
    bool sleep2 = s & STATE_SLP_STATE;
    
    switch (state) {
        case STATE_NORMAL:
            ESP_LOGI(TAG, "NORMAL");
            break;
        case STATE_ENTRY_SLP1:
            ESP_LOGI(TAG, "ENTRY SLEEP1");
            break;
        case STATE_ENTRY_SLP2:
            ESP_LOGI(TAG, "ENTRY SLEEP2");
            break;
        case STATE_SLEEP:
            ESP_LOGI(TAG, "%s", sleep2 ? "SLEEP2" : "SLEEP1");
            break;
        default:
            ESP_LOGI(TAG, "UNKNOWN");
            break;
    }
}

uint8_t mx8650_get_image_quality(void) {
    return mx8650_read_reg(IMAGE_QUALITY_REG);
}

void app_main(void){
    ESP_LOGI(TAG, "MX8650 SENSOR TEST");
    mx8650_init();
    mx8650_set_sleep_mode(MX8650_SLEEP1_SLEEP2);
   
    while (1) {
        int8_t dx = 0, dy=0; 
        if (mx8650_read_motion(&dx, &dy)) {
            ESP_LOGI(TAG, "MOVE -> X: %d, Y: %d", dx, dy);
        }
        vTaskDelay(1);
    }
}
