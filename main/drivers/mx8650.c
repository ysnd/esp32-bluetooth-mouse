#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "mx8650.h"

static cpi_t current_cpi = CPI_800;
static uint32_t hold_start = 0;
static dpi_state_t dpi_state = DPI_IDLE;
static const char *TAG = "MX8650";

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

static void mx8650_write_reg(uint8_t addr, uint8_t data) {
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

static uint8_t mx8650_read_reg(uint8_t addr) {
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

static void mx8650_resync(void) {
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
    *dy = -(int8_t)mx8650_read_reg(DELTA_Y_REG);
    return true;
}

//cpi 
static void mx8650_set_cpi(cpi_t cpi) {
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

static void mx8650_next_cpi(void) {
    current_cpi = (current_cpi + 1) % 4;
    mx8650_set_cpi(current_cpi);
}

void dpi_sm_update(uint8_t btn_state) {
    bool mmb = btn_state & 0x04;
    bool rmb = btn_state & 0x02;
    
    uint32_t now = esp_timer_get_time() / 1000;

    switch (dpi_state) {
        case DPI_IDLE:
            if (mmb && rmb) {
                hold_start = now;
                dpi_state = DPI_WAIT_HOLD;
            }
            break;

        case DPI_WAIT_HOLD:
            if (!mmb || !rmb) {
                dpi_state = DPI_IDLE;
                ESP_LOGD(TAG, "DPI IDLE");
            } else if ((now - hold_start) >= 2000) {
                mx8650_next_cpi();
                dpi_state = DPI_WAIT_RELEASE;
                ESP_LOGD(TAG, "DPI Berubah");
            }
            break;
        case DPI_WAIT_RELEASE:
            if (!mmb || !rmb) {
                dpi_state = DPI_IDLE;
            }
            break;
    }
}

void mx8650_set_sleep_mode(mx8650_sleep_mode_t mode) {
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

bool mx8650_is_in_sleep(void) {
uint8_t s = mx8650_read_reg(OPERATION_STATE_REG);
    return (s & STATE_OPSTATE_MASK) == STATE_SLEEP;
}


