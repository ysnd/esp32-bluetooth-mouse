#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "nvs_flash.h"
#include "drivers/config.h"
#include "drivers/battery.h"
#include "drivers/encoder.h"
#include "drivers/mx8650.h"
#include "drivers/button.h"
#include "ble/ble_hid.h"

typedef enum {
    POLL_ACTIVE,
    POLL_PENDING_IDLE,
    POLL_DEEP_IDLE
} poll_state_t;

static uint32_t batt_last_update = 0;

static const char *TAG = "MX8650_BLE_Mouse";

static poll_state_t poll_state = POLL_ACTIVE;
static uint32_t idle_counter = 0;

void mouse_polling_task(void *pvParameters) {
    uint8_t last_btn = 0;
    
    while (1) {
        bool report_needed = false;
        int8_t dx = 0, dy = 0;
        bool had_activity = false;

        if (mx8650_read_motion(&dx, &dy)) {
            report_needed = true;
            had_activity = true;
            ESP_LOGD(TAG, "MOVE -> X: %d, Y: %d", dx, dy);
        }

        uint8_t btn = read_btn();
        if (btn != last_btn) {
            report_needed = true;
            had_activity = true;
            last_btn = btn;
        }
        if (btn & 0x01) ESP_LOGD(TAG, "LEFT CLICK");
        if (btn & 0x02) ESP_LOGD(TAG, "RIGHT CLICK");
        if (btn & 0x04) ESP_LOGD(TAG, "MIDDLE CLICK");
        if (btn & 0x08) ESP_LOGD(TAG, "THUMB BACK");
        if (btn & 0x10) ESP_LOGD(TAG, "THUMB FORWARD");
        
        int8_t wheel = get_encoder_val();
        if (wheel != 0) {
            report_needed = true;
            had_activity = true;
            ESP_LOGD(TAG, "SCROLL: %d", wheel);
        }

        int8_t hwheel = read_tilt();
        if (hwheel != 0) {
            report_needed = true;
            had_activity = true;
            ESP_LOGD(TAG, "TILT : %d",hwheel);
        }

        dpi_sm_update(btn);
        uint32_t now = esp_log_timestamp();
        if ((now - batt_last_update) >= 30000) {
            ble_bas_update();
            batt_last_update = now;
        }

        if (report_needed && ble_is_connected()) {
            send_mouse_report(btn, dx, dy, wheel, hwheel);
            ESP_LOGD(TAG, "REPORTED");
        }

        if (had_activity) {
            poll_state = POLL_ACTIVE;
            idle_counter = 0;
        } else {
            idle_counter++;
        }

        switch (poll_state) {
            case POLL_ACTIVE:
                if (idle_counter >= 50) {
                    poll_state = POLL_PENDING_IDLE;
                    idle_counter = 0;
                }
                break;
            case POLL_PENDING_IDLE:
                if (idle_counter >= 250 && mx8650_is_in_sleep()) {
                    poll_state = POLL_DEEP_IDLE;
                    idle_counter = 0;
                }
                break;
            case POLL_DEEP_IDLE:
                break;
        }

        switch (poll_state) {
            case POLL_ACTIVE:
                vTaskDelay(pdMS_TO_TICKS(4));
                break;
            case POLL_PENDING_IDLE:
                vTaskDelay(pdMS_TO_TICKS(20));
                break;
            case POLL_DEEP_IDLE:
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
        }
    }
}

void app_main(void){
    ESP_LOGI(TAG, "MX8650 MOUSE BLE HID");
    esp_log_level_set("NimBLE", ESP_LOG_WARN);
    esp_log_level_set("ble_gap", ESP_LOG_WARN);
    esp_log_level_set("NIMBLE_HIDD", ESP_LOG_NONE);
    //cpu freq scalling
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 80,
        .light_sleep_enable = true
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_cfg));
    ESP_LOGI(TAG, "CPU PM %d Mhz max / %d Mhz min", pm_cfg.max_freq_mhz, pm_cfg.min_freq_mhz);
    
    btn_init();
    enc_init();
    battery_init();
    mx8650_init();
    mx8650_set_sleep_mode(MX8650_SLEEP1_SLEEP2);
    ESP_LOGI(TAG, "Tick rate: %d Hz, 1 tick = %d ms", configTICK_RATE_HZ, portTICK_PERIOD_MS);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret ==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ble_hid_init();
    ble_bas_update();
 
    //polling task
    xTaskCreate(mouse_polling_task, "POLLING TASK", 4096, NULL, 5, NULL);
}

