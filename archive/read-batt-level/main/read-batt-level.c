#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali_scheme.h"

#define BAT_ADC_CH ADC_CHANNEL_6
#define BATT_AVG_SAMPLE 16 
#define BATT_HYSTERESIS_MV 20 

static uint16_t last_mv = 0;
static char const *TAG = "Read Batt Level";

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali;
static bool adc_calibrated = false;

void adc_init(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BAT_ADC_CH, &config));

    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_cfg, &adc1_cali);
    if (ret == ESP_OK) {
        adc_calibrated = true;
        ESP_LOGI(TAG, "ADC kalibrasi enable");
    } else {
        ESP_LOGW(TAG, "ADC kalibrasi unavailable (%s)", esp_err_to_name(ret));
    }
}

uint16_t battery_voltage_mv(void) {
    int raw = 0, raw_sum = 0, mv = 0;
    for (int i = 0; i < BATT_AVG_SAMPLE; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BAT_ADC_CH, &raw));
        raw_sum += raw;
    }
    int raw_avg = raw_sum / BATT_AVG_SAMPLE;
    if (adc_calibrated) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali, raw_avg, &mv));
    } else {
        mv = raw_avg;
    }
    mv *= 2;

    //hysteresis
    if (last_mv != 0 && abs(mv - last_mv) < BATT_HYSTERESIS_MV) {
        return last_mv;
    }

    last_mv = (uint16_t)mv;
    return last_mv;
}
uint8_t battery_level_percent(int16_t mv) {

    if (mv >= 4200) return 100;
    if (mv >= 4150) return 95;
    if (mv >= 4100) return 90;
    if (mv >= 4050) return 85;
    if (mv >= 4000) return 75;
    if (mv >= 3950) return 65;
    if (mv >= 3900) return 55;
    if (mv >= 3850) return 45;
    if (mv >= 3800) return 35;
    if (mv >= 3750) return 25;
    if (mv >= 3700) return 18;
    if (mv >= 3650) return 12;
    if (mv >= 3600) return 8;
    if (mv >= 3500) return 5;

    return 0;
}

void app_main(void)
{
    adc_init();
    while (1) {
        int16_t battery_mv = battery_voltage_mv();
        ESP_LOGI(TAG, "Battery = %.2f V", battery_mv / 1000.0f);
        ESP_LOGI(TAG, "Battery Level: %u%%", battery_level_percent(battery_mv));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
