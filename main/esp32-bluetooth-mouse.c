#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_pm.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "esp_hidd.h"
#include "host/ble_gap.h"
#include "host/ble_hs_adv.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"
#include "nimble/ble.h"
#include "esp_nimble_hci.h"
#include "services/gap/ble_svc_gap.h"

const uint8_t SCLK = 18;
const uint8_t SDIO = 23;
const uint8_t BTN_L = 21;
const uint8_t BTN_R = 33;
const uint8_t BTN_M = 32;
const uint8_t BTN_ML = 22;
const uint8_t BTN_MR = 27;
const uint8_t BTN_TF = 5;
const uint8_t BTN_TB = 19;
const uint8_t ENC_A = 25;
const uint8_t ENC_B = 26;

#define PRODUCT_ID_REG 0x00
#define MOTION_STATUS_REG 0x02
#define DELTA_X_REG 0x03
#define DELTA_Y_REG 0x04
#define OPERATION_MODE_REG 0x05
#define CONFIGURATION_REG 0x06
#define OPERATION_STATE_REG 0x08
#define WRITE_PROTECT_REG 0x09

#define MX8650_LED_CTR (1 << 7)
#define MX8650_BIT6_MUST0 (0 << 6)
#define MX8650_BIT5_MUST1 (1 << 5)

#define MX8650_SLP_EN (1 << 4)
#define MX8650_SLP2_EN (1 << 3)

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

typedef enum {
    DPI_IDLE,
    DPI_WAIT_HOLD,
    DPI_WAIT_RELEASE
} dpi_state_t;

typedef enum {
    MX8650_SLEEP_DISABLED,
    MX8650_SLEEP1_ONLY,
    MX8650_SLEEP1_SLEEP2
} mx8650_sleep_mode_t;

typedef enum {
    MX8650_MOTSWK_MOTION = 0,
    MX8650_MOTSWK_SWKINT
} mx8650_motswk_mode_t;

typedef enum {
    POLL_ACTIVE,
    POLL_PENDING_IDLE,
    POLL_DEEP_IDLE
} poll_state_t;

typedef struct {
    uint8_t last_state;
    int32_t count;
} encoder_t;

static const int8_t enc_table[16] = {
    0, -1, +1, 0, 
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0
};

static const char *TAG = "MX8650_Mouse_Test";
static encoder_t enc = {0};
static cpi_t current_cpi = CPI_800;
static const uint16_t cpi_table[] = {800, 1000, 1200, 1600};
static dpi_state_t dpi_state = DPI_IDLE;
static uint32_t hold_start = 0;
static poll_state_t poll_state = POLL_ACTIVE;
static uint32_t idle_counter = 0;

//HID report descriptor
static const uint8_t hid_mouse_report_desc[] = {
    //mouse
    0x05, 0x01, //usage oage (Generic Desktop)
    0x09, 0x02, //usage mouse 
    0xA1, 0x01, //collection (application)

    //pointer
    0x09, 0x01, //usage (pointer)
    0xA1, 0x00, //collection (physical)

    //button 1-5
    0x05, 0x09, //usage page (button)
    0x19, 0x01, //usage min(1)
    0x29, 0x05, //usage max(5)
    0x15, 0x00, //logical min(0)
    0x25, 0x01, //logical max(1)
    0x95, 0x05, //report count(5)
    0x75, 0x01, //report size(1)
    0x81, 0x02, //input (data, variable, absolute)

    //padding
    0x95, 0x01, //report count(1)
    0x75, 0x03, //report size(3)
    0x81, 0x01, //Input(constant)

    //xy wheel
    0x05, 0x01, //use page (generic Desktop)
    0x09, 0x30, //usage (x)
    0x09, 0x31, //usage (y)
    0x09, 0x38, //usage (wheel)
                
    0x15, 0x81, // logical min(-127)
    0x25, 0x7F, //logical max(127)
    0x75, 0x08, //report size(8)
    0x95, 0x03, //report count(3)
    0x81, 0x06, //input (data, variable, relative)

    //AC-Pan/horizontall scroll
    0x05, 0x0C,
    0x0A, 0x38, 0x02,

    0x15, 0x81,
    0x25, 0x7F,
    0x75, 0x08,
    0x95, 0x01,
    0x81, 0x06,

    0xC0, //End collection
    0xC0, //End collection
};

#define HID_MOUSE_REPORT_DESC_LEN sizeof(hid_mouse_report_desc)
#define MOUSE_REPORT_SIZE 5
#define MOUSE_MAP_INDEX 0 
#define MOUSE_REPORT_ID 0 

static esp_hid_raw_report_map_t ble_report_maps[] = {
    {
        .data = hid_mouse_report_desc,
        .len = HID_MOUSE_REPORT_DESC_LEN
    }
};

static esp_hid_device_config_t ble_hid_cfg = {
    .vendor_id = 0x16C0,
    .product_id = 0x05DF,
    .version = 0x0100,
    .device_name = "MX8650 Mouse",
    .manufacturer_name = "DIY ESP32 MX8650",
    .serial_number = "1234567890",
    .report_maps = ble_report_maps,
    .report_maps_len = 1,
};

typedef struct{
    esp_hidd_dev_t *hid_dev;
    TaskHandle_t task_hdl;
    uint8_t buffer[MOUSE_REPORT_SIZE];
} local_param_t;

static local_param_t s_ble_hid_param = {0};
static SemaphoreHandle_t ble_hid_mutex = NULL;
static bool ble_connected = false;
static uint16_t s_conn_handle = 0;

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
    uint8_t a = gpio_get_level(ENC_A);
    uint8_t b = gpio_get_level(ENC_B);

    enc.last_state = (a << 1) | b;
}

int8_t get_encoder_val(void) {
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
        return -1;
    }

    if (enc.count <= -2) {
        enc.count += 2;
        return 1;
    }
    return 0;
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

void mx8650_next_cpi(void) {
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

static bool mx8650_is_in_sleep(void) {
uint8_t s = mx8650_read_reg(OPERATION_STATE_REG);
    return (s & STATE_OPSTATE_MASK) == STATE_SLEEP;
}

//ble HID 
void send_mouse_report(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel, int8_t hwheel) {
    if (!ble_connected || s_ble_hid_param.hid_dev == NULL) return;

    xSemaphoreTake(ble_hid_mutex, portMAX_DELAY);
    s_ble_hid_param.buffer[0] = buttons;
    s_ble_hid_param.buffer[1] = (uint8_t)dx;
    s_ble_hid_param.buffer[2] = (uint8_t)dy;
    s_ble_hid_param.buffer[3] = (uint8_t)wheel;
    s_ble_hid_param.buffer[4] = (uint8_t)hwheel;

    esp_hidd_dev_input_set(s_ble_hid_param.hid_dev, MOUSE_MAP_INDEX, MOUSE_REPORT_ID, s_ble_hid_param.buffer, MOUSE_REPORT_SIZE);
    xSemaphoreGive(ble_hid_mutex);
}

void ble_hid_task_start_up(void) {
    ESP_LOGI(TAG, "ble_hid_task_start_up (no-op, polling task selalu jalan)");
}

void ble_hid_task_shut_down(void) {
    ESP_LOGI(TAG, "ble_hid_task_shut_down (no-op)");
}

// nimble gap + sm device peripheral only from esp-idf example esp_hid_gap.c 
#define GATT_SVR_SVC_HID_UUID 0x1812

static struct ble_hs_adv_fields s_adv_fields;
static struct ble_hs_adv_fields s_rsp_fields;

static esp_err_t esp_hid_ble_gap_adv_init(uint16_t appearance, const char *device_name) {
    static ble_uuid16_t uuid16;
    
    memset(&s_adv_fields, 0, sizeof(s_adv_fields));
    s_adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    
    uuid16 = (ble_uuid16_t)BLE_UUID16_INIT(GATT_SVR_SVC_HID_UUID);
    s_adv_fields.uuids16 = &uuid16;
    s_adv_fields.num_uuids16 = 1;
    s_adv_fields.uuids16_is_complete = 1;
    s_adv_fields.appearance = appearance;
    s_adv_fields.appearance_is_present = 1;
    ble_svc_gap_device_appearance_set(appearance);

    memset(&s_rsp_fields, 0, sizeof(s_rsp_fields));
    s_rsp_fields.tx_pwr_lvl_is_present = 1;
    s_rsp_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    s_rsp_fields.name = (const uint8_t *)device_name;
    s_rsp_fields.name_len = strlen(device_name);
    s_rsp_fields.name_is_complete = 1;

    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ID | BLE_SM_PAIR_KEY_DIST_ENC;
    ble_hs_cfg.sm_their_key_dist |= BLE_SM_PAIR_KEY_DIST_ID | BLE_SM_PAIR_KEY_DIST_ENC;
    return ESP_OK;
}

// gap event handler
static int nimble_hid_gap_event(struct ble_gap_event *event, void *arg) {
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "connection %s, status = %d",event->connect.status == 0 ? "estabilished" : "failed", event->connect.status);
            //request fast connection interval
            if (event->connect.status == 0) {
                struct ble_gap_upd_params params;
                params.itvl_min = 6;
                params.itvl_max = 12;
                params.latency = 0;
                params.supervision_timeout = 200;
                params.min_ce_len = 0;
                params.max_ce_len = 0;

                int update_rc = ble_gap_update_params(event->connect.conn_handle, &params);
                ESP_LOGI(TAG, "Request fast connection params... rc=%d", update_rc);
            } else {
                rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
                if (rc == 0) {
                    ble_store_util_delete_peer(&desc.peer_id_addr);
                }
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            s_conn_handle = 0;
            ESP_LOGI(TAG, "disconnect reason=%d", event->disconnect.reason);
            return 0;

        case BLE_GAP_EVENT_ENC_CHANGE:
            ESP_LOGI(TAG, "encryption change event status = %d", event->enc_change.status); 
            ble_hid_task_start_up();
            return 0;

        case BLE_GAP_EVENT_REPEAT_PAIRING:
            rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
            if (rc == 0) {
                ble_store_util_delete_peer(&desc.peer_id_addr);
            }
            return BLE_GAP_REPEAT_PAIRING_RETRY;

        default:
            return 0;
    }
}

static esp_err_t esp_hid_ble_gap_adv_start(void) {
    int rc;
    struct ble_gap_adv_params adv_params;
    int32_t adv_duration_ms = 180000;

    rc = ble_gap_adv_set_fields(&s_adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting advertisement data rc=%d", rc);
        return ESP_FAIL;
    }

    rc = ble_gap_adv_rsp_set_fields(&s_rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting scan response data rc=%d", rc);
        return ESP_FAIL;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(30);
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(60);

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, adv_duration_ms, &adv_params, nimble_hid_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "error enabling advertisement rc=%d", rc);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t esp_hid_gap_init(void) {
    esp_err_t ret;
    esp_bt_controller_config_t ble_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_mem_release failed %d", ret);
        return ret;
    }
    ret = esp_bt_controller_init(&ble_cfg);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_init failed %d", ret);
        return ret;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_enable failed %d", ret);
        return ret;
    }
    ret = esp_nimble_init();
    if (ret) {
        ESP_LOGE(TAG, "esp_nimble_init failed %d", ret);
        return ret;
    }
    return ESP_OK;
}

static void ble_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

    switch (event) {
        case ESP_HIDD_START_EVENT:
            ESP_LOGI(TAG, "HID START, mulai advertising");
            esp_hid_ble_gap_adv_start();
            break;

        case ESP_HIDD_CONNECT_EVENT:
            ESP_LOGI(TAG, "BLE CONNECTED");
            ble_connected = true;
            break;

        case ESP_HIDD_PROTOCOL_MODE_EVENT:
            ESP_LOGI(TAG, "Protocol mode ganti ka: %s",param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
            break;

        case ESP_HIDD_DISCONNECT_EVENT:
            ESP_LOGI(TAG, "BLE DISCONNECTED, advertising deui");
            ble_connected = false;
            esp_hid_ble_gap_adv_start();
            break;

        case ESP_HIDD_STOP_EVENT:
            ESP_LOGI(TAG, "HID_STOP");
            break;

        default:
            break;
    }
}

static void ble_hid_device_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task Mulai");
    nimble_port_run();
    nimble_port_freertos_deinit();
}
void ble_store_config_init(void);//helper

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

        if (report_needed && ble_connected) {
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
        .light_sleep_enable = false
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_cfg));
    ESP_LOGI(TAG, "CPU PM %d Mhz max / %d Mhz min", pm_cfg.max_freq_mhz, pm_cfg.min_freq_mhz);
    
    btn_init();
    enc_init();
    mx8650_init();
    mx8650_set_sleep_mode(MX8650_SLEEP1_SLEEP2);
    ESP_LOGI(TAG, "Tick rate: %d Hz, 1 tick = %d ms", configTICK_RATE_HZ, portTICK_PERIOD_MS);
    
    //bt init
    ble_hid_mutex = xSemaphoreCreateMutex();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret ==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //init 
    ESP_ERROR_CHECK(esp_hid_gap_init());
    ESP_ERROR_CHECK(esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_MOUSE, ble_hid_cfg.device_name));
    ESP_LOGI(TAG, "Starting BLE HID Device...");
    ESP_ERROR_CHECK(esp_hidd_dev_init(&ble_hid_cfg, ESP_HID_TRANSPORT_BLE, ble_hidd_event_callback, &s_ble_hid_param.hid_dev));

    ble_svc_gap_device_name_set(ble_hid_cfg.device_name);

    ble_store_config_init();
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    nimble_port_freertos_init(ble_hid_device_host_task);

    //polling task
    xTaskCreate(mouse_polling_task, "POLLING TASK", 4096, NULL, 5, NULL);
}

