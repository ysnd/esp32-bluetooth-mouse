#include "drivers/config.h"
#include "drivers/battery.h"
#include "ble_hid.h"
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
#include "services/bas/ble_svc_bas.h"

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

static const char *TAG = "BLE_HID";
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
extern void ble_store_config_init(void);//helper

void ble_hid_init(void) {
    //bt init
    ble_hid_mutex = xSemaphoreCreateMutex(); 

    //init 
    ESP_ERROR_CHECK(esp_hid_gap_init());
    ESP_ERROR_CHECK(esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_MOUSE, ble_hid_cfg.device_name));
    ESP_LOGI(TAG, "Starting BLE HID Device...");
    ESP_ERROR_CHECK(esp_hidd_dev_init(&ble_hid_cfg, ESP_HID_TRANSPORT_BLE, ble_hidd_event_callback, &s_ble_hid_param.hid_dev));

    ble_svc_gap_device_name_set(ble_hid_cfg.device_name);

    ble_store_config_init();
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    
    ble_svc_bas_init();

    nimble_port_freertos_init(ble_hid_device_host_task);
}

bool ble_is_connected(void) {
    return ble_connected;
}

void ble_bas_update(void) {
    battery_info_t batt = battery_get_info();
    
    ESP_LOGD(TAG, "Battery %.3f V (%u%%)", batt.voltage_mv / 1000.0f, batt.percent);
    ble_svc_bas_battery_level_set(batt.percent);
}





