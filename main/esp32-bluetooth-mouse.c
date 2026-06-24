#include <stdio.h>
#include <string.h>
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hidd_api.h"

const uint8_t MOTSWK = 4;
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

#define IDLE_TIMEOUT_MS 65000


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

static dpi_state_t dpi_state = DPI_IDLE;
static uint32_t hold_start = 0;

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
    ESP_ACTIVE,
    ESP_LIGHT_SLEEP,
} esp_power_state_t;

typedef struct {
    uint8_t last_state;
    int32_t count;
} encoder_t;

static encoder_t enc = {0};

static const int8_t enc_table[16] = {
    0, -1, +1, 0, 
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0
};

static const char *TAG = "MX8650_Mouse_Test";
static const uint16_t cpi_table[] = {800, 1000, 1200, 1600};
static cpi_t current_cpi = CPI_800;
static esp_power_state_t esp_state = ESP_ACTIVE;
static uint32_t last_activity_ms = 0;

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
#define REPORT_PROTOCOL_MOUSE_REPORT_SIZE 5

//lokal parameter struct  
typedef struct{
    esp_hidd_app_param_t app_param;
    esp_hidd_qos_param_t both_qos;
    uint8_t protocol_mode;
    uint8_t buffer[8];
} local_param_t;

static local_param_t s_local_param = {0};
static SemaphoreHandle_t bt_hid_mutex = NULL;
static bool bt_connected = false;

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

void motswk_init(void) {
    gpio_reset_pin(MOTSWK);
    gpio_set_direction(MOTSWK, GPIO_MODE_INPUT);
    gpio_set_pull_mode(MOTSWK, GPIO_PULLUP_ONLY);// SWKINT active low
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
        return 1;
    }

    if (enc.count <= -2) {
        enc.count += 2;
        return -1;
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
                ESP_LOGI(TAG, "DPI IDLE");
            } else if ((now - hold_start) >= 2000) {
                mx8650_next_cpi();
                dpi_state = DPI_WAIT_RELEASE;
                ESP_LOGI(TAG, "DPI Berubah");
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

bool mx8650_is_sleeping(void) {
    uint8_t s = mx8650_read_reg(OPERATION_STATE_REG);
    return ((s & STATE_OPSTATE_MASK) == STATE_SLEEP);
}

void mx8650_force_wakeup(void) {
    uint8_t om = mx8650_read_reg(OPERATION_MODE_REG);
    om |= (1 << 0);
    mx8650_write_reg(OPERATION_MODE_REG, om);
}

void enter_light_sleep(void) {
    ESP_LOGI(TAG, "Entering light sleep...");

    vTaskDelay(pdMS_TO_TICKS(200));
    mx8650_print_state();

    esp_sleep_enable_ext0_wakeup(MOTSWK, 0);

    esp_state = ESP_LIGHT_SLEEP;
    esp_light_sleep_start();

    //resume here after wakeup 
    esp_state = ESP_ACTIVE;
    ESP_LOGI(TAG, "Wakeup from light sleep cause: %d", esp_sleep_get_wakeup_cause());
    last_activity_ms = esp_timer_get_time() /1000;

}

//bt HID 
void send_mouse_report(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel, int8_t hwheel) {
    if (!bt_connected) return;

    xSemaphoreTake(bt_hid_mutex, portMAX_DELAY);

    uint8_t report_id;
    uint16_t report_size;

    if (s_local_param.protocol_mode == ESP_HIDD_REPORT_MODE) {
        report_id = 0;
        report_size = REPORT_PROTOCOL_MOUSE_REPORT_SIZE;
        s_local_param.buffer[0] = buttons;
        s_local_param.buffer[1] = dx;
        s_local_param.buffer[2] = dy;
        s_local_param.buffer[3] = wheel;
        s_local_param.buffer[4] = hwheel;
    } else {
        //boot mode
        report_id = ESP_HIDD_BOOT_REPORT_ID_MOUSE;
        report_size = ESP_HIDD_BOOT_REPORT_SIZE_MOUSE - 1;
        s_local_param.buffer[0] = buttons;
        s_local_param.buffer[1] = dx;
        s_local_param.buffer[2] = dy;
    }

    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, report_id, report_size, s_local_param.buffer);
    xSemaphoreGive(bt_hid_mutex);
}

static void esp_bt_hidd_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param) {
    switch (event) {
    case ESP_HIDD_INIT_EVT:
        if (param->init.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "HID Init sukses, regiistering app...");
            esp_bt_hid_device_register_app(&s_local_param.app_param, &s_local_param.both_qos, &s_local_param.both_qos);
        } else {
            ESP_LOGE(TAG, "HID init Gagal kehed!");
        }
        break;

    case ESP_HIDD_REGISTER_APP_EVT:
        if (param->register_app.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "HIDD app registered, setting connectable/discoverable...");
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        } else {
            ESP_LOGE(TAG, "HID app register gagal!");
        }
        break;

    case ESP_HIDD_OPEN_EVT:
        if (param->open.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "Connected to: %02x:%02x:%02x:%02x:%02x:%02x", param->open.bd_addr[0], param->open.bd_addr[1], param->open.bd_addr[2], param->open.bd_addr[3], param->open.bd_addr[4], param->open.bd_addr[5]);
            bt_connected = true;
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        } else {
            ESP_LOGE(TAG, "Connection gagal, status : %d", param->open.status);
        }
        ESP_LOGW(TAG, "HID OPEN");
        break;

    case ESP_HIDD_CLOSE_EVT:
        ESP_LOGI(TAG, "Disconected");
        bt_connected = false;
        ESP_LOGW(TAG, "HID CLOSE!");
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;

    case ESP_HIDD_SEND_REPORT_EVT:
        //report berhasil
        break;

    case ESP_HIDD_SET_PROTOCOL_EVT:
        ESP_LOGI(TAG, "Protocol mode ganti ka: %s", param->set_protocol.protocol_mode == ESP_HIDD_BOOT_MODE ? "BOOT" : "REPORT");
        xSemaphoreTake(bt_hid_mutex, portMAX_DELAY);
        s_local_param.protocol_mode = param->set_protocol.protocol_mode;
        xSemaphoreGive(bt_hid_mutex);
        break;

    default:
        break;
    }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Auth berhasil: %s", param->auth_cmpl.device_name);
        } else {
            ESP_LOGE(TAG, "Auth gagal, status: %s", param->auth_cmpl.stat);
        }
        break;

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(TAG, "ganti mode: %d", param->mode_chg.mode);
        break;

    default:
        break;
    }
}

void mouse_polling_task(void *pvParameters) {
    uint8_t last_btn = 0;
    last_activity_ms = esp_timer_get_time() / 1000;

    while (1) {
        bool report_needed = false;
        int8_t dx = 0, dy = 0;
        bool had_activity = false;
        
        if (mx8650_read_motion(&dx, &dy)) {
            report_needed = true;
            had_activity = true;
            ESP_LOGI(TAG, "MOVE -> X: %d, Y: %d", dx, dy);
        }

        uint8_t btn = read_btn();
        if (btn != last_btn) {
            report_needed = true;
            had_activity = true;
            last_btn = btn;
        }
        if (btn & 0x01) ESP_LOGI(TAG, "LEFT CLICK");
        if (btn & 0x02) ESP_LOGI(TAG, "RIGHT CLICK");
        if (btn & 0x04) ESP_LOGI(TAG, "MIDDLE CLICK");
        if (btn & 0x08) ESP_LOGI(TAG, "THUMB BACK");
        if (btn & 0x10) ESP_LOGI(TAG, "THUMB FORWARD");
        
        int8_t wheel = get_encoder_val();
        if (wheel != 0) {
            report_needed = true;
            had_activity = true;
            ESP_LOGI(TAG, "SCROLL: %d", wheel);
        }

        int8_t hwheel = read_tilt();
        if (hwheel != 0) {
            report_needed = true;
            had_activity = true;
            ESP_LOGI(TAG, "TILT : %d",hwheel);
        }

        dpi_sm_update(btn);
        if (report_needed && bt_connected) {
            send_mouse_report(btn, dx, dy, wheel, hwheel);
            ESP_LOGI(TAG, "REPORTED");
        }
        uint32_t now = esp_timer_get_time() / 1000;

        switch (esp_state) {
            case ESP_ACTIVE:
                if (had_activity) {
                    last_activity_ms = now;
                }
                if (now - last_activity_ms > IDLE_TIMEOUT_MS) {
                    enter_light_sleep();
                }
                break;
            case ESP_LIGHT_SLEEP:
                //sleep
                break;
        }
        if (had_activity) {
            vTaskDelay(1);
        
        } else {
            vTaskDelay(5);
        }
    }   
}

void app_main(void){
    ESP_LOGI(TAG, "MX8650 MOUSE TEST");
    btn_init();
    enc_init();
    mx8650_init();
    mx8650_set_sleep_mode(MX8650_SLEEP1_SLEEP2);
    mx8650_set_motswk(MX8650_MOTSWK_SWKINT);
    motswk_init();
    
    //bt init
    bt_hid_mutex = xSemaphoreCreateMutex();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret ==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //bt init controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    //init bluedroid
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_bt_gap_set_device_name("ESP32 MX8650 Mouse");

    //set class of device peripheral
    esp_bt_cod_t cod;
    cod.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL;
    ESP_ERROR_CHECK(esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_MAJOR_MINOR));

    //init hid parameter
    s_local_param.app_param.name = "Mouse";
    s_local_param.app_param.description = "DIY ESP32 MX8650 Optical Mouse";
    s_local_param.app_param.provider = "ESP32";
    s_local_param.app_param.subclass = ESP_HID_CLASS_MIC; //mouse subclass
    s_local_param.app_param.desc_list = (uint8_t *)hid_mouse_report_desc;
    s_local_param.app_param.desc_list_len = HID_MOUSE_REPORT_DESC_LEN;
    memset(&s_local_param.both_qos, 0, sizeof(esp_hidd_qos_param_t));
    s_local_param.protocol_mode = ESP_HIDD_REPORT_MODE;

    //reg callbacks
    esp_bt_gap_register_callback(esp_bt_gap_cb);
    esp_bt_hid_device_register_callback(esp_bt_hidd_cb);

    //starting hid device
    ESP_LOGI(TAG, "Starting BT HID Device..");
    esp_bt_hid_device_init();

    //security setting (pairing tanpa pin) 
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    //print bt address
    const uint8_t *addr = esp_bt_dev_get_address();
    ESP_LOGI(TAG, "BT Adress: %02x:%02x:%02x:%02x:%02x:%02x", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    ESP_LOGI(TAG, "Device discoverable. sambungken entos aya di komputer maneh");

    //polling task
    xTaskCreate(mouse_polling_task, "POLLING TASK", 4096, NULL, 5, NULL);
}

