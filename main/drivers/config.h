#pragma once

//gpio definition
#define SCLK 18
#define SDIO 23
#define BTN_L 21
#define BTN_R 33
#define BTN_M 32
#define BTN_ML 22
#define BTN_MR 27
#define BTN_TF 5
#define BTN_TB 19
#define ENC_A 25
#define ENC_B 26

#define BAT_ADC_CH ADC_CHANNEL_6

//mx8650 address reg & instruction 
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
