#pragma once

#include "config.h"
#include <stdint.h>

typedef struct {
    uint8_t last_state;
    int32_t count;
} encoder_t;

void enc_init(void);
int8_t get_encoder_val(void);
