#pragma once

#include <stdint.h>
#include <stdbool.h>

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

static const uint16_t cpi_table[] = {800, 1000, 1200, 1600};

void mx8650_init(void);
bool mx8650_read_motion(int8_t *dx, int8_t *dy);
void mx8650_set_sleep_mode(mx8650_sleep_mode_t mode);
void mx8650_print_state(void);
void mx8650_set_motswk(mx8650_motswk_mode_t mode);
bool mx8650_is_in_sleep(void);
void dpi_sm_update(uint8_t btn_state);

