#ifndef SLIMENRF_GLOBALS
#define SLIMENRF_GLOBALS

#include <zephyr/logging/log.h>
#include <esb.h>
#include <zephyr/drivers/i2c.h>
#include <nrfx_timer.h>

#include "retained.h"

extern struct esb_payload tx_payload;

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define LED0_NODE DT_NODELABEL(pwm_led0)

#define MAIN_IMU_NODE DT_NODELABEL(icm_0)
#define MAIN_MAG_NODE DT_NODELABEL(mmc_0)

#define CLKOUT_NODE DT_NODELABEL(pwmclock)

extern int tickrate;

extern uint8_t batt;
extern uint8_t batt_v;
extern uint32_t batt_pptt;
extern bool batt_low;

extern const struct gpio_dt_spec dock;

extern bool threads_running;
extern bool main_running;
extern bool main_ok;
extern bool main_data;

#define USER_SHUTDOWN_ENABLED true
#define MAG_ENABLED true

// TODO: only scan/detect new imus on reset event, write to nvs

extern float q3[4]; // correction quat

extern int tracker_id;

extern int64_t led_time;
extern int64_t led_time_off;
extern uint8_t reset_mode;
extern uint8_t last_reset;
extern bool system_off_main;
//extern bool charging;

#define LAST_RESET_LIMIT 10

extern volatile uint32_t m_counter;
extern const nrfx_timer_t m_timer;

extern bool esb_state;
extern bool timer_state;
extern bool send_data;

extern uint16_t led_clock;
extern uint32_t led_clock_offset;

extern struct gpio_callback button_cb_data;

#endif