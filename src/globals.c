#include <zephyr/drivers/gpio.h>

#include "globals.h"

struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

int tickrate = 6;

uint8_t batt;
uint8_t batt_v;
uint32_t batt_pptt;
bool batt_low = false;

const struct gpio_dt_spec dock = GPIO_DT_SPEC_GET_OR(ZEPHYR_USER_NODE, dock_gpios, {0});

bool threads_running = false;
bool main_running = false;
bool main_ok = false;
bool main_data = false;

// TODO: only scan/detect new imus on reset event, write to nvs

float q3[4] = {0.5f, -0.5f, -0.5f, -0.5f}; // correction quat

int tracker_id = 0;

//int64_t led_time = 0;
//int64_t led_time_off = 0;
uint8_t reset_mode = -1;
uint8_t last_reset = 0;
bool system_off_main = false;
//bool charging = false;

volatile uint32_t m_counter = 1;
const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1);

bool esb_state = false;
bool timer_state = false;
bool send_data = false;

uint16_t led_clock = 0;
uint32_t led_clock_offset = 0;

struct gpio_callback button_cb_data;
