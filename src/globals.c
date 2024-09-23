#include <zephyr/drivers/gpio.h>

#include "globals.h"

int tickrate = 6;

// TODO: move to sys
uint8_t batt;
uint8_t batt_v;

// TODO: move to sensor
bool main_running = false;
bool main_ok = false;
bool main_data = false;

float q3[4] = {0.5f, -0.5f, -0.5f, -0.5f}; // correction quat

// TODO: move to esb or connection
int tracker_id = 0;

//int64_t led_time = 0;
//int64_t led_time_off = 0;
uint8_t last_reset = 0;
bool system_off_main = false; // TODO: move to sys

volatile uint32_t m_counter = 1;
const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1);

bool esb_state = false;
bool timer_state = false;
bool send_data = false;

uint16_t led_clock = 0;
uint32_t led_clock_offset = 0;
