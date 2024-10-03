#include <zephyr/drivers/gpio.h>

#include "globals.h"

float q3[4] = {0.5f, -0.5f, -0.5f, -0.5f}; // correction quat

//int64_t led_time = 0;
//int64_t led_time_off = 0;
uint8_t last_reset = 0;

volatile uint32_t m_counter = 1;
const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1);

bool esb_state = false;
bool timer_state = false;
bool send_data = false;

uint16_t led_clock = 0;
uint32_t led_clock_offset = 0;
