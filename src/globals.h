#ifndef SLIMENRF_GLOBALS
#define SLIMENRF_GLOBALS

#include <zephyr/logging/log.h>
#include <esb.h>
#include <zephyr/drivers/i2c.h>
#include <nrfx_timer.h>

#include "retained.h"

#define USER_SHUTDOWN_ENABLED true // Allow user to use reset or sw0 to shutdown
#define MAG_ENABLED true // Use magnetometer if it is present
#define IGNORE_RESET true // If sw0 available, don't change any reset behavior
//#define WOM_USE_DCDC true // Use DCDC instead of LDO for WOM if it is more efficient

extern float q3[4]; // correction quat

extern int64_t led_time;
extern int64_t led_time_off;
extern uint8_t last_reset;

#define LAST_RESET_LIMIT 10

extern volatile uint32_t m_counter;
extern const nrfx_timer_t m_timer;

extern bool esb_state;
extern bool timer_state;
extern bool send_data;

extern uint16_t led_clock;
extern uint32_t led_clock_offset;

#endif