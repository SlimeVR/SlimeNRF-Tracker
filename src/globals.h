#ifndef SLIMENRF_GLOBALS
#define SLIMENRF_GLOBALS

#include <zephyr/types.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>

#include <zephyr/settings/settings.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "battery.h"

#include "../Fusion/Fusion/Fusion.h"
#include "Fusion/FusionOffset2.h"
#include "../vqf-c/src/vqf.h"
#include "magneto/magneto1_4.h"

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include <zephyr/init.h>
#include <hal/nrf_gpio.h>

#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/reboot.h>

#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e

extern uint32_t* dbl_reset_mem;

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

#include "retained.h"

extern struct esb_payload rx_payload;
extern struct esb_payload tx_payload;

extern struct esb_payload tx_payload_pair;

#define _RADIO_SHORTS_COMMON                                       \
	(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | \
	 RADIO_SHORTS_ADDRESS_RSSISTART_Msk |                          \
	 RADIO_SHORTS_DISABLED_RSSISTOP_Msk)

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define LED0_NODE DT_NODELABEL(pwm_led0)

#define MAIN_IMU_NODE DT_NODELABEL(icm_0)
#define MAIN_MAG_NODE DT_NODELABEL(mmc_0)

// this was randomly generated
extern uint8_t discovery_base_addr_0[4];
extern uint8_t discovery_base_addr_1[4];
extern uint8_t discovery_addr_prefix[8];

extern uint8_t paired_addr[8];

extern uint8_t base_addr_0[4];
extern uint8_t base_addr_1[4];
extern uint8_t addr_prefix[8];

extern int tickrate;

extern uint8_t batt;
extern uint8_t batt_v;
extern uint32_t batt_pptt;
extern bool batt_low;

extern const struct i2c_dt_spec main_imu;
extern const struct i2c_dt_spec main_mag;

extern const struct gpio_dt_spec dock;

extern bool threads_running;

extern bool main_running;

extern bool main_ok;

extern bool main_data;

#define USER_SHUTDOWN_ENABLED true
#define MAG_ENABLED true

// TODO: only scan/detect new imus on reset event, write to nvs

extern float lin_ax, lin_ay, lin_az;					// linear acceleration (acceleration with gravity component subtracted)
extern float q[4];			// vector to hold quaternion
extern float last_q[4];		// vector to hold quaternion

extern float gOff[3]; // runtime fusion gyro offset

extern float q3[4]; // correction quat

extern FusionOffset offset; // could share goff and q with fusionoffset and fusionahrs but init clears the values
extern FusionAhrs ahrs;

extern FusionVector gyro_sanity_m;
extern int gyro_sanity;

extern int magCal;
extern int last_magCal;
extern int64_t magCal_time;
extern double ata[100]; // init cal
extern double norm_sum;
extern double sample_count;

extern int tracker_id;

// storing temporary values
extern uint16_t tx_buf[7];
//extern int64_t start_time;
extern int64_t last_data_time;
extern unsigned int last_batt_pptt[16];
extern int8_t last_batt_pptt_i;
extern int64_t led_time;
extern int64_t led_time_off;
extern uint8_t reset_mode;
extern uint8_t last_reset;
extern bool system_off_main;
extern bool reconfig;
//extern bool charging;

#define LAST_RESET_LIMIT 10

// TODO: move to sensor
extern float magBAinv[4][3];

#include <nrfx_timer.h>

extern volatile uint32_t m_counter;
extern const nrfx_timer_t m_timer;

extern bool esb_state;
extern bool timer_state;
extern bool send_data;

extern uint16_t led_clock;
extern uint32_t led_clock_offset;

extern int powerstate;
extern int last_powerstate;
extern int mag_level;
extern int last_mag_level;

extern struct gpio_callback button_cb_data;

#endif