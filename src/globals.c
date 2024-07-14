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
//#include "../vqf-c/src/vqf.h"
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

uint32_t* dbl_reset_mem = ((uint32_t*) DFU_DBL_RESET_MEM);

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

#include "retained.h"

//LOG_MODULE_REGISTER(main, 4);

struct esb_payload rx_payload;
struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0);

#define _RADIO_SHORTS_COMMON                                       \
	(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | \
	 RADIO_SHORTS_ADDRESS_RSSISTART_Msk |                          \
	 RADIO_SHORTS_DISABLED_RSSISTOP_Msk)

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define LED0_NODE DT_NODELABEL(pwm_led0)

#define MAIN_IMU_NODE DT_NODELABEL(icm_0)
#define MAIN_MAG_NODE DT_NODELABEL(mmc_0)

int tickrate = 6;

uint8_t batt;
uint8_t batt_v;
uint32_t batt_pptt;
bool batt_low = false;

const struct i2c_dt_spec main_imu = I2C_DT_SPEC_GET(MAIN_IMU_NODE);
const struct i2c_dt_spec main_mag = I2C_DT_SPEC_GET(MAIN_MAG_NODE);

const struct gpio_dt_spec dock = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, dock_gpios);

bool threads_running = false;

bool main_running = false;

bool main_ok = false;

bool main_data = false;

#define USER_SHUTDOWN_ENABLED true
#define MAG_ENABLED true

// TODO: only scan/detect new imus on reset event, write to nvs

float lin_ax, lin_ay, lin_az;					// linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};			// vector to hold quaternion
float last_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};		// vector to hold quaternion

float gOff[3] = {0.0f, 0.0f, 0.0f}; // runtime fusion gyro offset

float q3[4] = {0.5f, -0.5f, -0.5f, -0.5f}; // correction quat

FusionOffset offset; // could share goff and q with fusionoffset and fusionahrs but init clears the values
FusionAhrs ahrs;

FusionVector gyro_sanity_m;
int gyro_sanity = 0;

int magCal = 0;
int last_magCal = 0;
int64_t magCal_time = 0;
double ata[100] = {0.0}; // init cal
double norm_sum = 0.0;
double sample_count = 0.0;

int tracker_id = 0;

// storing temporary values
uint16_t tx_buf[7];
//int64_t start_time;
int64_t last_data_time;
unsigned int last_batt_pptt[16] = {10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001};
int8_t last_batt_pptt_i = 0;
int64_t led_time = 0;
int64_t led_time_off = 0;
uint8_t reset_mode = -1;
uint8_t last_reset = 0;
bool system_off_main = false;
bool reconfig;
//bool charging = false;

#define LAST_RESET_LIMIT 10

// TODO: move to sensor
float magBAinv[4][3];

#include <nrfx_timer.h>

volatile uint32_t m_counter = 1;
const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1);

bool esb_state = false;
bool timer_state = false;
bool send_data = false;

uint16_t led_clock = 0;
uint32_t led_clock_offset = 0;

int powerstate = 0;
int last_powerstate = 0;
int mag_level = 0;
int last_mag_level = -1;

struct gpio_callback button_cb_data;
