#include <zephyr/types.h>
#include <stddef.h>
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

#include "Fusion/Fusion.h"
#include "magneto1_4.h"

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
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <hal/nrf_gpio.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>

#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

#include "retained.h"

LOG_MODULE_REGISTER(main, 4);

static struct nvs_fs fs;

#define NVS_PARTITION		storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define RBT_CNT_ID 1
#define PAIRED_ID 2
#define MAIN_ACCEL_BIAS_ID 3
#define MAIN_GYRO_BIAS_ID 4
#define MAIN_MAG_BIAS_ID 5
#define AUX_ACCEL_BIAS_ID 6
#define AUX_GYRO_BIAS_ID 7
#define AUX_MAG_BIAS_ID 8

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0);

#define _RADIO_SHORTS_COMMON                                       \
	(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | \
	 RADIO_SHORTS_ADDRESS_RSSISTART_Msk |                          \
	 RADIO_SHORTS_DISABLED_RSSISTOP_Msk)

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define LED0_NODE DT_NODELABEL(pwm_led0)

#define MAIN_IMU_NODE DT_NODELABEL(icm_0)
#define MAIN_MAG_NODE DT_NODELABEL(mmc_0)
#define AUX_IMU_NODE DT_NODELABEL(icm_1)
#define AUX_MAG_NODE DT_NODELABEL(mmc_1)

// this was randomly generated
uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

uint8_t paired_addr[8] = {0,0,0,0,0,0,0,0};

uint8_t base_addr_0[4] = {0,0,0,0};
uint8_t base_addr_1[4] = {0,0,0,0};
uint8_t addr_prefix[8] = {0,0,0,0,0,0,0,0};

int tickrate = 6;

uint8_t batt;
uint8_t batt_v;
unsigned int batt_pptt;

const struct i2c_dt_spec main_imu = I2C_DT_SPEC_GET(MAIN_IMU_NODE);
const struct i2c_dt_spec main_mag = I2C_DT_SPEC_GET(MAIN_MAG_NODE);
const struct i2c_dt_spec aux_imu = I2C_DT_SPEC_GET(AUX_IMU_NODE);
const struct i2c_dt_spec aux_mag = I2C_DT_SPEC_GET(AUX_MAG_NODE);

//const struct pwm_dt_spec led0 = PWM_DT_SPEC_GET(LED0_NODE);
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_gpios);

const struct gpio_dt_spec dock = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, dock_gpios);
//const struct gpio_dt_spec chgstat = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, chgstat_gpios);
//const struct gpio_dt_spec int0 = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, int0_gpios);
//const struct gpio_dt_spec int1 = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, int1_gpios);

bool threads_running = false;

bool main_running = false;
bool aux_running = false;

bool main_ok = false;
bool aux_ok = false;
bool aux_exists = true;

bool main_data = false;
bool aux_data = false;

#define MAG_ENABLED true

#define INT16_TO_UINT16(x) ((uint16_t)32768 + (uint16_t)(x))
#define TO_FIXED_14(x) ((int16_t)((x) * (1 << 14)))
#define TO_FIXED_10(x) ((int16_t)((x) * (1 << 10)))
#define FIXED_14_TO_DOUBLE(x) (((double)(x)) / (1 << 14))
#define FIXED_10_TO_DOUBLE(x) (((double)(x)) / (1 << 10))

// only scan/detect new imus on reset event, write to nvs

typedef void (*snrf_error_cb_t)(int error);

struct snrf_sensor { // some stuff for imus and mags, sensor define should also include capabilities (quat/imu/mag/aos, has temp, has fifo, fifo buffer size, has WOM)
	snrf_error_cb_t init; // this does a lot of stuff automatically, starts fifo
	snrf_error_cb_t mode_set; // (ln = 0, lp = 1, also apply mode)
	snrf_error_cb_t quat_read; // if applicable
	snrf_error_cb_t accel_read; // return scaled floats
	snrf_error_cb_t gyro_read; // return scaled floats
	snrf_error_cb_t mag_read; // return scaled floats
	snrf_error_cb_t temp_read; // return if no temp
	snrf_error_cb_t fifo_length; // return if no fifo
	snrf_error_cb_t fifo_read; // (arrays, how much to read)
	snrf_error_cb_t fifo_parse; // (return if packet empty, otherwise write accel/gyro/mag if applicable)
	snrf_error_cb_t reset; // includes delay
	snrf_error_cb_t reset_fast; // not including delay
	snrf_error_cb_t setup_WOM; // reset then setup WOM, return if not applicable
	void *config;
};


#include "ICM42688.h"
#include "MMC5983MA.h"

float lin_ax, lin_ay, lin_az;					// linear acceleration (acceleration with gravity component subtracted)
float lin_ax2, lin_ay2, lin_az2;					// linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};			// vector to hold quaternion
float q2[4] = {1.0f, 0.0f, 0.0f, 0.0f};			// vector to hold quaternion
float last_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};		// vector to hold quaternion
float last_q2[4] = {1.0f, 0.0f, 0.0f, 0.0f};	// vector to hold quaternion

float q3[4] = {-0.5f, 0.5f, 0.5f, 0.5f}; // correction quat

FusionOffset offset;
FusionAhrs ahrs;
FusionOffset offset2;
FusionAhrs ahrs2;

int tracker_id = 0;

// storing temporary values
uint16_t tx_buf[7];
int64_t start_time;
int64_t last_data_time;
unsigned int last_batt_pptt[16] = {10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001};
int8_t last_batt_pptt_i = 0;
int64_t led_time = 0;
int64_t led_time_off = 0;
uint8_t reset_mode = 0;
uint8_t last_reset = 0;
bool system_off_main = false;
bool system_off_aux = false;
bool reconfig;
//bool charging = false;

#define LAST_RESET_LIMIT 10

// TODO: move to sensor
// ICM42688 definitions

// TODO: move to sensor
/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
	  AFS_2G, AFS_4G, AFS_8G, AFS_16G
	  GFS_15_625DPS, GFS_31_25DPS, GFS_62_5DPS, GFS_125DPS, GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
	  AODR_1_5625Hz, AODR_3_125Hz, AODR_6_25Hz, AODR_50AODR_12_5Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_500Hz,
	  AODR_1kHz, AODR_2kHz, AODR_4kHz, AODR_8kHz, AODR_16kHz, AODR_32kHz
	  GODR_12_5Hz, GODR_25Hz, GODR_50Hz, GODR_100Hz, GODR_200Hz, GODR_500Hz, GODR_1kHz, GODR_2kHz, GODR_4kHz, GODR_8kHz, GODR_16kHz, GODR_32kHz
*/
uint8_t Ascale = AFS_8G, Gscale = GFS_2000DPS, AODR = AODR_200Hz, GODR = GODR_1kHz, aMode = aMode_LN, gMode = gMode_LN;
#define INTEGRATION_TIME 0.001
#define INTEGRATION_TIME_LP 0.005

// TODO: make sure these are separate for main vs. aux (and also store/read them!)
float accelBias[3] = {0.0f, 0.0f, 0.0f}, gyroBias[3] = {0.0f, 0.0f, 0.0f}; // offset biases for the accel and gyro
float accelBias2[3] = {0.0f, 0.0f, 0.0f}, gyroBias2[3] = {0.0f, 0.0f, 0.0f}; // offset biases for the accel and gyro

// MMC5983MA definitions

// TODO: move to sensor
/* Specify sensor parameters (continuous mode sample rate is dependent on bandwidth)
 * choices are: MODR_ONESHOT, MODR_1Hz, MODR_10Hz, MODR_20Hz, MODR_50 Hz, MODR_100Hz, MODR_200Hz (BW = 0x01), MODR_1000Hz (BW = 0x03)
 * Bandwidth choices are: MBW_100Hz, MBW_200Hz, MBW_400Hz, MBW_800Hz
 * Set/Reset choices are: MSET_1, MSET_25, MSET_75, MSET_100, MSET_250, MSET_500, MSET_1000, MSET_2000, so MSET_100 set/reset occurs every 100th measurement, etc.
 */
uint8_t MODR = MODR_200Hz, MBW = MBW_400Hz, MSET = MSET_2000;

// TODO: move to sensor
// TODO: make sure these are separate for main vs. aux (and also store/read them!)
float magBAinv[4][3];
float magBAinv2[4][3];

void q_multiply(float *x, float *y, float *out) {
	out[0] = x[0]*y[0] - x[1]*y[1] - x[2]*y[2] - x[3]*y[3];
	out[1] = x[1]*y[0] + x[0]*y[1] - x[3]*y[2] + x[2]*y[3];
	out[2] = x[2]*y[0] + x[3]*y[1] + x[0]*y[2] - x[1]*y[3];
	out[3] = x[3]*y[0] - x[2]*y[1] + x[1]*y[2] + x[0]*y[3];
}

#include <nrfx_timer.h>

static volatile uint32_t m_counter = 1;
static const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1);

bool esb_state = false;
bool timer_state = false;

static void timer_handler(nrf_timer_event_t event_type, void *p_context) {
	if (event_type == NRF_TIMER_EVENT_COMPARE1 && esb_state == true) {
		if (last_reset < LAST_RESET_LIMIT) {
			last_reset++;
			esb_start_tx();
		} else {
			esb_disable();
			esb_initialize_rx();
			esb_state = false;
			nrfx_timer_pause(&m_timer);
			timer_state = false;
		}
	} else if (event_type == NRF_TIMER_EVENT_COMPARE2 && esb_state == true) {
		esb_disable();
		esb_initialize_rx();
		esb_state = false;
	} else if (event_type == NRF_TIMER_EVENT_COMPARE3 && esb_state == false) {
		esb_disable();
		esb_initialize();
		esb_state = true;
	}
}

void timer_init(void) {
    nrfx_err_t err;
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;  
	timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
    //timer_cfg.mode = NRF_TIMER_MODE_TIMER;
    //timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_16;
    //timer_cfg.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY;
    //timer_cfg.p_context = NULL;
	nrfx_timer_init(&m_timer, &timer_cfg, timer_handler);
    uint32_t ticks = nrfx_timer_ms_to_ticks(&m_timer, 3);
    nrfx_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
    nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL1, ticks * (paired_addr[1] + 3) / 21, true); // timeslot to send data
    nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL2, ticks * 19 / 21, true); // switch to rx
    nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL3, ticks * 2 / 21, true); // switch to tx
    nrfx_timer_enable(&m_timer);
	IRQ_DIRECT_CONNECT(TIMER1_IRQn, 0, nrfx_timer_1_irq_handler, 0);
	irq_enable(TIMER1_IRQn);
	timer_state = true;
}

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id)
	{
	case ESB_EVENT_TX_SUCCESS:
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_INF("TX FAILED");
		break;
	case ESB_EVENT_RX_RECEIVED:
		if (esb_read_rx_payload(&rx_payload) == 0) {
			if (paired_addr[0] == 0x00) {
				if (rx_payload.length == 8) {
					for (int i = 0; i < 8; i++) {
						paired_addr[i] = rx_payload.data[i];
					}
				}
			} else {
				if (rx_payload.length == 4) {
					if (timer_state == false) {
						nrfx_timer_resume(&m_timer);
						timer_state = true;
					}
					nrfx_timer_clear(&m_timer);
					last_reset = 0;
					LOG_INF("RX, timer reset");
				}
			}
		}
		break;
	}
}

int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr)
	{
		// LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0)
	{
		// LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do
	{
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res)
		{
			// LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

	// LOG_DBG("HF clock started");
	return 0;
}

int esb_initialize(void)
{
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	// config.protocol = ESB_PROTOCOL_ESB_DPL;
	// config.mode = ESB_MODE_PTX;
	config.event_handler = event_handler;
	// config.bitrate = ESB_BITRATE_2MBPS;
	// config.crc = ESB_CRC_16BIT;
	config.tx_output_power = 4;
	// config.retransmit_delay = 600;
	// config.retransmit_count = 3;
	config.tx_mode = ESB_TXMODE_MANUAL;
	// config.payload_length = 32;
	config.selective_auto_ack = true;

	// Fast startup mode
	NRF_RADIO->MODECNF0 |= RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;
	// nrf_radio_modecnf0_set(NRF_RADIO, true, 0);

	err = esb_init(&config);

	if (err)
	{
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err)
	{
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err)
	{
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err)
	{
		return err;
	}

	return 0;
}

int esb_initialize_rx(void)
{
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	// config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.mode = ESB_MODE_PRX;
	config.event_handler = event_handler;
	// config.bitrate = ESB_BITRATE_2MBPS;
	// config.crc = ESB_CRC_16BIT;
	config.tx_output_power = 4;
	// config.retransmit_delay = 600;
	// config.retransmit_count = 3;
	// config.tx_mode = ESB_TXMODE_AUTO;
	// config.payload_length = 32;
	config.selective_auto_ack = true;

	// Fast startup mode
	NRF_RADIO->MODECNF0 |= RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;
	// nrf_radio_modecnf0_set(NRF_RADIO, true, 0);

	err = esb_init(&config);

	if (err)
	{
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err)
	{
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err)
	{
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err)
	{
		return err;
	}

	return 0;
}

int powerstate = 0;
int last_powerstate = 0;

void set_LN(void) {
	tickrate = 6;
	// TODO: This becomes part of the sensor
//	aMode = aMode_LN;
#if (MAG_ENABLED == true)
//	gMode = gMode_LN;
//	MBW = MBW_400Hz;
	MODR = MODR_200Hz;
#endif
}

void set_LP(void) {
	tickrate = 33;
	// TODO: This becomes part of the sensor
//	aMode = aMode_LP;
#if (MAG_ENABLED == true)
//	gMode = gMode_SBY;
//	MBW = MBW_800Hz;
	MODR = MODR_ONESHOT;
#endif
}

void reconfigure_imu(const struct i2c_dt_spec imu) {
    i2c_reg_write_byte_dt(&imu, ICM42688_ACCEL_CONFIG0, Ascale << 5 | AODR); // set accel ODR and FS
    i2c_reg_write_byte_dt(&imu, ICM42688_GYRO_CONFIG0, Gscale << 5 | GODR); // set gyro ODR and FS
    i2c_reg_write_byte_dt(&imu, ICM42688_PWR_MGMT0, gMode << 2 | aMode); // set accel and gyro modes
}

void reconfigure_mag(const struct i2c_dt_spec mag) {
    i2c_reg_write_byte_dt(&mag, MMC5983MA_CONTROL_1, MBW); // set mag bandwidth
    i2c_reg_write_byte_dt(&mag, MMC5983MA_CONTROL_2, 0x80 | (MSET << 4) | 0x08 | MODR); // set mag ODR
}

void configure_system_off_WOM(const struct i2c_dt_spec imu)
{
	icm_setup_WOM(imu); // enable WOM feature
	// Configure WOM interrupt
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios), NRF_GPIO_PIN_SENSE_LOW);
	// Store the last quats
	for (uint8_t i = 0; i < 4; i++){
		retained.q[i] = q[i];
		retained.q2[i] = q2[i];
	}
	retained.stored_quats = true;
	retained_update();
	// Set system off
	pm_state_force(0u, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});
	k_sleep(K_SECONDS(1));
}

void configure_system_off_chgstat(void){
//	// Configure chgstat interrupt
//	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL);
//	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chgstat_gpios), NRF_GPIO_PIN_PULLUP);
//	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chgstat_gpios), NRF_GPIO_PIN_SENSE_LOW);
	// Configure dock interrupt
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_PULLUP); // Still works
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_LOW);
	// Set system off
	pm_state_force(0u, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});
	k_sleep(K_SECONDS(1));
}

void configure_system_off_dock(void){
	// Configure dock interrupt
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL); // Still works
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_HIGH);
	// Set system off
	pm_state_force(0u, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});
	k_sleep(K_SECONDS(1));
}

//static inline float fabs(float __n)
//{
//	return (__n < 0) ? -__n : __n;
//}

bool quat_epsilon(float *q, float *q2) {
	return fabs(q[0] - q2[0]) < 0.0001f && fabs(q[1] - q2[1]) < 0.0001f && fabs(q[2] - q2[2]) < 0.0001f && fabs(q[3] - q2[3]) < 0.0001f;
}

bool quat_epsilon_coarse(float *q, float *q2) {
	return fabs(q[0] - q2[0]) < 0.0005f && fabs(q[1] - q2[1]) < 0.0005f && fabs(q[2] - q2[2]) < 0.0005f && fabs(q[3] - q2[3]) < 0.0005f;
}

bool vec_epsilon(float *a, float *a2) {
	return fabs(a[0] - a2[0]) < 0.1f && fabs(a[1] - a2[1]) < 0.1f && fabs(a[2] - a2[2]) < 0.1f;
}

void apply_BAinv(float xyz[3], float BAinv[4][3]) {
	float temp[3];
	for (int i = 0; i < 3; i++)
	    temp[i] = xyz[i] - BAinv[0][i];
	xyz[0] = BAinv[1][0] * temp[0] + BAinv[1][1] * temp[1] + BAinv[1][2] * temp[2];
	xyz[1] = BAinv[2][0] * temp[0] + BAinv[2][1] * temp[1] + BAinv[2][2] * temp[2];
	xyz[2] = BAinv[3][0] * temp[0] + BAinv[3][1] * temp[1] + BAinv[3][2] * temp[2];
}

bool wait_for_motion(const struct i2c_dt_spec mag, bool motion, int samples) {
	uint8_t counts = 0;
	float a[3], last_a[3];
	icm_accel_read(main_imu, last_a);
	for (int i = 0; i < samples + counts; i++) {
gpio_pin_toggle_dt(&led); // scuffed led
		LOG_INF("Accel: %.5f %.5f %.5f", a[0], a[1], a[2]);
		k_msleep(500);
		icm_accel_read(main_imu, a);
		if (vec_epsilon(a, last_a) != motion) {
			LOG_INF("Pass");
			counts++;
			if (counts == 2) {
				return true;
			}
		} else {
			counts = 0;
		}
		memcpy(last_a, a, sizeof(a));
	}
	LOG_INF("Fail");
	return false;
}

// TODO: make threads more abstract, pass in imus n stuff instead
void main_imu_thread(void) {
	k_sleep(K_FOREVER);
	main_running = true;
	while (1) {
		if (main_ok)
		{
			// TODO: on any errors set main_ok false and skip (make functions return nonzero)
			// Read main FIFO
			uint8_t rawCount[2];
			i2c_burst_read_dt(&main_imu, ICM42688_FIFO_COUNTH, &rawCount[0], 2);
			uint16_t count = (uint16_t)(rawCount[0] << 8 | rawCount[1]); // Turn the 16 bits into a unsigned 16-bit value
//						LOG_INF("packs %u", count);
			count += 32; // Add a few read buffer packets (4 ms)
			uint16_t packets = count / 8;								 // Packet size 8 bytes
			uint8_t rawData[2080];
			uint16_t stco = 0;
			uint8_t addr = ICM42688_FIFO_DATA;
			i2c_write_dt(&main_imu, &addr, 1); // Start read buffer
			while (count > 0) {
				i2c_read_dt(&main_imu, &rawData[stco], count > 248 ? 248 : count); // Read less than 255 at a time (for nRF52832)
				stco += 248;
				count = count > 248 ? count - 248 : 0;
//						LOG_INF("left %u", count);
			}

    		float a[3];
			icm_accel_read(main_imu, a);
			float ax = a[0] - accelBias[0];
			float ay = a[1] - accelBias[1];
			float az = a[2] - accelBias[2];

#if (MAG_ENABLED == true)
			float mx, my, mz;
			if (last_powerstate == 0) {
				float m[3];
				mmc_mag_read(main_mag, m);
				apply_BAinv(m, magBAinv);
				mx = m[0];
				my = m[1];
				mz = m[2];
			}
#endif

#if (MAG_ENABLED == true)
			if (reconfig) {
				switch (powerstate) {
					case 0:
						set_LN();
						LOG_INF("Switch main imus to low noise");
						break;
					case 1:
						set_LP();
						LOG_INF("Switch main imus to low power");
						break;
				};
				//reconfigure_imu(main_imu); // Reconfigure if needed
				reconfigure_mag(main_mag); // Reconfigure if needed
			}
#endif

        	FusionVector z = {.array = {0, 0, 0}};
			if (packets == 2 && powerstate == 1 && MAG_ENABLED == true) {
					ahrs.initialising = true;
					ahrs.rampedGain = 10.0f;
					ahrs.accelerometerIgnored = false;
					ahrs.accelerationRejectionTimer = 0;
					ahrs.accelerationRejectionTimeout = false;
        			FusionVector a = {.array = {ax, -az, ay}};
        			FusionAhrsUpdate(&ahrs, z, a, z, INTEGRATION_TIME_LP);
					FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);
					memcpy(q, quat.array, sizeof(q));
			} else {
				for (uint16_t i = 0; i < packets; i++)
				{
					uint16_t index = i * 8; // Packet size 8 bytes
					if ((rawData[index] & 0x80) == 0x80) {
						continue; // Skip empty packets
					}
					// combine into 16 bit values
					float raw0 = (int16_t)((((int16_t)rawData[index + 1]) << 8) | rawData[index + 2]); // gx
					float raw1 = (int16_t)((((int16_t)rawData[index + 3]) << 8) | rawData[index + 4]); // gy
					float raw2 = (int16_t)((((int16_t)rawData[index + 5]) << 8) | rawData[index + 6]); // gz
					if (raw0 < -32766 || raw1 < -32766 || raw2 < -32766) {
						continue; // Skip invalid data
					}
					// transform and convert to float values
					float gx = raw0 * (2000.0f/32768.0f) - gyroBias[0]; //gres
					float gy = raw1 * (2000.0f/32768.0f) - gyroBias[1]; //gres
					float gz = raw2 * (2000.0f/32768.0f) - gyroBias[2]; //gres
        			FusionVector g = {.array = {gx, -gz, gy}};
        			FusionVector a = {.array = {ax, -az, ay}};
        			g = FusionOffsetUpdate(&offset, g);
#if MAG_ENABLED
        			FusionVector m = {.array = {my, mz, -mx}};
        			FusionAhrsUpdate(&ahrs, g, a, m, INTEGRATION_TIME);
#else
        			FusionAhrsUpdate(&ahrs, g, a, z, INTEGRATION_TIME);
#endif
				}
        		const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);
				lin_ax = earth.array[0];
				lin_ay = earth.array[1];
				lin_az = earth.array[2];
				FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);
				memcpy(q, quat.array, sizeof(q));
			}

			if (quat_epsilon_coarse(q, last_q)) { // Probably okay to use the constantly updating last_q
				if (k_uptime_get() - last_data_time > 60 * 1000) { // No motion in last 1m
					LOG_INF("No motion from main imus in 1m");
					system_off_main = true;
				} else if (powerstate == 0 && k_uptime_get() - last_data_time > 500) { // No motion in last 500ms
					LOG_INF("No motion from main imus in 500ms");
					powerstate = 1;
				}
			} else {
				last_data_time = k_uptime_get();
				powerstate = 0;
			}

			if (!(quat_epsilon(q, last_q))) {
				for (uint8_t i = 0; i < 4; i++) {
					last_q[i] = q[i];
				}
				for (uint16_t i = 0; i < 16; i++) {
					tx_payload.data[i] = 0;
				}
				float q_offset[4];
				q_multiply(q, q3, q_offset);
				tx_buf[0] = INT16_TO_UINT16(TO_FIXED_14(q_offset[0]));
				tx_buf[1] = INT16_TO_UINT16(TO_FIXED_14(q_offset[1]));
				tx_buf[2] = INT16_TO_UINT16(TO_FIXED_14(q_offset[2]));
				tx_buf[3] = INT16_TO_UINT16(TO_FIXED_14(q_offset[3]));
				tx_buf[4] = INT16_TO_UINT16(TO_FIXED_10(lin_ax));
				tx_buf[5] = INT16_TO_UINT16(TO_FIXED_10(lin_ay));
				tx_buf[6] = INT16_TO_UINT16(TO_FIXED_10(lin_az));
				tx_payload.data[0] = 0; //reserved for something idk
				tx_payload.data[1] = tracker_id << 4;
				//tx_payload.data[2] = batt | (charging ? 128 : 0);
				// TODO: Send temperature
				tx_payload.data[2] = batt;
				tx_payload.data[3] = batt_v;
				tx_payload.data[4] = (tx_buf[0] >> 8) & 255;
				tx_payload.data[5] = tx_buf[0] & 255;
				tx_payload.data[6] = (tx_buf[1] >> 8) & 255;
				tx_payload.data[7] = tx_buf[1] & 255;
				tx_payload.data[8] = (tx_buf[2] >> 8) & 255;
				tx_payload.data[9] = tx_buf[2] & 255;
				tx_payload.data[10] = (tx_buf[3] >> 8) & 255;
				tx_payload.data[11] = tx_buf[3] & 255;
				tx_payload.data[12] = (tx_buf[4] >> 8) & 255;
				tx_payload.data[13] = tx_buf[4] & 255;
				tx_payload.data[14] = (tx_buf[5] >> 8) & 255;
				tx_payload.data[15] = tx_buf[5] & 255;
				tx_payload.data[16] = (tx_buf[6] >> 8) & 255;
				tx_payload.data[17] = tx_buf[6] & 255;
				esb_flush_tx();
				main_data = true;
				esb_write_payload(&tx_payload); // Add transmission to queue
			}
		} else {
// 5ms delta (???) from entering loop
			int64_t time_delta = k_uptime_get() - start_time;
			if (time_delta < 11)
			{
				k_msleep(11 - time_delta);
			}
			//k_msleep(11);														 // Wait for start up (1ms for ICM, 10ms for MMC -> 10ms)
			uint8_t ICM42688ID = icm_getChipID(main_imu);						 // Read CHIP_ID register for ICM42688
				LOG_INF("ICM: %u", ICM42688ID);
			uint8_t MMC5983ID = mmc_getChipID(main_mag);						 // Read CHIP_ID register for MMC5983MA
				LOG_INF("MMC: %u", MMC5983ID);
			if ((ICM42688ID == 0x47 || ICM42688ID == 0xDB) && MMC5983ID == 0x30) // check if all I2C sensors have acknowledged
			{
				LOG_INF("Found main imus");
    			i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Reset MMC now to avoid waiting 10ms later
				icm_reset(main_imu);												 // software reset ICM42688 to default registers
				uint8_t temp;
				i2c_reg_read_byte_dt(&main_imu, ICM42688_INT_STATUS, &temp); // clear reset done int flag
				icm_init(main_imu, Ascale, Gscale, AODR, GODR, aMode, gMode, false); // configure
// 55-66ms delta to wait, get chip ids, and setup icm (50ms spent waiting for accel and gyro to start)
#if (MAG_ENABLED == true)
				mmc_SET(main_mag);													 // "deGauss" magnetometer
				mmc_init(main_mag, MODR, MBW, MSET);								 // configure
// 0-1ms delta to setup mmc
#endif
				main_ok = true;
				do {
				if (reset_mode == 1) { // Reset mode main calibration
					LOG_INF("Enter main calibration");
gpio_pin_set_dt(&led, 0); // scuffed led
					// TODO: Add LED flashies
					LOG_INF("Rest the device on a stable surface");
					if (!wait_for_motion(main_imu, false, 20)) { // Wait for accelerometer to settle, timeout 10s
gpio_pin_set_dt(&led, 0); // scuffed led
						break; // Timeout, calibration failed
					}
gpio_pin_set_dt(&led, 1); // scuffed led
					k_msleep(500); // Delay before beginning acquisition
					LOG_INF("Start accel and gyro calibration");
					icm_offsetBias(main_imu, accelBias, gyroBias); // This takes about 3s
					nvs_write(&fs, MAIN_ACCEL_BIAS_ID, &accelBias, sizeof(accelBias));
					nvs_write(&fs, MAIN_GYRO_BIAS_ID, &gyroBias, sizeof(gyroBias));
					LOG_INF("%.5f %.5f %.5f", accelBias[0], accelBias[1], accelBias[2]);
					LOG_INF("%.5f %.5f %.5f", gyroBias[0], gyroBias[1], gyroBias[2]);
					LOG_INF("Finished accel and gyro zero offset calibration");
gpio_pin_set_dt(&led, 0); // scuffed led
#if (MAG_ENABLED == true)
					LOG_INF("Gently rotate device in all directions");
					if (!wait_for_motion(main_imu, true, 20)) { // Wait for accelerometer motion, timeout 10s
gpio_pin_set_dt(&led, 0); // scuffed led
						break; // Timeout, calibration failed
					}
gpio_pin_set_dt(&led, 1); // scuffed led
					k_msleep(500); // Delay before beginning acquisition
					LOG_INF("Start mag calibration");
					double ata[100] = {0.0};
					double norm_sum = 0.0;
					double sample_count = 0.0;
					float m[3];
					for (int i = 0; i < 200; i++) { // 200 samples in 20s, 100ms per sample
gpio_pin_toggle_dt(&led); // scuffed led
						mmc_mag_read(main_mag, m);
						LOG_INF("Mag: %.5f %.5f %.5f (%d/200)", m[0], m[1], m[2], i+1);
						magneto_sample(m[0], m[1], m[2], ata, &norm_sum, &sample_count);
						k_msleep(100);
					}
					magneto_current_calibration(magBAinv, ata, norm_sum, sample_count);
					nvs_write(&fs, MAIN_MAG_BIAS_ID, &magBAinv, sizeof(magBAinv));
					for (int i = 0; i < 3; i++) {
						LOG_INF("%.5f %.5f %.5f %.5f", magBAinv[0][i], magBAinv[1][i], magBAinv[2][i], magBAinv[3][i]);
					}
					LOG_INF("Finished mag hard/soft iron offset calibration");
gpio_pin_set_dt(&led, 0); // scuffed led
#endif
					reset_mode = 0; // Clear reset mode
				}
				} while (false);
				// Setup fusion
			    FusionOffsetInitialise(&offset, 1/INTEGRATION_TIME);
			    FusionAhrsInitialise(&ahrs);
			    const FusionAhrsSettings settings = {
			            .convention = FusionConventionNwu,
			            .gain = 0.5f,
			            .accelerationRejection = 10.0f,
			            .magneticRejection = 20.0f,
			            .rejectionTimeout = 5 * 1/INTEGRATION_TIME, /* 5 seconds */
			    };
			    FusionAhrsSetSettings(&ahrs, &settings);
				memcpy(ahrs.quaternion.array, q, sizeof(q)); // Load existing quat
			}
		}
		main_running = false;
		k_sleep(K_FOREVER);
		main_running = true;
	}
}

K_THREAD_DEFINE(main_imu_thread_id, 4096, main_imu_thread, NULL, NULL, NULL, 7, 0, 0);

void wait_for_main_imu_thread(void) {
	while (main_running) {
		k_usleep(1);
	}
}
#ifdef bruh
void aux_imu_thread(void) {
	k_sleep(K_FOREVER);
	aux_running = true;
	while (1) {
		if (aux_ok)
		{
			// TODO: on any errors set aux_ok false and skip (make functions return nonzero)
			// Read main FIFO
			uint8_t rawCount[2];
			i2c_burst_read_dt(&aux_imu, ICM42688_FIFO_COUNTH, &rawCount[0], 2);
			uint16_t count = (uint16_t)(rawCount[0] << 8 | rawCount[1]); // Turn the 16 bits into a unsigned 16-bit value
			count += 16; // Add two read buffer packets
			uint16_t packets = count / 8;								 // Packet size 8 bytes
			uint8_t rawData[2080];
			i2c_burst_read_dt(&aux_imu, ICM42688_FIFO_DATA, &rawData[0], count); // Read buffer
    		uint8_t rawAccel[6];
    		i2c_burst_read_dt(&aux_imu, ICM42688_ACCEL_DATA_X1, &rawAccel[0], 6); // Read accel only
			float raw0 = (int16_t)((((int16_t)rawAccel[0]) << 8) | rawAccel[1]);
			float raw1 = (int16_t)((((int16_t)rawAccel[2]) << 8) | rawAccel[3]);
			float raw2 = (int16_t)((((int16_t)rawAccel[4]) << 8) | rawAccel[5]);
			// transform and convert to float values
			float ax = raw0 * aRes - accelBias2[0];
			float ay = raw1 * aRes - accelBias2[1];
			float az = raw2 * aRes - accelBias2[2];
#if (MAG_ENABLED == true)
			if (last_powerstate == 0) {
				mmc_readData(aux_mag, MMC5983MAData2);
				// transform and convert to float values
				// TODO: make sure these are reading to separate buffers for main vs. aux
				mx2 = ((float)MMC5983MAData2[0] - MMC5983MA_offset) * mRes - magBias2[0];
				my2 = ((float)MMC5983MAData2[1] - MMC5983MA_offset) * mRes - magBias2[1];
				mz2 = ((float)MMC5983MAData2[2] - MMC5983MA_offset) * mRes - magBias2[2];
				mx2 *= magScale2[0];
				my2 *= magScale2[1];
				mz2 *= magScale2[2];
			}
#endif

			if (reconfig) {
				switch (powerstate) {
					case 0:
						set_LN();
						LOG_INF("Switch aux imus to low noise");
						break;
					case 1:
						set_LP();
						LOG_INF("Switch aux imus to low power");
						break;
				};
				reconfigure_imu(aux_imu); // Reconfigure if needed
#if (MAG_ENABLED == true)
				reconfigure_mag(aux_mag); // Reconfigure if needed
#endif
			}

			if (packets == 2 && powerstate == 1) {
				MadgwickQuaternionUpdate(ax, -az, ay, 0, 0, 0, my2, mz2, -mx2, INTEGRATION_TIME_LP, q2, MAG_ENABLED);
			} else {
				for (uint16_t i = 0; i < packets; i++)
				{
					uint16_t index = i * 8; // Packet size 8 bytes
					if ((rawData[index] & 0x80) == 0x80) {
						continue; // Skip empty packets
					}
					// combine into 16 bit values
					float raw0 = (int16_t)((((int16_t)rawData[index + 1]) << 8) | rawData[index + 2]); // gx
					float raw1 = (int16_t)((((int16_t)rawData[index + 3]) << 8) | rawData[index + 4]); // gy
					float raw2 = (int16_t)((((int16_t)rawData[index + 5]) << 8) | rawData[index + 6]); // gz
					if (raw0 < -32766 || raw1 < -32766 || raw2 < -32766) {
						continue; // Skip invalid data
					}
					// transform and convert to float values
					float gx = raw0 * gRes - gyroBias2[0];
					float gy = raw1 * gRes - gyroBias2[1];
					float gz = raw2 * gRes - gyroBias2[2];
					// TODO: swap out fusion?
					MadgwickQuaternionUpdate(ax, -az, ay, gx * pi / 180.0f, -gz * pi / 180.0f, gy * pi / 180.0f, my2, mz2, -mx2, INTEGRATION_TIME, q2, MAG_ENABLED);
					if (i == packets - 1) {
						// Calculate linear acceleration (no gravity)
						lin_ax2 = ax + 2.0f * (q2[0] * q2[1] + q2[2] * q2[3]);
						lin_ay2 = ay + 2.0f * (q2[1] * q2[3] - q2[0] * q2[2]);
						lin_az2 = az - q2[0] * q2[0] - q2[1] * q2[1] - q2[2] * q2[2] + q2[3] * q2[3];
					}
				}
			}

			if (quat_epsilon_coarse(q2, last_q2)) { // Probably okay to use the constantly updating last_q
				if (k_uptime_get() - last_data_time > 60 * 1000) { // No motion in last 1m
					LOG_INF("No motion from aux imus in 1m");
					system_off_main = true;
				} else if (powerstate == 0 && k_uptime_get() - last_data_time > 500) { // No motion in last 500ms
					LOG_INF("No motion from aux imus in 500ms");
					powerstate = 1;
				}
			} else {
				last_data_time = k_uptime_get();
				powerstate = 0;
			}

			if (!(quat_epsilon(q2, last_q2))) {
				for (uint8_t i = 0; i < 4; i++) {
					last_q2[i] = q2[i];
				}
				for (uint16_t i = 0; i < 16; i++) {
					tx_payload.data[i] = 0;
				}
				wait_for_main_imu_thread();
				float q_offset[4];
				q_multiply(q2, q3, q_offset);
				tx_buf[0] = INT16_TO_UINT16(TO_FIXED_14(q_offset[0]));
				tx_buf[1] = INT16_TO_UINT16(TO_FIXED_14(q_offset[1]));
				tx_buf[2] = INT16_TO_UINT16(TO_FIXED_14(q_offset[2]));
				tx_buf[3] = INT16_TO_UINT16(TO_FIXED_14(q_offset[3]));
				tx_buf[4] = INT16_TO_UINT16(TO_FIXED_10(lin_ax2));
				tx_buf[5] = INT16_TO_UINT16(TO_FIXED_10(lin_ay2));
				tx_buf[6] = INT16_TO_UINT16(TO_FIXED_10(lin_az2));
				tx_payload.data[0] = (tracker_id << 4) | 1;
				tx_payload.data[1] = (uint8_t)(batt_pptt / 100) | (charging ? 128 : 0);
				//tx_payload.data[1] = (uint8_t)(batt_pptt / 100);
				tx_payload.data[2] = (tx_buf[0] >> 8) & 255;
				tx_payload.data[3] = tx_buf[0] & 255;
				tx_payload.data[4] = (tx_buf[1] >> 8) & 255;
				tx_payload.data[5] = tx_buf[1] & 255;
				tx_payload.data[6] = (tx_buf[2] >> 8) & 255;
				tx_payload.data[7] = tx_buf[2] & 255;
				tx_payload.data[8] = (tx_buf[3] >> 8) & 255;
				tx_payload.data[9] = tx_buf[3] & 255;
				tx_payload.data[10] = (tx_buf[4] >> 8) & 255;
				tx_payload.data[11] = tx_buf[4] & 255;
				tx_payload.data[12] = (tx_buf[5] >> 8) & 255;
				tx_payload.data[13] = tx_buf[5] & 255;
				tx_payload.data[14] = (tx_buf[6] >> 8) & 255;
				tx_payload.data[15] = tx_buf[6] & 255;
				if (!main_data) esb_flush_tx();
				esb_write_payload(&tx_payload); // Add transmission to queue
			}
		} else if (aux_exists) {
			uint8_t ICM42688ID = icm_getChipID(aux_imu);						 // Read CHIP_ID register for ICM42688
			uint8_t MMC5983ID = mmc_getChipID(aux_mag);							 // Read CHIP_ID register for MMC5983MA
			if ((ICM42688ID == 0x47 || ICM42688ID == 0xDB) && MMC5983ID == 0x30) // check if all I2C sensors have acknowledged
			{
				LOG_INF("Found aux imus");
    			i2c_reg_write_byte_dt(&aux_mag, MMC5983MA_CONTROL_1, 0x80); // Reset MMC now to avoid waiting 10ms later
				icm_reset(aux_imu);												 // software reset ICM42688 to default registers
				icm_DRStatus(aux_imu);												 // clear reset done int flag
				icm_init(aux_imu, Ascale, Gscale, AODR, GODR, aMode, gMode, false); // configure
// 55-66ms delta to wait, get chip ids, and setup icm (50ms spent waiting for accel and gyro to start)
#if (MAG_ENABLED == true)
				mmc_SET(aux_mag);													 // "deGauss" magnetometer
				mmc_init(aux_mag, MODR, MBW, MSET);								 // configure
#endif
// 0-1ms delta to setup mmc
				aux_ok = true;
				if (reset_mode == 2) { // Reset mode aux calibration
					LOG_INF("Enter aux calibration");
gpio_pin_set_dt(&led, 0); // scuffed led
					// TODO: Add LED flashies
					// TODO: Wait for accelerometer to settle, then calibrate (5 sec timeout to skip)
					icm_offsetBias(aux_imu, accelBias2, gyroBias2); // This takes about 750ms
					nvs_write(&fs, AUX_ACCEL_BIAS_ID, &accelBias2, sizeof(accelBias));
					nvs_write(&fs, AUX_GYRO_BIAS_ID, &gyroBias2, sizeof(gyroBias));
					LOG_INF("Finished accel and gyro zero offset calibration");
gpio_pin_set_dt(&led, 1); // scuffed led
					// TODO: Wait for accelerometer movement, then calibrate (5 sec timeout to skip)
#if (MAG_ENABLED == true)
					mmc_offsetBias(aux_mag, magBias2, magScale2); // This takes about 20s
					nvs_write(&fs, AUX_MAG_BIAS_ID, &magBias2, sizeof(magBias));
					nvs_write(&fs, AUX_MAG_SCALE_ID, &magScale2, sizeof(magScale));
					LOG_INF("Finished mag hard/soft iron offset calibration");
#endif
					reset_mode = 0; // Clear reset mode
				}
			} else {
				aux_exists = false;
			}
		}
		aux_running = false;
		k_sleep(K_FOREVER);
		aux_running = true;
	}
}
#else
void aux_imu_thread(void) {
	k_sleep(K_FOREVER);
}
#endif
K_THREAD_DEFINE(aux_imu_thread_id, 4096, aux_imu_thread, NULL, NULL, NULL, 7, 0, 0);

void wait_for_threads(void) {
	if (threads_running) {
		while (main_running) {
			k_usleep(1);
		}
		while (aux_running) {
			k_usleep(1);
		}
	}
}

void power_check(void) {
	bool docked = gpio_pin_get_dt(&dock);
	batt_pptt = read_batt();
	if (batt_pptt == 0 && !docked) {
		gpio_pin_set_dt(&led, 0); // Turn off LED
		configure_system_off_chgstat();
	} else if (docked) {
		gpio_pin_set_dt(&led, 0); // Turn off LED
		configure_system_off_dock();
	}
}

void main(void)
{
	int32_t reset_reason = NRF_POWER->RESETREAS;
	NRF_POWER->RESETREAS = NRF_POWER->RESETREAS; // Clear RESETREAS
	uint8_t reboot_counter = 0;

	gpio_pin_configure_dt(&dock, GPIO_INPUT);
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);

	power_check(); // check the battery and dock first before continuing

	start_time = k_uptime_get(); // Need to get start time for imu startup delta
	gpio_pin_set_dt(&led, 1); // Boot LED

	struct flash_pages_info info;
	fs.flash_device = NVS_PARTITION_DEVICE;
	fs.offset = NVS_PARTITION_OFFSET; // Start NVS FS here
	flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	fs.sector_size = info.size; // Sector size equal to page size
	fs.sector_count = 4U; // 4 sectors
	nvs_mount(&fs);
// 5-6ms delta to initialize NVS
	if (reset_reason & 0x01) { // Count pin resets
		nvs_read(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
		reset_mode = reboot_counter;
		reboot_counter++;
		nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
		LOG_INF("Reset Count: %u", reboot_counter);
		k_msleep(1000); // Wait before clearing counter and continuing
		reboot_counter = 0;
		nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
	}
// 0ms or 1000ms delta for reboot counter

	gpio_pin_set_dt(&led, 0);

// TODO: if reset counter is 0 but reset reason was 1 then perform imu scanning (pressed reset once)

	if (reset_mode == 4) { // Reset mode DFU
		// TODO: DFU
		LOG_INF("Enter DFU");
		reset_mode = 0; // Clear reset mode
	}

	if (reset_mode == 3) { // Reset mode pairing reset
		LOG_INF("Enter pairing reset");
		nvs_write(&fs, PAIRED_ID, &paired_addr, sizeof(paired_addr)); // Clear paired address
		reset_mode = 0; // Clear reset mode
	} else {
		// Read paired address from NVS
		nvs_read(&fs, PAIRED_ID, &paired_addr, sizeof(paired_addr));
	}

	clocks_start();

	if (paired_addr[0] == 0x00) { // No dongle paired
		for (int i = 0; i < 4; i++) {
			base_addr_0[i] = discovery_base_addr_0[i];
			base_addr_1[i] = discovery_base_addr_1[i];
		}
		for (int i = 0; i < 8; i++) {
			addr_prefix[i] = discovery_addr_prefix[i];
		}
		esb_initialize();
	timer_init();
		tx_payload_pair.noack = false;
		uint64_t addr = (((uint64_t)(NRF_FICR->DEVICEADDR[1]) << 32) | NRF_FICR->DEVICEADDR[0]) & 0xFFFFFF;
		uint8_t check = addr & 255;
		if (check == 0) check = 8;
		LOG_INF("Check Code: %02X", paired_addr[0]);
		tx_payload_pair.data[0] = check; // Use int from device address to make sure packet is for this device
		for (int i = 0; i < 6; i++) {
			tx_payload_pair.data[i+2] = (addr >> (8 * i)) & 0xFF;
		}
		while (paired_addr[0] != check) {
			if (paired_addr[0] != 0x00) {
				LOG_INF("Incorrect check code: %02X", paired_addr[0]);
				paired_addr[0] = 0x00; // Packet not for this device
			}
			esb_flush_rx();
			esb_flush_tx();
			esb_write_payload(&tx_payload_pair); // Still fails after a while
			gpio_pin_set_dt(&led, 1);
			k_msleep(100);
			gpio_pin_set_dt(&led, 0);
			k_msleep(900);
			power_check();
		}
		LOG_INF("Paired");
		nvs_write(&fs, PAIRED_ID, &paired_addr, sizeof(paired_addr)); // Write new address and tracker id
		esb_disable();
	}
	LOG_INF("Read pairing data");
	LOG_INF("Check Code: %02X", paired_addr[0]);
	LOG_INF("Tracker ID: %u", paired_addr[1]);
	LOG_INF("Address: %02X %02X %02X %02X %02X %02X", paired_addr[2], paired_addr[3], paired_addr[4], paired_addr[5], paired_addr[6], paired_addr[7]);

	tracker_id = paired_addr[1];

	// Recreate dongle address
	uint8_t buf2[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	for (int i = 0; i < 4; i++) {
		buf2[i] = paired_addr[i+2];
		buf2[i+4] = paired_addr[i+2] + paired_addr[6];
	}
	for (int i = 0; i < 8; i++) {
		buf2[i+8] = paired_addr[7] + i;
	}
	for (int i = 0; i < 16; i++) {
		if (buf2[i] == 0x00 || buf2[i] == 0x55 || buf2[i] == 0xAA) {
			buf2[i] += 8;
		};
	}
	for (int i = 0; i < 4; i++) {
		base_addr_0[i] = buf2[i];
		base_addr_1[i] = buf2[i+4];
	}
	for (int i = 0; i < 8; i++) {
		addr_prefix[i] = buf2[i+8];
	}

	// Read calibration from NVS
	nvs_read(&fs, MAIN_ACCEL_BIAS_ID, &accelBias, sizeof(accelBias));
	nvs_read(&fs, MAIN_GYRO_BIAS_ID, &gyroBias, sizeof(gyroBias));
	nvs_read(&fs, MAIN_MAG_BIAS_ID, &magBAinv, sizeof(magBAinv));
	nvs_read(&fs, AUX_ACCEL_BIAS_ID, &accelBias2, sizeof(accelBias));
	nvs_read(&fs, AUX_GYRO_BIAS_ID, &gyroBias2, sizeof(gyroBias));
	nvs_read(&fs, AUX_MAG_BIAS_ID, &magBAinv2, sizeof(magBAinv));
	LOG_INF("Read calibrations");
	LOG_INF("Main accel bias: %.5f %.5f %.5f", accelBias[0], accelBias[1], accelBias[2]);
	LOG_INF("Main gyro bias: %.5f %.5f %.5f", gyroBias[0], gyroBias[1], gyroBias[2]);
	LOG_INF("Main mag matrix:");
	for (int i = 0; i < 3; i++) {
		LOG_INF("%.5f %.5f %.5f %.5f", magBAinv[0][i], magBAinv[1][i], magBAinv[2][i], magBAinv[3][i]);
	}
	LOG_INF("Aux accel bias: %.5f %.5f %.5f", accelBias2[0], accelBias2[1], accelBias2[2]);
	LOG_INF("Aux gyro bias: %.5f %.5f %.5f", gyroBias2[0], gyroBias2[1], gyroBias2[2]);
	LOG_INF("Aux mag matrix:");
	for (int i = 0; i < 3; i++) {
		LOG_INF("%.5f %.5f %.5f %.5f", magBAinv2[0][i], magBAinv2[1][i], magBAinv2[2][i], magBAinv2[3][i]);
	}

	//gpio_pin_configure_dt(&chgstat, GPIO_INPUT);
	//gpio_pin_configure_dt(&int0, GPIO_INPUT);
	//gpio_pin_configure_dt(&int1, GPIO_INPUT);

	//pwm_set_pulse_dt(&led0, PWM_MSEC(20)); // 10/20 = 50%
	//gpio_pin_set_dt(&led, 1);

	// Recover quats if present
	retained_validate();
	if (retained.stored_quats) {
		for (uint8_t i = 0; i < 4; i++){
			q[i] = retained.q[i];
			q2[i] = retained.q2[i];
		}
		LOG_INF("Recovered quaternions\nMain: %.2f %.2f %.2f %.2f\nAux: %.2f %.2f %.2f %.2f", q[0], q[1], q[2], q[3], q2[0], q2[1], q2[2], q2[3]);
		retained.stored_quats = false; // Invalidate the retained quaternions
		retained_update();
	}
// 0ms delta to read calibration and configure pins (unknown time to read retained data but probably negligible)
	esb_initialize();
	tx_payload.noack = false;
	timer_init();
// 1ms delta to start ESB
	while (1)
	{
		// Get start time
		int64_t time_begin = k_uptime_get();

		//charging = gpio_pin_get_dt(&chgstat); // TODO: Charging detect doesn't work (hardware issue)
		bool docked = gpio_pin_get_dt(&dock);

		int batt_mV;
		batt_pptt = read_batt_mV(&batt_mV);
		
		if (batt_pptt == 0 && !docked)
		{
			LOG_INF("Waiting for system off (Low battery)");
			wait_for_threads();
			LOG_INF("Shutdown");
			// Communicate all imus to shut down
    		i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
			i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
			if (aux_ok) {
    			i2c_reg_write_byte_dt(&aux_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for aux ICM to finish reset
				i2c_reg_write_byte_dt(&aux_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for aux MMC to finish reset
			}
			// Turn off LED
			gpio_pin_set_dt(&led, 0);
			configure_system_off_chgstat();
		}
		last_batt_pptt[last_batt_pptt_i] = batt_pptt;
		last_batt_pptt_i++;
		last_batt_pptt_i %= 15;
		for (uint8_t i = 0; i < 15; i++) {  // Average battery readings across 16 samples
			if (last_batt_pptt[i] == 10001) {
				batt_pptt += batt_pptt / (i + 1);
			} else {
				batt_pptt += last_batt_pptt[i];
			}
		}
		batt_pptt /= 16;
		if (batt_pptt + 100 < last_batt_pptt[15]) {last_batt_pptt[15] = batt_pptt + 100;} // Lower bound -100pptt
		else if (batt_pptt > last_batt_pptt[15]) {last_batt_pptt[15] = batt_pptt;} // Upper bound +0pptt
		else {batt_pptt = last_batt_pptt[15];} // Effectively 100-10000 -> 1-100%

		// format for packet send
		batt = batt_pptt / 100;
		if (batt < 1) {batt = 1;} // Clamp to 1%
		batt_mV /= 10;
		batt_mV -= 245;
		if (batt_mV < 0) {batt_v = 0;} // Very dead but it is what it is
		else if (batt_mV > 255) {batt_v = 255;}
		else {batt_v = batt_mV;} // 0-255 -> 2.45-5.00V

		if (time_begin > led_time) {
			if (led_time != 0) {
				led_time_off = time_begin + 300;
			}
			if (batt_pptt < 1000) { // Under 10% battery left
				led_time += 600;
			} else {
				led_time += 10000;
			}
			gpio_pin_set_dt(&led, 1);
		} else if (time_begin > led_time_off) {
			gpio_pin_set_dt(&led, 0);
		}

		if (docked)
		{ // TODO: move to interrupts? (Then you do not need to do the above)
			LOG_INF("Waiting for system off (Docked)");
			wait_for_threads();
			LOG_INF("Shutdown");
			// Communicate all imus to shut down
    		i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
			i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
			if (aux_ok) {
    			i2c_reg_write_byte_dt(&aux_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for aux ICM to finish reset
				i2c_reg_write_byte_dt(&aux_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for aux MMC to finish reset
			}
			// Turn off LED
			gpio_pin_set_dt(&led, 0);
			configure_system_off_dock();
		}

		reconfig = last_powerstate != powerstate ? true : false;
		last_powerstate = powerstate;
		main_data = false;

		wait_for_threads();
		k_wakeup(main_imu_thread_id);
		k_wakeup(aux_imu_thread_id);
		threads_running = true;

		if (system_off_main && (aux_ok ? system_off_aux : true)) { // System off on extended no movement
			LOG_INF("Waiting for system off (No movement)");
			wait_for_threads();
			LOG_INF("Shutdown");
			// Communicate all imus to shut down
			icm_reset(main_imu);
    		i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
			if (aux_ok) {
    			i2c_reg_write_byte_dt(&aux_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for aux ICM to finish reset
				i2c_reg_write_byte_dt(&aux_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for aux MMC to finish reset
			}
			// Turn off LED
			gpio_pin_set_dt(&led, 0);
			configure_system_off_WOM(main_imu);
		}

		// Get time elapsed and sleep/yield until next tick
		int64_t time_delta = k_uptime_get() - time_begin;
		if (time_delta > tickrate)
		{
			k_yield();
		}
		else
		{
			k_msleep(tickrate - time_delta);
		}
	}
}
