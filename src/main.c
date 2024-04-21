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
#include <hal/nrf_gpio.h>

#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/reboot.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>

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
uint32_t batt_pptt;
bool batt_low = false;

const struct i2c_dt_spec main_imu = I2C_DT_SPEC_GET(MAIN_IMU_NODE);
const struct i2c_dt_spec main_mag = I2C_DT_SPEC_GET(MAIN_MAG_NODE);

const struct pwm_dt_spec led0 = PWM_DT_SPEC_GET(LED0_NODE);
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_gpios);

const struct gpio_dt_spec dock = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, dock_gpios);
//const struct gpio_dt_spec chgstat = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, chgstat_gpios);
//const struct gpio_dt_spec int0 = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, int0_gpios);
//const struct gpio_dt_spec int1 = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, int1_gpios);

bool nvs_init = false;

bool threads_running = false;

bool main_running = false;

bool main_ok = false;

bool main_data = false;

#define USER_SHUTDOWN_ENABLED true
#define MAG_ENABLED true

// Saturate int to 16 bits
// Optimized to a single ARM assembler instruction
#define SATURATE_INT16(x) ((x) > 32767 ? 32767 : ((x) < -32768 ? -32768 : (x)))

#define TO_FIXED_15(x) ((int16_t)SATURATE_INT16((x) * (1 << 15)))
#define TO_FIXED_10(x) ((int16_t)((x) * (1 << 10)))
#define TO_FIXED_7(x) ((int16_t)SATURATE_INT16((x) * (1 << 7)))
#define FIXED_14_TO_DOUBLE(x) (((double)(x)) / (1 << 14))
#define FIXED_10_TO_DOUBLE(x) (((double)(x)) / (1 << 10))
#define FIXED_7_TO_DOUBLE(x) (((double)(x)) / (1 << 7))

#define CONST_EARTH_GRAVITY 9.80665

// only scan/detect new imus on reset event, write to nvs

#include "ICM42688.h"
#include "MMC5983MA.h"

float lin_ax, lin_ay, lin_az;					// linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};			// vector to hold quaternion
float last_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};		// vector to hold quaternion

float gOff[3] = {0.0f, 0.0f, 0.0f}; // runtime fusion gyro offset

float q3[4] = {0.5f, -0.5f, -0.5f, -0.5f}; // correction quat

FusionOffset offset; // could share goff and q with fusionoffset and fusionahrs but init clears the values
FusionAhrs ahrs;

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
uint8_t Ascale = AFS_8G, Gscale = GFS_2000DPS, AODR = AODR_200Hz, GODR = GODR_1kHz, aMode = aMode_LN, gMode = gMode_LN; // also change gyro range in fusion!
#define INTEGRATION_TIME 0.001
#define INTEGRATION_TIME_LP 0.005

float accelBias[3] = {0.0f, 0.0f, 0.0f}, gyroBias[3] = {0.0f, 0.0f, 0.0f}; // offset biases for the accel and gyro

// MMC5983MA definitions

// TODO: move to sensor
/* Specify sensor parameters (continuous mode sample rate is dependent on bandwidth)
 * choices are: MODR_ONESHOT, MODR_1Hz, MODR_10Hz, MODR_20Hz, MODR_50 Hz, MODR_100Hz, MODR_200Hz (BW = 0x01), MODR_1000Hz (BW = 0x03)
 * Bandwidth choices are: MBW_100Hz, MBW_200Hz, MBW_400Hz, MBW_800Hz
 * Set/Reset choices are: MSET_1, MSET_25, MSET_75, MSET_100, MSET_250, MSET_500, MSET_1000, MSET_2000, so MSET_100 set/reset occurs every 100th measurement, etc.
 */
uint8_t MODR = MODR_200Hz, MBW = MBW_400Hz, MSET = MSET_2000;

// TODO: move to sensor
float magBAinv[4][3];

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
bool send_data = false;

uint16_t led_clock = 0;
uint32_t led_clock_offset = 0;

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
					led_clock = (rx_payload.data[0] << 8) + rx_payload.data[1]; // sync led flashes :)
					led_clock_offset = 0;
					//LOG_INF("RX, timer reset");
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
	config.retransmit_count = 0;
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

static void timer_handler(nrf_timer_event_t event_type, void *p_context) {
	if (event_type == NRF_TIMER_EVENT_COMPARE1 && esb_state == true) {
		if (last_reset < LAST_RESET_LIMIT) {
			last_reset++;
			if (send_data) { // scuffed check
				esb_write_payload(&tx_payload); // Add transmission to queue
				esb_start_tx();
				send_data = false;
			}
//			esb_flush_tx();
		} else {
			esb_disable();
			esb_initialize_rx();
			esb_start_rx();
			esb_state = false;
			nrfx_timer_pause(&m_timer);
			timer_state = false;
			LOG_INF("timer reset elapsed");
		}
	} else if (event_type == NRF_TIMER_EVENT_COMPARE2 && esb_state == true) {
		esb_disable();
		esb_initialize_rx();
		esb_start_rx();
		esb_state = false;
	} else if (event_type == NRF_TIMER_EVENT_COMPARE3 && esb_state == false) {
		esb_stop_rx();
		esb_disable();
		esb_initialize();
		esb_state = true;
	}
}

void timer_init(void) {
//	nrfx_err_t err;
	nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(1000000);  
	//timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
	//timer_cfg.mode = NRF_TIMER_MODE_TIMER;
	//timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_16;
	//timer_cfg.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY;
	//timer_cfg.p_context = NULL;
	nrfx_timer_init(&m_timer, &timer_cfg, timer_handler);
	uint32_t ticks = nrfx_timer_ms_to_ticks(&m_timer, 3);
	nrfx_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
	LOG_INF("timer at %d", ticks * (paired_addr[1]*2 + 3) / 21); // TODO: temp set max 8
	nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL1, ticks * (paired_addr[1]*2 + 3) / 21, true); // timeslot to send data  TODO: temp set max 8
	nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL2, ticks * 19 / 21, true); // switch to rx
	nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL3, ticks * 2 / 21, true); // switch to tx
	nrfx_timer_enable(&m_timer);
	IRQ_DIRECT_CONNECT(TIMER1_IRQn, 0, nrfx_timer_1_irq_handler, 0);
	irq_enable(TIMER1_IRQn);
	timer_state = true;
}

int powerstate = 0;
int last_powerstate = 0;
int mag_level = 0;
int last_mag_level = -1;

void set_LN(void) {
	tickrate = 6;
	// TODO: This becomes part of the sensor
//	aMode = aMode_LN;
#if MAG_ENABLED
//	gMode = gMode_LN;
//	MBW = MBW_400Hz;
	MODR = MODR_200Hz;
#endif
}

void set_LP(void) {
	tickrate = 33;
	// TODO: This becomes part of the sensor
//	aMode = aMode_LP;
#if MAG_ENABLED
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
	//i2c_reg_write_byte_dt(&mag, MMC5983MA_CONTROL_1, MBW); // set mag bandwidth
	i2c_reg_write_byte_dt(&mag, MMC5983MA_CONTROL_2, 0x80 | (MSET << 4) | 0x08 | MODR); // set mag ODR
}

void configure_system_off_WOM(const struct i2c_dt_spec imu)
{
	// Configure WOM interrupt
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios), NRF_GPIO_PIN_SENSE_LOW);
	// Store the last quats
	for (uint8_t i = 0; i < 4; i++){
		retained.q[i] = q[i];
	}
	retained.stored_quats = true;
	// Store fusion gyro offset
	for (uint8_t i = 0; i < 3; i++){
		retained.gOff[i] = gOff[i];
	}
	retained_update();
	// Set system off
	icm_setup_WOM(imu); // enable WOM feature
	sys_poweroff();
}

void configure_system_off_chgstat(void){
//	// Configure chgstat interrupt
//	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL);
//	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chgstat_gpios), NRF_GPIO_PIN_PULLUP);
//	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chgstat_gpios), NRF_GPIO_PIN_SENSE_LOW);
	// Configure dock interrupt
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_PULLUP); // Still works
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_LOW);
	// Store fusion gyro offset
	for (uint8_t i = 0; i < 3; i++){
		retained.gOff[i] = gOff[i];
	}
	retained_update();
	// Set system off
	sys_poweroff();
}

void configure_system_off_dock(void){
	// Configure dock interrupt
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL); // Still works
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_HIGH);
	// Store fusion gyro offset
	for (uint8_t i = 0; i < 3; i++){
		retained.gOff[i] = gOff[i];
	}
	retained_update();
	// Set system off
	sys_poweroff();
}

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

bool wait_for_motion(const struct i2c_dt_spec imu, bool motion, int samples) {
	uint8_t counts = 0;
	float a[3], last_a[3];
	icm_accel_read(imu, last_a);
	for (int i = 0; i < samples + counts; i++) {
gpio_pin_toggle_dt(&led); // scuffed led
		LOG_INF("Accel: %.5f %.5f %.5f", a[0], a[1], a[2]);
		k_msleep(500);
		icm_accel_read(imu, a);
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
	main_running = true;
	while (1) {
		if (main_ok)
		{
			// Reading IMUs will take between 2.5ms (~7 samples, low noise) - 7ms (~33 samples, low power)
			// Magneto sample will take ~400us
			// Fusing data will take between 100us (~7 samples, low noise) - 500us (~33 samples, low power)
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

#if MAG_ENABLED
			float mx = 0, my = 0, mz = 0;
			if (last_powerstate == 0) {
				float m[3];
				mmc_mag_read(main_mag, m);
				magneto_sample(m[0], m[1], m[2], ata, &norm_sum, &sample_count); // 400us
				apply_BAinv(m, magBAinv);
				mx = m[0];
				my = m[1];
				mz = m[2];
				int new_magCal = magCal;
				new_magCal |= (-1.2 < ax && ax < -0.8 ? 1 << 0 : 0) | (1.2 > ax && ax > 0.8 ? 1 << 1 : 0) | // dumb check if all accel axes were reached for cal, assume the user is intentionally doing this
					(-1.2 < ay && ay < -0.8 ? 1 << 2 : 0) | (1.2 > ay && ay > 0.8 ? 1 << 3 : 0) |
					(-1.2 < az && az < -0.8 ? 1 << 4 : 0) | (1.2 > az && az > 0.8 ? 1 << 5 : 0);
				if (new_magCal > magCal && new_magCal == last_magCal) {
					if (k_uptime_get() > magCal_time) {
						magCal = new_magCal;
						LOG_INF("Progress magCal: %d", new_magCal);
					}
				} else {
					magCal_time = k_uptime_get() + 1000;
					last_magCal = new_magCal;
				}
				if (magCal == 0b111111) {
					gpio_pin_set_dt(&led, 1);
				}
			}

			if (reconfig) {
				switch (powerstate) {
					case 0:
						set_LN();
						LOG_INF("Switch main imus to low noise");
						last_mag_level = -1;
						break;
					case 1:
						set_LP();
						LOG_INF("Switch main imus to low power");
						reconfigure_mag(main_mag);
						break;
				};
				//reconfigure_imu(main_imu); // Reconfigure if needed
				//reconfigure_mag(main_mag); // Reconfigure if needed
			}
			if (last_mag_level != mag_level && powerstate == 0) {
				switch (mag_level) {
					case 0:
						MODR = MODR_10Hz;
						LOG_INF("Switch mag to 10Hz");
						break;
					case 1:
						MODR = MODR_20Hz;
						LOG_INF("Switch mag to 20Hz");
						break;
					case 2:
						MODR = MODR_50Hz;
						LOG_INF("Switch mag to 50Hz");
						break;
					case 3:
						MODR = MODR_100Hz;
						LOG_INF("Switch mag to 100Hz");
						break;
					case 4:
						MODR = MODR_200Hz;
						LOG_INF("Switch mag to 200Hz");
						break;
				};
				reconfigure_mag(main_mag);
			}
			last_mag_level = mag_level;
			mag_level = 0;
#endif

			FusionVector z = {.array = {0, 0, 0}};
			if (packets == 2 && powerstate == 1 && MAG_ENABLED) {
					ahrs.initialising = true;
					ahrs.rampedGain = 10.0f;
					ahrs.accelerometerIgnored = false;
					ahrs.accelerationRecoveryTrigger = 0;
					ahrs.accelerationRecoveryTimeout = 0;
					FusionVector a = {.array = {ax, -az, ay}};
					FusionAhrsUpdate(&ahrs, z, a, z, INTEGRATION_TIME_LP);
					memcpy(q, ahrs.quaternion.array, sizeof(q));
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
					float gyro_speed_square = g.array[0]*g.array[0] + g.array[1]*g.array[1] + g.array[2]*g.array[2];
					// target mag ODR for ~0.25 deg error
					if (gyro_speed_square > 25*25 && mag_level < 3) // >25dps -> 200hz ODR
						mag_level = 4;
					else if (gyro_speed_square > 12*12 && mag_level < 2) // 12-25dps -> 100hz ODR
						mag_level = 3;
					else if (gyro_speed_square > 5*5 && mag_level < 1) // 5-12dps -> 50hz ODR
						mag_level = 2;
					else if (gyro_speed_square > 2*2 && mag_level < 1) // 2-5dps -> 20hz ODR
						mag_level = 1;
					// <2dps -> 10hz ODR
					FusionVector m = {.array = {my, mz, -mx}};
					if (offset.timer < offset.timeout)
						FusionAhrsUpdate(&ahrs, g, a, m, INTEGRATION_TIME);
					else
						FusionAhrsUpdate(&ahrs, z, a, m, INTEGRATION_TIME);
#else
					if (offset.timer < offset.timeout)
						FusionAhrsUpdate(&ahrs, g, a, z, INTEGRATION_TIME);
					else
						FusionAhrsUpdate(&ahrs, z, a, z, INTEGRATION_TIME);
#endif
				}
				const FusionVector lin_a = FusionAhrsGetLinearAcceleration(&ahrs); // im going insane
				lin_ax = lin_a.array[0] * CONST_EARTH_GRAVITY; // Also change to m/s for SlimeVR server
				lin_ay = lin_a.array[1] * CONST_EARTH_GRAVITY;
				lin_az = lin_a.array[2] * CONST_EARTH_GRAVITY;
				memcpy(q, ahrs.quaternion.array, sizeof(q));
				memcpy(gOff, offset.gyroscopeOffset.array, sizeof(gOff));
			}

			if (quat_epsilon_coarse(q, last_q)) { // Probably okay to use the constantly updating last_q
				int64_t imu_timeout = CLAMP(last_data_time, 1 * 1000, 15 * 1000); // Ramp timeout from last_data_time
				if (k_uptime_get() - last_data_time > imu_timeout) { // No motion in last 1s - 10s
					LOG_INF("No motion from main imus in %llds", imu_timeout/1000);
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
				tx_buf[0] = TO_FIXED_15(q_offset[3]);
				tx_buf[1] = TO_FIXED_15(q_offset[0]);
				tx_buf[2] = TO_FIXED_15(q_offset[1]);
				tx_buf[3] = TO_FIXED_15(q_offset[2]);
				tx_buf[4] = TO_FIXED_7(lin_ax);
				tx_buf[5] = TO_FIXED_7(lin_ay);
				tx_buf[6] = TO_FIXED_7(lin_az);
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
//				esb_flush_tx();
				main_data = true;
//				esb_write_payload(&tx_payload); // Add transmission to queue
				send_data = true;
			}
		} else {
// 5ms delta (???) from entering loop
// skip sleep, surely this wont cause issues :D
/*
			int64_t time_delta = k_uptime_get() - start_time;
			if (time_delta < 11)
			{
				k_msleep(11 - time_delta);
			}
			//k_msleep(11);														 // Wait for start up (1ms for ICM, 10ms for MMC -> 10ms)
*/
			uint8_t ICM42688ID = icm_getChipID(main_imu);						 // Read CHIP_ID register for ICM42688
				LOG_INF("ICM: %u", ICM42688ID);
			uint8_t MMC5983ID = mmc_getChipID(main_mag);						 // Read CHIP_ID register for MMC5983MA
				LOG_INF("MMC: %u", MMC5983ID);
			if ((ICM42688ID == 0x47 || ICM42688ID == 0xDB) && MMC5983ID == 0x30) // check if all I2C sensors have acknowledged
			{
				LOG_INF("Found main imus");
				i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // i dont wanna wait on icm!!
				i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Reset MMC now to avoid waiting 10ms later
				//icm_reset(main_imu);												 // software reset ICM42688 to default registers
				uint8_t temp;
				i2c_reg_read_byte_dt(&main_imu, ICM42688_INT_STATUS, &temp); // clear reset done int flag
				icm_init(main_imu, Ascale, Gscale, AODR, GODR, aMode, gMode, false); // configure
// 55-66ms delta to wait, get chip ids, and setup icm (50ms spent waiting for accel and gyro to start)
#if MAG_ENABLED
				mmc_SET(main_mag);													 // "deGauss" magnetometer
				mmc_init(main_mag, MODR, MBW, MSET);								 // configure
// 0-1ms delta to setup mmc
#endif
				LOG_INF("Initialized main imus");
				main_ok = true;
				main_running = false;
				k_sleep(K_FOREVER); // Wait for after calibrations have loaded the first time
				main_running = true;
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
					if (!nvs_init) {
						nvs_mount(&fs);
						nvs_init = true;
					}
					nvs_write(&fs, MAIN_ACCEL_BIAS_ID, &accelBias, sizeof(accelBias));
					nvs_write(&fs, MAIN_GYRO_BIAS_ID, &gyroBias, sizeof(gyroBias));
					memcpy(retained.accelBias, accelBias, sizeof(accelBias));
					memcpy(retained.gyroBias, gyroBias, sizeof(gyroBias));
					retained_update();
					LOG_INF("%.5f %.5f %.5f", accelBias[0], accelBias[1], accelBias[2]);
					LOG_INF("%.5f %.5f %.5f", gyroBias[0], gyroBias[1], gyroBias[2]);
					LOG_INF("Finished accel and gyro zero offset calibration");
					// clear fusion gyro offset
					for (uint8_t i = 0; i < 3; i++){
						gOff[i] = 0;
						retained.gOff[i] = gOff[i];
					}
					retained_update();
gpio_pin_set_dt(&led, 0); // scuffed led
					reset_mode = 0; // Clear reset mode
				}
				} while (false);
				// Setup fusion
				LOG_INF("Init fusion");
				FusionOffsetInitialise(&offset, 1/INTEGRATION_TIME);
				FusionAhrsInitialise(&ahrs);
				// ahrs.initialising = true; // cancel fusion init, maybe only if there is a quat stored? oh well
				const FusionAhrsSettings settings = {
						.convention = FusionConventionNwu,
						.gain = 0.5f,
						.gyroscopeRange = 2000.0f, // also change gyro range in fusion! (.. does it actually work if its set to the limit?)
						.accelerationRejection = 10.0f,
						.magneticRejection = 20.0f,
						.recoveryTriggerPeriod = 5 * 1/INTEGRATION_TIME, // 5 seconds
				};
				FusionAhrsSetSettings(&ahrs, &settings);
				memcpy(ahrs.quaternion.array, q, sizeof(q)); // Load existing quat
				memcpy(offset.gyroscopeOffset.array, gOff, sizeof(gOff)); // Load fusion gyro offset
				LOG_INF("Initialized fusion");
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

void wait_for_threads(void) {
	if (threads_running || main_running) {
		while (main_running) {
			k_usleep(1);
		}
	}
}

void power_check(void) {
	bool docked = gpio_pin_get_dt(&dock);
	int batt_mV;
	uint32_t batt_pptt = read_batt_mV(&batt_mV);
	if (batt_pptt == 0 && !docked) {
		// Communicate all imus to shut down
		i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
		i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
		gpio_pin_set_dt(&led, 0); // Turn off LED
		configure_system_off_chgstat();
	} else if (docked) {
		// Communicate all imus to shut down
		i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
		i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
		gpio_pin_set_dt(&led, 0); // Turn off LED
		configure_system_off_dock(); // usually charging, i would flash LED but that will drain the battery while it is charging..
	}
	LOG_INF("Battery %u%% (%dmV)", batt_pptt/100, batt_mV);
}

int main(void)
{
	int32_t reset_reason = NRF_POWER->RESETREAS;
	NRF_POWER->RESETREAS = NRF_POWER->RESETREAS; // Clear RESETREAS
	uint8_t reboot_counter = 0;
	bool booting_from_shutdown = false;

	gpio_pin_configure_dt(&dock, GPIO_INPUT);
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);

	power_check(); // check the battery and dock first before continuing (4ms delta to read from ADC)

//	start_time = k_uptime_get(); // Need to get start time for imu startup delta
	gpio_pin_set_dt(&led, 1); // Boot LED

	bool ram_retention = retained_validate(); // check ram retention

	if (reset_reason & 0x01) { // Count pin resets
		//nvs_read(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
		reboot_counter = retained.reboot_counter;
		if (reboot_counter > 200) reboot_counter = 200; // How did you get here
		booting_from_shutdown = reboot_counter == 0 ? true : false; // 0 means from user shutdown or failed ram validation
		if (reboot_counter == 0) reboot_counter = 100;
		reset_mode = reboot_counter - 100;
		reboot_counter++;
		//nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
		retained.reboot_counter = reboot_counter;
		retained_update();
		LOG_INF("Reset Count: %u", reboot_counter);
		k_msleep(1000); // Wait before clearing counter and continuing
		reboot_counter = 100;
		//nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
		retained.reboot_counter = reboot_counter;
		retained_update();
	}
// 0ms or 1000ms delta for reboot counter

#if USER_SHUTDOWN_ENABLED
	if (reset_mode == 0 && !booting_from_shutdown) { // Reset mode user shutdown
		reboot_counter = 0;
		//nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
		retained.reboot_counter = reboot_counter;
		retained_update();
		bool docked = gpio_pin_get_dt(&dock);
		if (!docked) { // TODO: should the tracker start again if docking state changes?
			// Communicate all imus to shut down
			i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
			i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
			gpio_pin_set_dt(&led, 0); // Turn off LED
			configure_system_off_chgstat();
		} else {
			// Communicate all imus to shut down
			i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
			i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
			gpio_pin_set_dt(&led, 0); // Turn off LED
			configure_system_off_dock(); // usually charging, i would flash LED but that will drain the battery while it is charging..
		}
	}
// How long user shutdown take does not matter really ("0ms")
#endif

	struct flash_pages_info info;
	fs.flash_device = NVS_PARTITION_DEVICE;
	fs.offset = NVS_PARTITION_OFFSET; // Start NVS FS here
	flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	fs.sector_size = info.size; // Sector size equal to page size
	fs.sector_count = 4U; // 4 sectors
// 5-6ms delta to initialize NVS (only done when needed)

	// All contents of NVS was stored in RAM to not need initializing NVS often
	if (!ram_retention) { 
		LOG_INF("Invalidated RAM");
		if (!nvs_init) {
			nvs_mount(&fs);
			nvs_init = true;
		}
		nvs_read(&fs, PAIRED_ID, &retained.paired_addr, sizeof(paired_addr));
		nvs_read(&fs, MAIN_ACCEL_BIAS_ID, &retained.accelBias, sizeof(accelBias));
		nvs_read(&fs, MAIN_GYRO_BIAS_ID, &retained.gyroBias, sizeof(gyroBias));
		nvs_read(&fs, MAIN_MAG_BIAS_ID, &retained.magBAinv, sizeof(magBAinv));
		retained_update();
	} else {
		LOG_INF("Recovered calibration from RAM");
	}

	memcpy(paired_addr, retained.paired_addr, sizeof(paired_addr));

	gpio_pin_set_dt(&led, 0);

// TODO: if reset counter is 0 but reset reason was 1 then perform imu scanning (pressed reset once)
	if (reset_mode == 0) { // Reset mode scan imus
	}
// ?? delta

	if (reset_mode == 3) { // Reset mode pairing reset
		LOG_INF("Enter pairing reset");
		if (!nvs_init) {
			nvs_mount(&fs);
			nvs_init = true;
		}
		nvs_write(&fs, PAIRED_ID, &paired_addr, sizeof(paired_addr)); // Clear paired address
		memcpy(retained.paired_addr, paired_addr, sizeof(paired_addr));
		retained_update();
		reset_mode = 0; // Clear reset mode
	} else {
		// Read paired address from NVS
		//nvs_read(&fs, PAIRED_ID, &paired_addr, sizeof(paired_addr));
		memcpy(paired_addr, retained.paired_addr, sizeof(paired_addr));
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
//	timer_init();
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
			esb_start_tx();
			gpio_pin_set_dt(&led, 1);
			k_msleep(100);
			gpio_pin_set_dt(&led, 0);
			k_msleep(900);
			power_check();
		}
		LOG_INF("Paired");
		if (!nvs_init) {
			nvs_mount(&fs);
			nvs_init = true;
		}
		nvs_write(&fs, PAIRED_ID, &paired_addr, sizeof(paired_addr)); // Write new address and tracker id
		memcpy(retained.paired_addr, paired_addr, sizeof(paired_addr));
		retained_update();
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
	//nvs_read(&fs, MAIN_ACCEL_BIAS_ID, &accelBias, sizeof(accelBias));
	//nvs_read(&fs, MAIN_GYRO_BIAS_ID, &gyroBias, sizeof(gyroBias));
	//nvs_read(&fs, MAIN_MAG_BIAS_ID, &magBAinv, sizeof(magBAinv));
	memcpy(accelBias, retained.accelBias, sizeof(accelBias));
	memcpy(gyroBias, retained.gyroBias, sizeof(gyroBias));
	memcpy(magBAinv, retained.magBAinv, sizeof(magBAinv));
	LOG_INF("Read calibrations");
	LOG_INF("Main accel bias: %.5f %.5f %.5f", accelBias[0], accelBias[1], accelBias[2]);
	LOG_INF("Main gyro bias: %.5f %.5f %.5f", gyroBias[0], gyroBias[1], gyroBias[2]);
	LOG_INF("Main mag matrix:");
	for (int i = 0; i < 3; i++) {
		LOG_INF("%.5f %.5f %.5f %.5f", magBAinv[0][i], magBAinv[1][i], magBAinv[2][i], magBAinv[3][i]);
	}

	//gpio_pin_configure_dt(&chgstat, GPIO_INPUT);
	//gpio_pin_configure_dt(&int0, GPIO_INPUT);
	//gpio_pin_configure_dt(&int1, GPIO_INPUT);

	//pwm_set_pulse_dt(&led0, PWM_MSEC(5)); // 5/20 = 25%
	//gpio_pin_set_dt(&led, 1);

	// Recover quats if present
	//retained_validate();
	if (retained.stored_quats) {
		for (uint8_t i = 0; i < 4; i++){
			q[i] = retained.q[i];
		}
		LOG_INF("Recovered quaternions\nMain: %.2f %.2f %.2f %.2f", q[0], q[1], q[2], q[3]);
		retained.stored_quats = false; // Invalidate the retained quaternions
		retained_update();
	}
	for (uint8_t i = 0; i < 3; i++){
		gOff[i] = retained.gOff[i];
	}
	LOG_INF("Recovered fusion gyro offset\nMain: %.2f %.2f %.2f", gOff[0], gOff[1], gOff[2]);
// 0ms delta to read calibration and configure pins (unknown time to read retained data but probably negligible)
	esb_initialize();
	tx_payload.noack = false;
	timer_init();
// 1ms delta to start ESB
	while (1)
	{
		// Get start time
		int64_t time_begin = k_uptime_get();
		int64_t led_time2 = led_clock * 3 + led_clock_offset; // funny led sync

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

		if (batt_pptt < 1000 || batt_low) { // Under 10% battery left
			batt_low = true;
			pwm_set_pulse_dt(&led0, led_time2 % 600 > 300 ? PWM_MSEC(10) : 0); // 10/20 = 50%
			//gpio_pin_set_dt(&led, led_time2 % 600 > 300 ? 1 : 0);
		} else if (led_time2 < 1000) { // funny led sync
			led_time_off = time_begin + 300;
		} else if (time_begin > led_time_off) {
			pwm_set_pulse_dt(&led0, 0);
			//gpio_pin_set_dt(&led, 0);
		} else {
			pwm_set_pulse_dt(&led0, PWM_MSEC(20)); // 20/20 = 100% - this is pretty bright lol
			//gpio_pin_set_dt(&led, 1);
		}

		if (docked) // TODO: keep sending battery state while plugged and docked?
		{ // TODO: move to interrupts? (Then you do not need to do the above)
			LOG_INF("Waiting for system off (Docked)");
			wait_for_threads();
			LOG_INF("Shutdown");
			// Communicate all imus to shut down
			i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
			i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
			// Turn off LED
			gpio_pin_set_dt(&led, 0);
			configure_system_off_dock();
		}

		if (system_off_main) { // System off on extended no movement
			LOG_INF("Waiting for system off (No movement)");
			wait_for_threads();
			LOG_INF("Shutdown");
			// Communicate all imus to shut down
			icm_reset(main_imu);
			i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
			// Turn off LED
			gpio_pin_set_dt(&led, 0);
			configure_system_off_WOM(main_imu);
		}

		reconfig = last_powerstate != powerstate ? true : false;
		last_powerstate = powerstate;
		main_data = false;

		wait_for_threads();
		k_wakeup(main_imu_thread_id);
		threads_running = true;

#if MAG_ENABLED
		// Save magCal while idling
		if (magCal == 0b111111 && last_powerstate == 1) {
			if (!nvs_init) {
				nvs_mount(&fs);
				nvs_init = true;
			}
			k_usleep(1); // yield to imu thread first
			wait_for_threads(); // make sure not to interrupt anything (8ms)
			LOG_INF("Calibrating magnetometer");
			magneto_current_calibration(magBAinv, ata, norm_sum, sample_count); // 25ms
			nvs_write(&fs, MAIN_MAG_BIAS_ID, &magBAinv, sizeof(magBAinv));
			memcpy(retained.magBAinv, magBAinv, sizeof(magBAinv));
			retained_update();
			for (int i = 0; i < 3; i++) {
				LOG_INF("%.5f %.5f %.5f %.5f", magBAinv[0][i], magBAinv[1][i], magBAinv[2][i], magBAinv[3][i]);
			}
			LOG_INF("Finished mag hard/soft iron offset calibration");
			//magCal |= 1 << 7;
			magCal = 0;
			// clear data
			//memset(ata[0], 0, sizeof(ata)); // does this work??
			for (int i = 0; i < 100; i++) {
				ata[i] = 0.0;
			}
			norm_sum = 0.0;
			sample_count = 0.0;
		}
#endif

		// Get time elapsed and sleep/yield until next tick
		int64_t time_delta = k_uptime_get() - time_begin;
		led_clock_offset += time_delta;
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
