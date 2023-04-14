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

LOG_MODULE_REGISTER(main, 3);

static struct nvs_fs fs;

#define NVS_PARTITION		storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define RBT_CNT_ID 1
#define PAIRED_ID 2
#define MAIN_ACCEL_BIAS_ID 3
#define MAIN_GYRO_BIAS_ID 4
#define MAIN_MAG_BIAS_ID 5
#define MAIN_MAG_SCALE_ID 6
#define MAIN_MAG_OFFSET_ID 7
#define AUX_ACCEL_BIAS_ID 8
#define AUX_GYRO_BIAS_ID 9
#define AUX_MAG_BIAS_ID 10
#define AUX_MAG_SCALE_ID 11
#define AUX_MAG_OFFSET_ID 12

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

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

int TICKRATE_MS = 6;

unsigned int batt_pptt;

const struct i2c_dt_spec main_imu = I2C_DT_SPEC_GET(MAIN_IMU_NODE);
const struct i2c_dt_spec main_mag = I2C_DT_SPEC_GET(MAIN_MAG_NODE);
const struct i2c_dt_spec aux_imu = I2C_DT_SPEC_GET(AUX_IMU_NODE);
const struct i2c_dt_spec aux_mag = I2C_DT_SPEC_GET(AUX_MAG_NODE);

//const struct pwm_dt_spec led0 = PWM_DT_SPEC_GET(LED0_NODE);
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_gpios);

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

#include "ICM42688.h"
#include "MMC5983MA.h"

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define pi 3.141592653589793238462643383279502884f
#define GyroMeasError pi * (40.0f / 180.0f)		// gyroscope measurement error in rads/s (start at 40 deg/s)
#define GyroMeasDrift pi * (0.0f / 180.0f)		// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
#define beta sqrtf(3.0f / 4.0f) * GyroMeasError // compute beta
#define zeta sqrtf(3.0f / 4.0f) * GyroMeasDrift // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float lin_ax, lin_ay, lin_az;					// linear acceleration (acceleration with gravity component subtracted)
float lin_ax2, lin_ay2, lin_az2;					// linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};			// vector to hold quaternion
float q2[4] = {1.0f, 0.0f, 0.0f, 0.0f};			// vector to hold quaternion
float last_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};		// vector to hold quaternion
float last_q2[4] = {1.0f, 0.0f, 0.0f, 0.0f};	// vector to hold quaternion

float q3[4] = {-0.5f, 0.5f, 0.5f, 0.5f}; // correction quat

int tracker_id = 0;

// storing temporary values
float mx, my, mz; // variables to hold latest mag data values
float mx2, my2, mz2; // variables to hold latest mag data values
uint16_t tx_buf[7];
int64_t start_time;
int64_t last_data_time;
unsigned int last_batt_pptt[16] = {10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001};
int8_t last_batt_pptt_i = 0;
int64_t led_time = 0;
int64_t led_time_off = 0;
uint8_t reset_mode = 0;
bool system_off_main = false;
bool system_off_aux = false;
bool reconfig;
bool charging = false;

// ICM42688 definitions

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
	  AFS_2G, AFS_4G, AFS_8G, AFS_16G
	  GFS_15_625DPS, GFS_31_25DPS, GFS_62_5DPS, GFS_125DPS, GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
	  AODR_1_5625Hz, AODR_3_125Hz, AODR_6_25Hz, AODR_50AODR_12_5Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_500Hz,
	  AODR_1kHz, AODR_2kHz, AODR_4kHz, AODR_8kHz, AODR_16kHz, AODR_32kHz
	  GODR_12_5Hz, GODR_25Hz, GODR_50Hz, GODR_100Hz, GODR_200Hz, GODR_500Hz, GODR_1kHz, GODR_2kHz, GODR_4kHz, GODR_8kHz, GODR_16kHz, GODR_32kHz
*/
uint8_t Ascale = AFS_8G, Gscale = GFS_1000DPS, AODR = AODR_200Hz, GODR = GODR_1kHz, aMode = aMode_LN, gMode = gMode_LN;
#define INTEGRATION_TIME 0.001
#define INTEGRATION_TIME_LP 0.005

float aRes, gRes;														   // scale resolutions per LSB for the accel and gyro sensor2
// TODO: make sure these are separate for main vs. aux (and also store/read them!)
float accelBias[3] = {0.0f, 0.0f, 0.0f}, gyroBias[3] = {0.0f, 0.0f, 0.0f}; // offset biases for the accel and gyro

float accelBias2[3] = {0.0f, 0.0f, 0.0f}, gyroBias2[3] = {0.0f, 0.0f, 0.0f}; // offset biases for the accel and gyro

// MMC5983MA definitions

/* Specify sensor parameters (continuous mode sample rate is dependent on bandwidth)
 * choices are: MODR_ONESHOT, MODR_1Hz, MODR_10Hz, MODR_20Hz, MODR_50 Hz, MODR_100Hz, MODR_200Hz (BW = 0x01), MODR_1000Hz (BW = 0x03)
 * Bandwidth choices are: MBW_100Hz, MBW_200Hz, MBW_400Hz, MBW_800Hz
 * Set/Reset choices are: MSET_1, MSET_25, MSET_75, MSET_100, MSET_250, MSET_500, MSET_1000, MSET_2000, so MSET_100 set/reset occurs every 100th measurement, etc.
 */
uint8_t MODR = MODR_200Hz, MBW = MBW_400Hz, MSET = MSET_2000;

float mRes = 1.0f / 16384.0f;											   // mag sensitivity if using 18 bit data
// TODO: make sure these are separate for main vs. aux (and also store/read them!)
float magBias[3] = {0, 0, 0}, magScale[3] = {1, 1, 1}; // Bias corrections for magnetometer
uint32_t MMC5983MAData[3];												   // Stores the 18-bit unsigned magnetometer sensor output
#define MMC5983MA_offset 131072.0f // mag range unsigned to signed

float magBias2[3] = {0, 0, 0}, magScale2[3] = {1, 1, 1}; // Bias corrections for magnetometer
uint32_t MMC5983MAData2[3];												   // Stores the 18-bit unsigned magnetometer sensor output

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
__attribute__((optimize("O3"))) void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float ts, float *q, bool mag)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	if (mag) {
		// Normalise magnetometer measurement
		norm = sqrtf(mx * mx + my * my + mz * mz);
		if (norm == 0.0f)
			return; // handle NaN
		norm = 1.0f / norm;
		mx *= norm;
		my *= norm;
		mz *= norm;

		// Reference direction of Earth's magnetic field
		_2q1mx = 2.0f * q1 * mx;
		_2q1my = 2.0f * q1 * my;
		_2q1mz = 2.0f * q1 * mz;
		_2q2mx = 2.0f * q2 * mx;
		hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
		hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
		_2bx = sqrtf(hx * hx + hy * hy);
		_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	} else {
		// Gradient decent algorithm corrective step
		s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay);
		s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az);
		s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az);
		s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay);
	}
	norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
	norm = 1.0f / norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * ts;
	q2 += qDot2 * ts;
	q3 += qDot3 * ts;
	q4 += qDot4 * ts;
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // normalise quaternion
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}

void q_multiply(float *x, float *y, float *out) {
	out[0] = x[0]*y[0] - x[1]*y[1] - x[2]*y[2] - x[3]*y[3];
	out[1] = x[1]*y[0] + x[0]*y[1] - x[3]*y[2] + x[2]*y[3];
	out[2] = x[2]*y[0] + x[3]*y[1] + x[0]*y[2] - x[1]*y[3];
	out[3] = x[3]*y[0] - x[2]*y[1] + x[1]*y[2] + x[0]*y[3];
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
				LOG_INF("RX RECEIVED");
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
	TICKRATE_MS = 6;
	aMode = aMode_LN;
	gMode = gMode_LN;
	MBW = MBW_400Hz;
	MODR = MODR_200Hz;
}

void set_LP(void) {
	TICKRATE_MS = 33;
	aMode = aMode_LP;
	gMode = gMode_SBY;
	MBW = MBW_800Hz;
	MODR = MODR_ONESHOT;
}

void reconfigure_imu(const struct i2c_dt_spec imu) {
	aRes = icm_getAres(Ascale);
	gRes = icm_getGres(Gscale);
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
	icm_DRStatus(imu); // clear reset done int flag
	i2c_reg_write_byte_dt(&imu, ICM42688_INT_SOURCE0, 0x00); // temporary disable interrupts
	i2c_reg_write_byte_dt(&imu, ICM42688_ACCEL_CONFIG0, Ascale << 5 | AODR_200Hz); // set accel ODR and FS
	i2c_reg_write_byte_dt(&imu, ICM42688_PWR_MGMT0, aMode_LP); // set accel and gyro modes
	i2c_reg_write_byte_dt(&imu, ICM42688_INTF_CONFIG1, 0x00); // set low power clock
	k_busy_wait(1000);
	i2c_reg_write_byte_dt(&imu, ICM42688_REG_BANK_SEL, 0x04); // select register bank 4
	i2c_reg_write_byte_dt(&imu, ICM42688_ACCEL_WOM_X_THR, 0x08); // set wake thresholds // 80 x 3.9 mg is ~312 mg
	i2c_reg_write_byte_dt(&imu, ICM42688_ACCEL_WOM_Y_THR, 0x08); // set wake thresholds
	i2c_reg_write_byte_dt(&imu, ICM42688_ACCEL_WOM_Z_THR, 0x08); // set wake thresholds
	k_busy_wait(1000);
	i2c_reg_write_byte_dt(&imu, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
	i2c_reg_write_byte_dt(&imu, ICM42688_INT_SOURCE1, 0x07); // enable WOM interrupt
	k_busy_wait(50000);
	i2c_reg_write_byte_dt(&imu, ICM42688_SMD_CONFIG, 0x05); // enable WOM feature
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
			count += 16; // Add two read buffer packets
			uint16_t packets = count / 8;								 // Packet size 8 bytes
			uint8_t rawData[2080];
			i2c_burst_read_dt(&main_imu, ICM42688_FIFO_DATA, &rawData[0], count); // Read buffer
    		uint8_t rawAccel[6];
    		i2c_burst_read_dt(&main_imu, ICM42688_ACCEL_DATA_X1, &rawAccel[0], 6); // Read accel only
			float raw0 = (int16_t)((((int16_t)rawAccel[0]) << 8) | rawAccel[1]);
			float raw1 = (int16_t)((((int16_t)rawAccel[2]) << 8) | rawAccel[3]);
			float raw2 = (int16_t)((((int16_t)rawAccel[4]) << 8) | rawAccel[5]);
			// transform and convert to float values
			float ax = raw0 * aRes - accelBias[0];
			float ay = raw1 * aRes - accelBias[1];
			float az = raw2 * aRes - accelBias[2];
#if (MAG_ENABLED == true)
			if (last_powerstate == 0) {
				mmc_readData(main_mag, MMC5983MAData);
				// transform and convert to float values
				// TODO: make sure these are reading to separate buffers for main vs. aux
				mx = ((float)MMC5983MAData[0] - MMC5983MA_offset) * mRes - magBias[0];
				my = ((float)MMC5983MAData[1] - MMC5983MA_offset) * mRes - magBias[1];
				mz = ((float)MMC5983MAData[2] - MMC5983MA_offset) * mRes - magBias[2];
				mx *= magScale[0];
				my *= magScale[1];
				mz *= magScale[2];
			}
#endif

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
				reconfigure_imu(main_imu); // Reconfigure if needed
#if (MAG_ENABLED == true)
				reconfigure_mag(main_mag); // Reconfigure if needed
#endif
			}

			if (packets == 2 && powerstate == 1) {
				MadgwickQuaternionUpdate(ax, -az, ay, 0, 0, 0, my, mz, -mx, INTEGRATION_TIME_LP, q, MAG_ENABLED);
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
					float gx = raw0 * gRes - gyroBias[0];
					float gy = raw1 * gRes - gyroBias[1];
					float gz = raw2 * gRes - gyroBias[2];
					// TODO: swap out fusion?
					MadgwickQuaternionUpdate(ax, -az, ay, gx * pi / 180.0f, -gz * pi / 180.0f, gy * pi / 180.0f, my, mz, -mx, INTEGRATION_TIME, q, MAG_ENABLED);
					if (i == packets - 1) {
						// Calculate linear acceleration (no gravity)
						lin_ax = ax + 2.0f * (q[0] * q[1] + q[2] * q[3]);
						lin_ay = ay + 2.0f * (q[1] * q[3] - q[0] * q[2]);
						lin_az = az - q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
					}
				}
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
				tx_payload.data[0] = tracker_id << 4;
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
			uint8_t MMC5983ID = mmc_getChipID(main_mag);						 // Read CHIP_ID register for MMC5983MA
			if ((ICM42688ID == 0x47 || ICM42688ID == 0xDB) && MMC5983ID == 0x30) // check if all I2C sensors have acknowledged
			{
				LOG_INF("Found main imus");
    			i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Reset MMC now to avoid waiting 10ms later
				icm_reset(main_imu);												 // software reset ICM42688 to default registers
				icm_DRStatus(main_imu);												 // clear reset done int flag
				icm_init(main_imu, Ascale, Gscale, AODR, GODR, aMode, gMode, false); // configure
// 55-66ms delta to wait, get chip ids, and setup icm (50ms spent waiting for accel and gyro to start)
#if (MAG_ENABLED == true)
				mmc_SET(main_mag);													 // "deGauss" magnetometer
				mmc_init(main_mag, MODR, MBW, MSET);								 // configure
#endif
// 0-1ms delta to setup mmc
				main_ok = true;
				if (reset_mode == 1) { // Reset mode main calibration
					LOG_INF("Enter main calibration");
gpio_pin_set_dt(&led, 0); // scuffed led
					// TODO: Add LED flashies
					// TODO: Wait for accelerometer to settle, then calibrate (5 sec timeout to skip)
					icm_offsetBias(main_imu, accelBias, gyroBias); // This takes about 750ms
					nvs_write(&fs, MAIN_ACCEL_BIAS_ID, &accelBias, sizeof(accelBias));
					nvs_write(&fs, MAIN_GYRO_BIAS_ID, &gyroBias, sizeof(gyroBias));
					LOG_INF("Finished accel and gyro zero offset calibration");
gpio_pin_set_dt(&led, 1); // scuffed led
					// TODO: Wait for accelerometer movement, then calibrate (5 sec timeout to skip)
#if (MAG_ENABLED == true)
					mmc_offsetBias(main_mag, magBias, magScale); // This takes about 20s
					nvs_write(&fs, MAIN_MAG_BIAS_ID, &magBias, sizeof(magBias));
					nvs_write(&fs, MAIN_MAG_SCALE_ID, &magScale, sizeof(magScale));
					LOG_INF("Finished mag hard/soft iron offset calibration");
#endif
					reset_mode = 0; // Clear reset mode
				}
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
		k_yield();
	}
}

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

K_THREAD_DEFINE(aux_imu_thread_id, 4096, aux_imu_thread, NULL, NULL, NULL, 7, 0, 0);

void wait_for_threads(void) {
	if (threads_running) {
		while (main_running) {
			while (aux_running) {
				k_yield();
			}
		}
	}
}

void main(void)
{
	start_time = k_uptime_get(); // ~75ms from start_time to first data sent with main imus only
	// TODO: Need to skip all this junk and check the battery and dock first
	int32_t reset_reason = NRF_POWER->RESETREAS;
	NRF_POWER->RESETREAS = NRF_POWER->RESETREAS; // Clear RESETREAS
	uint8_t reboot_counter = 0;

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

	gpio_pin_configure_dt(&led, GPIO_OUTPUT);

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
				paired_addr[0] == 0x00; // Packet not for this device
			}
			esb_flush_rx();
			esb_flush_tx();
			esb_write_payload(&tx_payload_pair); // Still fails after a while
			gpio_pin_set_dt(&led, 1);
			k_msleep(100);
			gpio_pin_set_dt(&led, 0);
			k_msleep(900);
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
	nvs_read(&fs, MAIN_MAG_BIAS_ID, &magBias, sizeof(magBias));
	nvs_read(&fs, MAIN_MAG_SCALE_ID, &magScale, sizeof(magScale));
	nvs_read(&fs, AUX_ACCEL_BIAS_ID, &accelBias2, sizeof(accelBias));
	nvs_read(&fs, AUX_GYRO_BIAS_ID, &gyroBias2, sizeof(gyroBias));
	nvs_read(&fs, AUX_MAG_BIAS_ID, &magBias2, sizeof(magBias));
	nvs_read(&fs, AUX_MAG_SCALE_ID, &magScale2, sizeof(magScale));
	LOG_INF("Read calibrations");
	LOG_INF("Main accel bias: %.5f %.5f %.5f", accelBias[0], accelBias[1], accelBias[2]);
	LOG_INF("Main gyro bias: %.5f %.5f %.5f", gyroBias[0], gyroBias[1], gyroBias[2]);
	LOG_INF("Main mag bias: %.5f %.5f %.5f", magBias[0], magBias[1], magBias[2]);
	LOG_INF("Main mag scale: %.5f %.5f %.5f", magScale[0], magScale[1], magScale[2]);
	LOG_INF("Aux accel bias: %.5f %.5f %.5f", accelBias2[0], accelBias2[1], accelBias2[2]);
	LOG_INF("Aux gyro bias: %.5f %.5f %.5f", gyroBias2[0], gyroBias2[1], gyroBias2[2]);
	LOG_INF("Aux mag bias: %.5f %.5f %.5f", magBias2[0], magBias2[1], magBias2[2]);
	LOG_INF("Aux mag scale: %.5f %.5f %.5f", magScale2[0], magScale2[1], magScale2[2]);

	// get sensor resolutions for user settings, only need to do this once
	// TODO: surely these can be defines lol
	aRes = icm_getAres(Ascale);
	gRes = icm_getGres(Gscale);

	const struct gpio_dt_spec dock = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, dock_gpios);
	const struct gpio_dt_spec chgstat = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, chgstat_gpios);
	//const struct gpio_dt_spec int0 = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, int0_gpios);
	//const struct gpio_dt_spec int1 = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, int1_gpios);

	gpio_pin_configure_dt(&dock, GPIO_INPUT);
	gpio_pin_configure_dt(&chgstat, GPIO_INPUT);
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
// 1ms delta to start ESB
	while (1)
	{
		// Get start time
		int64_t time_begin = k_uptime_get();

		//charging = gpio_pin_get_dt(&chgstat); // TODO: Charging detect doesn't work (hardware issue)
		bool docked = gpio_pin_get_dt(&dock);

		batt_pptt = read_batt();
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
		else {batt_pptt = last_batt_pptt[15];} // Effectively 100-10000 -> 1-100

		if (time_begin > led_time) {
			if (led_time != 0) {
				led_time_off = time_begin + 500;
			}
			if (batt_pptt < 1000) { // Under 10% battery left
				led_time += 1000;
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
		if (time_delta > TICKRATE_MS)
		{
			k_yield();
		}
		else
		{
			k_msleep(TICKRATE_MS - time_delta);
		}
	}
}
