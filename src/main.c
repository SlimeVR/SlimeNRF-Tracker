#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>

//#include <zephyr/bluetooth/bluetooth.h>
//#include <zephyr/bluetooth/hci.h>
//#include <zephyr/bluetooth/conn.h>
//#include <zephyr/bluetooth/uuid.h>
//#include <zephyr/bluetooth/gatt.h>

//#include <bluetooth/services/nsms.h>

#include <zephyr/settings/settings.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "battery.h"

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
//#include <zephyr/logging/log.h>
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

//static bool ready = true;
static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);

#define _RADIO_SHORTS_COMMON                                                   \
	(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk |         \
	 RADIO_SHORTS_ADDRESS_RSSISTART_Msk |                                  \
	 RADIO_SHORTS_DISABLED_RSSISTOP_Msk)

//#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
//#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define LED0_NODE DT_NODELABEL(pwm_led0)

#define MAIN_IMU_NODE DT_NODELABEL(icm_0)
#define MAIN_MAG_NODE DT_NODELABEL(mmc_0)
#define AUX_IMU_NODE DT_NODELABEL(icm_1)
#define AUX_MAG_NODE DT_NODELABEL(mmc_1)

#define TICKRATE_MS 6

bool main_ok = false;
bool aux_ok = false;

bool main_data = false;
bool aux_data = false;








#include "ICM42688.h"
#include "MMC5983MA.h"

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define pi 3.141592653589793238462643383279502884f
#define GyroMeasError pi * (40.0f / 180.0f)     // gyroscope measurement error in rads/s (start at 40 deg/s)
#define GyroMeasDrift pi * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
#define beta sqrtf(3.0f / 4.0f) * GyroMeasError // compute beta
#define zeta sqrtf(3.0f / 4.0f) * GyroMeasDrift // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
uint32_t delt_t = 0;                            // used to control display output rate
uint32_t sumCount = 0;                          // used to control display output rate
float pitch, yaw, roll;                         // absolute orientation
float a12, a22, a31, a32, a33;                  // rotation matrix coefficients for Euler angles and gravity components
float deltat = 0.0f, sum = 0.0f;                // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;       // used to calculate integration interval
uint32_t Now = 0;                               // used to calculate integration interval
float lin_ax, lin_ay, lin_az;                   // linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};          // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};             // vector to hold integral error for Mahony method

// ICM42688 definitions
#define ICM42688_intPin1 8 // interrupt1 pin definitions, data ready
#define ICM42688_intPin2 9 // interrupt2 pin definitions, clock in

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G
      GFS_15_625DPS, GFS_31_25DPS, GFS_62_5DPS, GFS_125DPS, GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
      AODR_1_5625Hz, AODR_3_125Hz, AODR_6_25Hz, AODR_50AODR_12_5Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_500Hz,
      AODR_1kHz, AODR_2kHz, AODR_4kHz, AODR_8kHz, AODR_16kHz, AODR_32kHz
      GODR_12_5Hz, GODR_25Hz, GODR_50Hz, GODR_100Hz, GODR_200Hz, GODR_500Hz, GODR_1kHz, GODR_2kHz, GODR_4kHz, GODR_8kHz, GODR_16kHz, GODR_32kHz
*/
uint8_t Ascale = AFS_16G, Gscale = GFS_250DPS, AODR = AODR_500Hz, GODR = GODR_500Hz, aMode = aMode_LN, gMode = gMode_LN;

float aRes, gRes;                                                          // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {0.0f, 0.0f, 0.0f}, gyroBias[3] = {0.0f, 0.0f, 0.0f}; // offset biases for the accel and gyro
int16_t accelDiff[3] = {0, 0, 0}, gyroDiff[3] = {0, 0, 0};                 // difference betwee ST and normal values
float STratio[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};             // self-test results for the accel and gyro
int16_t ICM42688Data[7];                                                   // Stores the 16-bit signed sensor output
float Gtemperature;                                                        // Stores the real internal gyro temperature in degrees Celsius

bool newICM42688Data = false;

// MMC5983MA definitions
#define MMC5983MA_intPin 5 // interrupt for magnetometer data ready

/* Specify sensor parameters (continuous mode sample rate is dependent on bandwidth)
 * choices are: MODR_ONESHOT, MODR_1Hz, MODR_10Hz, MODR_20Hz, MODR_50 Hz, MODR_100Hz, MODR_200Hz (BW = 0x01), MODR_1000Hz (BW = 0x03)
 * Bandwidth choices are: MBW_100Hz, MBW_200Hz, MBW_400Hz, MBW_800Hz
 * Set/Reset choices are: MSET_1, MSET_25, MSET_75, MSET_100, MSET_250, MSET_500, MSET_1000, MSET_2000, so MSET_100 set/reset occurs every 100th measurement, etc.
 */
uint8_t MODR = MODR_200Hz, MBW = MBW_200Hz, MSET = MSET_2000;

float mRes = 1.0f / 16384.0f;                                              // mag sensitivity if using 18 bit data
float magBias[3] = {0, 0, 0}, magScale[3] = {1, 1, 1}, magOffset[3] = {0}; // Bias corrections for magnetometer
uint32_t MMC5983MAData[3];                                                 // Stores the 18-bit unsigned magnetometer sensor output
uint8_t MMC5983MAtemperature;                                              // Stores the magnetometer temperature register data
float Mtemperature;                                                        // Stores the real internal chip temperature in degrees Celsius
uint8_t MMC5983MAstatus;
float MMC5983MA_offset = 131072.0f;

bool newMMC5983MAData = false;




// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
__attribute__((optimize("O3"))) void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float ts)
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




/*
connection is a thread

main-ish thread timer on <10ms

check batt
if batt low
	shut off?
check charging
check dock
if docked and not charging
	shut off?
check main
if main setup
	read main fifo
	read main mag
	fusion
if main not setup
	setup main
check aux
if aux setup
	read aux fifo
	read aux mag
	fusion
if aux not setup
	setup aux
if docked
	read usb cdc
else
	check connection
	if disconnected
		retry connection
	else
		send data
relinquish thread to rtos (send data)
*/

void event_handler(struct esb_evt const *event)
{
	//ready = true;

	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		//LOG_DBG("TX SUCCESS EVENT");
		break;
	case ESB_EVENT_TX_FAILED:
		//LOG_DBG("TX FAILED EVENT");
		break;
	case ESB_EVENT_RX_RECEIVED:
		while (esb_read_rx_payload(&rx_payload) == 0) {
			//LOG_DBG("Packet received, len %d : "
			//	"0x%02x, 0x%02x, 0x%02x, 0x%02x, "
			//	"0x%02x, 0x%02x, 0x%02x, 0x%02x",
			//	rx_payload.length, rx_payload.data[0],
			//	rx_payload.data[1], rx_payload.data[2],
			//	rx_payload.data[3], rx_payload.data[4],
			//	rx_payload.data[5], rx_payload.data[6],
			//	rx_payload.data[7]);
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
	if (!clk_mgr) {
		//LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		//LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			//LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

	//LOG_DBG("HF clock started");
	return 0;
}

int esb_initialize(void)
{
	int err;
	/* These are arbitrary default addresses. In end user products
	 * different addresses should be used for each set of devices.
	 */
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	//config.protocol = ESB_PROTOCOL_ESB_DPL;
	//config.mode = ESB_MODE_PTX;
	config.event_handler = event_handler;
	//config.bitrate = ESB_BITRATE_2MBPS;
	//config.crc = ESB_CRC_16BIT;
	config.tx_output_power = 4;
	//config.retransmit_delay = 600;
	//config.retransmit_count = 3;
	//config.tx_mode = ESB_TXMODE_AUTO;
	//config.payload_length = 32;
	config.selective_auto_ack = true;

    // Fast startup mode
    NRF_RADIO->MODECNF0 = RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;
	//nrf_radio_modecnf0_set(NRF_RADIO, true, 0);

	err = esb_init(&config);

	if (err) {
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err) {
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err) {
		return err;
	}

	return 0;
}

void main(void)
{
	// get sensor resolutions for user settings, only need to do this once
	// surely these can be defines lol
	aRes = icm_getAres(Ascale);
	gRes = icm_getGres(Gscale);

	const struct gpio_dt_spec dock = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, dock_gpios);
	const struct gpio_dt_spec chgstat = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, chgstat_gpios);
	//const struct gpio_dt_spec int0 = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, int0_gpios);
	//const struct gpio_dt_spec int1 = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, int1_gpios);
	
	const struct pwm_dt_spec led0 = PWM_DT_SPEC_GET(LED0_NODE);

	const struct i2c_dt_spec main_imu = I2C_DT_SPEC_GET(MAIN_IMU_NODE);
	const struct i2c_dt_spec main_mag = I2C_DT_SPEC_GET(MAIN_MAG_NODE);
	//const struct i2c_dt_spec aux_imu = I2C_DT_SPEC_GET(AUX_IMU_NODE);
	//const struct i2c_dt_spec aux_mag = I2C_DT_SPEC_GET(AUX_MAG_NODE);

	gpio_pin_configure_dt(&dock, GPIO_INPUT);
	gpio_pin_configure_dt(&chgstat, GPIO_INPUT);
	//gpio_pin_configure_dt(&int0, GPIO_INPUT);
	//gpio_pin_configure_dt(&int1, GPIO_INPUT);

	//maybey test pwm?
	pwm_set_pulse_dt(&led0, PWM_MSEC(10)); //10/20 = 50%
	// i dunno when to be using the led lol

	int err;

	//LOG_INF("Enhanced ShockBurst ptx sample");

	err = clocks_start();
	if (err) {
		return;
	}

	err = esb_initialize();
	if (err) {
		//LOG_ERR("ESB initialization failed, err %d", err);
		return;
	}

	//LOG_INF("Initialization complete");
	//LOG_INF("Sending test packet");
	tx_payload.noack = false;

	for (;;) {
		// Get start time
		int64_t time_begin = k_uptime_get();

		unsigned int batt_pptt = read_batt();
		if (batt_pptt == 0){
			// Communicate all imus to shut down
			icm_reset(main_imu);
			mmc_reset(main_mag);
			//icm_reset(aux_imu);
			//mmc_reset(aux_mag);
			// Configure chgstat interrupt
			nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chgstat_gpios), NRF_GPIO_PIN_PULLUP);
			nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chgstat_gpios), NRF_GPIO_PIN_SENSE_LOW);
			// Set system off
			pm_state_force(0u, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});
			k_sleep(K_SECONDS(1));
		}
		bool charging = gpio_pin_get_dt(&chgstat);
		bool docked = gpio_pin_get_dt(&dock);
		if (docked && !charging) { //TODO: move to interrupts?
			// Communicate all imus to shut down
			icm_reset(main_imu);
			mmc_reset(main_mag);
			//icm_reset(aux_imu);
			//mmc_reset(aux_mag);
			// Configure dock interrupt
			nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_PULLUP);
			nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_HIGH);
			// Set system off
			pm_state_force(0u, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});
			k_sleep(K_SECONDS(1));
		}

		if (main_ok) {
			//TODO: add calibration steps and store calibration
			//TODO: on any errors set main_ok false and skip
			// Read main FIFO
    		uint8_t rawCount[2];
    		i2c_burst_read_dt(&main_imu, ICM42688_FIFO_COUNTH, &rawCount[0], 2);
    		uint16_t count = (uint16_t)(rawCount[0] << 8 | rawCount[1]); // Turn the 16 bits into a unsigned 16-bit value
			uint16_t packets = count / 16; // Packet size 16 bytes
    		uint8_t rawData[2080];
			//TODO: include read buffer (+2*packetsize)
    		i2c_burst_read_dt(&main_imu, ICM42688_FIFO_DATA, &rawData[0], count); // Read buffer
			mmc_readData(main_mag, MMC5983MAData);
			float ax[258], ay[258], az[258], gx[258], gy[258], gz[258];    // variables to hold latest accel/gyro data values
			uint16_t ts[258];
			float mx, my, mz;                                                    // variables to hold latest mag data values
			for (uint16_t i = 0; i < packets; i++) {
				uint16_t index = i * 16; // Packet size 16 bytes
				// combine into 16 bit values
				int16_t rawMeas[7];
				//TODO: check header that it contains data
				rawMeas[0] = (((int16_t)rawData[index + 1]) << 8) | rawData[index + 2]; // ax
				rawMeas[1] = (((int16_t)rawData[index + 3]) << 8) | rawData[index + 4]; // ay
				rawMeas[2] = (((int16_t)rawData[index + 5]) << 8) | rawData[index + 6]; // az
				rawMeas[3] = (((int16_t)rawData[index + 7]) << 8) | rawData[index + 8]; // gx
				rawMeas[4] = (((int16_t)rawData[index + 9]) << 8) | rawData[index + 10]; // gy
				rawMeas[5] = (((int16_t)rawData[index + 11]) << 8) | rawData[index + 12]; // gz
				rawMeas[6] = (((int16_t)rawData[index + 14]) << 8) | rawData[index + 15]; // timestamp
				// transform and convert to float values
    			ax[i] = (float)rawMeas[0] * aRes - accelBias[0];
    			ay[i] = (float)rawMeas[1] * aRes - accelBias[1];
    			az[i] = (float)rawMeas[2] * aRes - accelBias[2];
    			gx[i] = (float)rawMeas[3] * gRes - gyroBias[0];
    			gy[i] = (float)rawMeas[4] * gRes - gyroBias[1];
    			gz[i] = (float)rawMeas[5] * gRes - gyroBias[2];
				ts[i] = rawMeas[6];
			}
			// transform and convert to float values
            mx = ((float)MMC5983MAData[0] - MMC5983MA_offset) * mRes - magBias[0];
            my = ((float)MMC5983MAData[1] - MMC5983MA_offset) * mRes - magBias[1];
            mz = ((float)MMC5983MAData[2] - MMC5983MA_offset) * mRes - magBias[2];
            mx *= magScale[0];
            my *= magScale[1];
            mz *= magScale[2];
			//TODO: swap out fusion?
			for (uint16_t i = 0; i < packets; i++) {
				MadgwickQuaternionUpdate(ax[i], ay[i], az[i], gx[i], gy[i], gz[i], mx, my, mz, (double)ts[i]*32.0/30.0/1000.0);
			}
			//TODO: on significant change set main_data true
			tx_payload.data[2] = batt_pptt / 100;
			esb_write_payload(&tx_payload); //need to actually send imu data (target dongle, self id, battery, quats, accel)
		} else {
    		k_msleep(11); // Wait for start up
			uint8_t ICM42688ID = icm_getChipID(main_imu); // Read CHIP_ID register for ICM42688
			uint8_t MMC5983ID = mmc_getChipID(main_mag); // Read CHIP_ID register for MMC5983MA
			if ((ICM42688ID == 0x47 || ICM42688ID == 0xD8) && MMC5983ID == 0x30) // check if all I2C sensors have acknowledged
			{
				icm_reset(main_imu); // software reset ICM42688 to default registers
				icm_DRStatus(main_imu); // clear reset done int flag
				icm_init(main_imu, Ascale, Gscale, AODR, GODR, aMode, gMode, false); // configure
				mmc_reset(main_mag); // software reset MMC5983MA to default registers
				mmc_SET(main_mag); // "deGauss" magnetometer
				mmc_init(main_mag, MODR, MBW, MSET); // configure
				main_ok = true;
			}
		}
/*
check aux
if aux setup
	read aux fifo
	read aux mag
	fusion
if aux not setup
	setup aux
*/

		if (docked) {
			//TODO: read usb cdc
			//TODO: this is for really basic configuring
			//TODO: resetting target dongle
//		} else {
//			//TODO: dongle can communicate back to the tracker? ie toggle mag or disable/enable tracking to save power
// 			//if (true) { //if connected (does this really need to be checked?)
//				//if (ready) {
//					//ready = false;
//					//esb_flush_tx();
//
//					//tx_payload = ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08);
//					err = esb_write_payload(&tx_payload); //need to actually send imu data (target dongle, self id, battery, quats, accel)
//					if (err) {
//						//LOG_ERR("Payload write failed, err %d", err);
//					}
//					//tx_payload.data[1]++;
//					main_data = false;
//					aux_data = false;
//				//}
//			//} else {
//				//TODO: retry connection
//				//TODO: if no dongle paired, search for dongles and connect to the first one (set target dongle, write to memory)
//			//}
		}

		// Get time elapsed and sleep/yield until next tick
		int64_t time_delta = k_uptime_get() - time_begin;
		if (time_delta > TICKRATE_MS) {
			k_yield();
		} else {
			k_msleep(TICKRATE_MS - time_delta);
		}
	}
}
