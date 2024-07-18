#include "globals.h"
#include "sys.h"
#include "util.h"
#include "connection.h"

#include "fusion.h"
#include "magneto/magneto1_4.h"

#include "sensor/ICM42688.h"
#include "sensor/MMC5983MA.h"

#include "sensor.h"

const struct i2c_dt_spec main_imu = I2C_DT_SPEC_GET(MAIN_IMU_NODE);
const struct i2c_dt_spec main_mag = I2C_DT_SPEC_GET(MAIN_MAG_NODE);

float lin_a[3] = {0};							// linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};			// vector to hold quaternion
float last_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};		// vector to hold quaternion

float gOff[3] = {0}; // runtime fusion gyro offset

int64_t last_data_time;

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
uint8_t Ascale, Gscale, AODR, GODR, aMode, gMode; // also change gyro range in fusion!
#define INTEGRATION_TIME 0.001
#define INTEGRATION_TIME_LP 0.005

float accelBias[3], gyroBias[3]; // offset biases for the accel and gyro

// MMC5983MA definitions

// TODO: move to sensor
/* Specify sensor parameters (continuous mode sample rate is dependent on bandwidth)
 * choices are: MODR_ONESHOT, MODR_1Hz, MODR_10Hz, MODR_20Hz, MODR_50 Hz, MODR_100Hz, MODR_200Hz (BW = 0x01), MODR_1000Hz (BW = 0x03)
 * Bandwidth choices are: MBW_100Hz, MBW_200Hz, MBW_400Hz, MBW_800Hz
 * Set/Reset choices are: MSET_1, MSET_25, MSET_75, MSET_100, MSET_250, MSET_500, MSET_1000, MSET_2000, so MSET_100 set/reset occurs every 100th measurement, etc.
 */
uint8_t MODR, MBW, MSET;

int magCal;
int last_magCal;
int64_t magCal_time;
double ata[100]; // init cal
double norm_sum;
double sample_count;

float magBAinv[4][3];

int mag_level;
int last_mag_level;

bool sensor_fusion_init;

LOG_MODULE_REGISTER(sensor, LOG_LEVEL_INF);

K_THREAD_DEFINE(main_imu_thread_id, 4096, main_imu_thread, NULL, NULL, NULL, 7, 0, 0);

void sensor_retained_read(void) // TODO: move some of this to sys?
{
	// Read calibration from retained
	memcpy(accelBias, retained.accelBias, sizeof(accelBias));
	memcpy(gyroBias, retained.gyroBias, sizeof(gyroBias));
	memcpy(magBAinv, retained.magBAinv, sizeof(magBAinv));
	LOG_INF("Main accelerometer bias: %.5f %.5f %.5f", accelBias[0], accelBias[1], accelBias[2]);
	LOG_INF("Main gyroscope bias: %.5f %.5f %.5f", gyroBias[0], gyroBias[1], gyroBias[2]);
	LOG_INF("Main magnetometer matrix:");
	for (int i = 0; i < 3; i++)
		LOG_INF("%.5f %.5f %.5f %.5f", magBAinv[0][i], magBAinv[1][i], magBAinv[2][i], magBAinv[3][i]);

	// Recover quats if present
	if (retained.stored_quats)
	{
		for (uint8_t i = 0; i < 4; i++)
			q[i] = retained.q[i];
		LOG_INF("Recovered quaternions\nMain: %.2f %.2f %.2f %.2f", q[0], q[1], q[2], q[3]);
		retained.stored_quats = false; // Invalidate the retained quaternions
		retained_update();
	}

	for (uint8_t i = 0; i < 3; i++)
		gOff[i] = retained.gOff[i];
	LOG_INF("Recovered fusion gyroscope offset\nMain: %.2f %.2f %.2f", gOff[0], gOff[1], gOff[2]);
}

void sensor_retained_write_quat(void) // TODO: move some of this to sys?
{
	if (!sensor_fusion_init)
		return;
	// Store the last quats
	for (uint8_t i = 0; i < 4; i++)
		retained.q[i] = q[i];
	retained.stored_quats = true;
	retained_update();
}

void sensor_retained_write_gOff(void) // TODO: move some of this to sys?
{
	if (!sensor_fusion_init)
		return;
	// Store fusion gyro offset
	for (uint8_t i = 0; i < 3; i++)
		retained.gOff[i] = gOff[i];
	retained_update();
}

void sensor_shutdown(void) // Communicate all imus to shut down
{
	i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
	i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
};

void sensor_setup_WOM(void)
{
	icm_setup_WOM(main_imu);
}

void sensor_calibrate_imu(void)
{
	LOG_INF("Calibrating main accelerometer and gyroscope zero rate offset");
	// TODO: Add LED flashies
	LOG_INF("Rest the device on a stable surface");
	if (!wait_for_motion(main_imu, false, 20)) // Wait for accelerometer to settle, timeout 10s
		return; // Timeout, calibration failed
	set_led(SYS_LED_PATTERN_ON); // scuffed led
	k_msleep(500); // Delay before beginning acquisition
	LOG_INF("Reading data");
	icm_offsetBias(main_imu, accelBias, gyroBias); // This takes about 3s
	sys_write(MAIN_ACCEL_BIAS_ID, &retained.accelBias, accelBias, sizeof(accelBias));
	sys_write(MAIN_GYRO_BIAS_ID, &retained.gyroBias, gyroBias, sizeof(gyroBias));
	LOG_INF("%.5f %.5f %.5f", accelBias[0], accelBias[1], accelBias[2]);
	LOG_INF("%.5f %.5f %.5f", gyroBias[0], gyroBias[1], gyroBias[2]);
	LOG_INF("Finished calibration");
	// clear fusion gyro offset
	for (uint8_t i = 0; i < 3; i++)
		gOff[i] = 0;
	sensor_retained_write_gOff();
	set_led(SYS_LED_PATTERN_OFF); // scuffed led
}

void sensor_calibrate_mag(void)
{
	LOG_INF("Calibrating magnetometer hard/soft iron offset");
	magneto_current_calibration(magBAinv, ata, norm_sum, sample_count); // 25ms
	sys_write(MAIN_MAG_BIAS_ID, &retained.magBAinv, magBAinv, sizeof(magBAinv));
	for (int i = 0; i < 3; i++)
		LOG_INF("%.5f %.5f %.5f %.5f", magBAinv[0][i], magBAinv[1][i], magBAinv[2][i], magBAinv[3][i]);
	LOG_INF("Finished calibration");
	//magCal |= 1 << 7;
	magCal = 0;
	// clear data
	//memset(ata[0], 0, sizeof(ata)); // does this work??
	for (int i = 0; i < 100; i++)
		ata[i] = 0.0;
	norm_sum = 0.0;
	sample_count = 0.0;
}

void set_LN(void)
{
	tickrate = 6;
	// TODO: This becomes part of the sensor
//	aMode = aMode_LN;
#if MAG_ENABLED
//	gMode = gMode_LN;
//	MBW = MBW_400Hz;
	MODR = MODR_200Hz;
#endif
}

void set_LP(void)
{
	tickrate = 33;
	// TODO: This becomes part of the sensor
//	aMode = aMode_LP;
#if MAG_ENABLED
//	gMode = gMode_SBY;
//	MBW = MBW_800Hz;
	MODR = MODR_ONESHOT;
#endif
}

void reconfigure_imu(const struct i2c_dt_spec imu)
{
	i2c_reg_write_byte_dt(&imu, ICM42688_ACCEL_CONFIG0, Ascale << 5 | AODR); // set accel ODR and FS
	i2c_reg_write_byte_dt(&imu, ICM42688_GYRO_CONFIG0, Gscale << 5 | GODR); // set gyro ODR and FS
	i2c_reg_write_byte_dt(&imu, ICM42688_PWR_MGMT0, gMode << 2 | aMode); // set accel and gyro modes
}

void reconfigure_mag(const struct i2c_dt_spec mag)
{
	//i2c_reg_write_byte_dt(&mag, MMC5983MA_CONTROL_1, MBW); // set mag bandwidth
	i2c_reg_write_byte_dt(&mag, MMC5983MA_CONTROL_2, 0x80 | (MSET << 4) | 0x08 | MODR); // set mag ODR
}

bool wait_for_motion(const struct i2c_dt_spec imu, bool motion, int samples)
{
	uint8_t counts = 0;
	float a[3], last_a[3];
	icm_accel_read(imu, last_a);
	set_led(SYS_LED_PATTERN_LONG);
	for (int i = 0; i < samples + counts; i++)
	{
		LOG_INF("Accelerometer: %.5f %.5f %.5f", a[0], a[1], a[2]);
		k_msleep(500);
		icm_accel_read(imu, a);
		if (vec_epsilon(a, last_a) != motion)
		{
			LOG_INF("No motion detected");
			counts++;
			if (counts == 2)
			{
				set_led(SYS_LED_PATTERN_OFF);
				return true;
			}
		}
		else
		{
			counts = 0;
		}
		memcpy(last_a, a, sizeof(a));
	}
	LOG_INF("Motion detected");
	set_led(SYS_LED_PATTERN_OFF);
	return false;
}

void main_imu_init(void)
{
	// TODO: on any errors set main_ok false and skip (make functions return nonzero)
// 5ms delta (???) from entering loop
// skip sleep, surely this wont cause issues :D
/*
	int64_t time_delta = k_uptime_get() - start_time;
	if (time_delta < 11)
		k_msleep(11 - time_delta);
	//k_msleep(11);														 // Wait for start up (1ms for ICM, 10ms for MMC -> 10ms)
*/
	uint8_t ICM42688ID = icm_getChipID(main_imu);						 // Read CHIP_ID register for ICM42688
	LOG_INF("ICM: %u", ICM42688ID);
	uint8_t MMC5983ID = mmc_getChipID(main_mag);						 // Read CHIP_ID register for MMC5983MA
	LOG_INF("MMC: %u", MMC5983ID);
	if (!((ICM42688ID == 0x47 || ICM42688ID == 0xDB) && (!MAG_ENABLED || MMC5983ID == 0x30))) // check if all I2C sensors have acknowledged
		return;
	LOG_INF("Found main IMUs");
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
	LOG_INF("Initialized main IMUs");
	main_ok = true;
	main_running = false;
	k_sleep(K_FOREVER); // Wait for after calibrations have loaded the first time
	main_running = true;
	do
	{
		if (reset_mode == 1) // Reset mode main calibration
		{
			sensor_calibrate_imu();
			reset_mode = 0; // Clear reset mode
		}
	} while (false); // TODO: ????? why is this here
	// Setup fusion
	LOG_INF("Initialize fusion");
	sensor_retained_read();
	fusion_init(q, gOff, 1/INTEGRATION_TIME);
	LOG_INF("Initialized fusion");
	sensor_fusion_init = true;
}

// TODO: make threads more abstract, pass in imus n stuff instead
void main_imu_thread(void)
{
	main_running = true;
	main_imu_init(); // Initialize IMUs and Fusion
	while (1)
	{
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
			LOG_DBG("IMU packet count: %u", count);
			count += 32; // Add a few read buffer packets (4 ms)
			uint16_t packets = count / 8;								 // Packet size 8 bytes
			uint8_t rawData[2080];
			uint16_t stco = 0;
			uint8_t addr = ICM42688_FIFO_DATA;
			i2c_write_dt(&main_imu, &addr, 1); // Start read buffer
			while (count > 0)
			{
				i2c_read_dt(&main_imu, &rawData[stco], count > 248 ? 248 : count); // Read less than 255 at a time (for nRF52832)
				stco += 248;
				count = count > 248 ? count - 248 : 0;
				LOG_DBG("IMU packets left: %u", count);
			}

			float raw_a[3];
			icm_accel_read(main_imu, raw_a);
			float ax = raw_a[0] - accelBias[0];
			float ay = raw_a[1] - accelBias[1];
			float az = raw_a[2] - accelBias[2];

			float mx = 0, my = 0, mz = 0;
#if MAG_ENABLED
			if (last_powerstate == 0)
			{
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
				if (new_magCal > magCal && new_magCal == last_magCal)
				{
					if (k_uptime_get() > magCal_time)
					{
						magCal = new_magCal;
						LOG_INF("Magnetometer calibration progress: %d", new_magCal);
					}
				}
				else
				{
					magCal_time = k_uptime_get() + 1000;
					last_magCal = new_magCal;
				}
				if (magCal == 0b111111)
					set_led(SYS_LED_PATTERN_ON); // TODO: will interfere with things
			}

			if (reconfig)
			{
				switch (powerstate)
				{
				case 0:
					set_LN();
					LOG_INF("Switching main IMUs to low noise");
					last_mag_level = -1;
					break;
				case 1:
					set_LP();
					LOG_INF("Switching main IMUs to low power");
					reconfigure_mag(main_mag);
					break;
				};
				//reconfigure_imu(main_imu); // Reconfigure if needed
				//reconfigure_mag(main_mag); // Reconfigure if needed
			}

			if (last_mag_level != mag_level && powerstate == 0)
			{
				switch (mag_level)
				{
				case 0:
					MODR = MODR_10Hz;
					LOG_DBG("Switching magnetometer ODR to 10Hz");
					break;
				case 1:
					MODR = MODR_20Hz;
					LOG_DBG("Switching magnetometer ODR to 20Hz");
					break;
				case 2:
					MODR = MODR_50Hz;
					LOG_DBG("Switching magnetometer ODR to 50Hz");
					break;
				case 3:
					MODR = MODR_100Hz;
					LOG_DBG("Switching magnetometer ODR to 100Hz");
					break;
				case 4:
					MODR = MODR_200Hz;
					LOG_DBG("Switching magnetometer ODR to 200Hz");
					break;
				};
				reconfigure_mag(main_mag);
			}
			last_mag_level = mag_level;
			mag_level = 0;
#endif

			float a[] = {ax, -az, ay};
			if (packets == 2 && powerstate == 1 && MAG_ENABLED)
			{ // Fuse accelerometer only
			// TODO: FUSION UPDATE, ACCEL ONLY
				fusion_update_accel(a, INTEGRATION_TIME_LP);
			}
			else
			{ // Fuse all data
				float g[3] = {0};
				float m[] = {my, mz, -mx};
				for (uint16_t i = 0; i < packets; i++)
				{
					// TODO: Packet processing on specific sensor!
					uint16_t index = i * 8; // Packet size 8 bytes
					if ((rawData[index] & 0x80) == 0x80)
						continue; // Skip empty packets
					// combine into 16 bit values
					float raw0 = (int16_t)((((int16_t)rawData[index + 1]) << 8) | rawData[index + 2]); // gx
					float raw1 = (int16_t)((((int16_t)rawData[index + 3]) << 8) | rawData[index + 4]); // gy
					float raw2 = (int16_t)((((int16_t)rawData[index + 5]) << 8) | rawData[index + 6]); // gz
					if (raw0 < -32766 || raw1 < -32766 || raw2 < -32766)
						continue; // Skip invalid data
					// transform and convert to float values
					float gx = raw0 * (2000.0f/32768.0f) - gyroBias[0]; //gres
					float gy = raw1 * (2000.0f/32768.0f) - gyroBias[1]; //gres
					float gz = raw2 * (2000.0f/32768.0f) - gyroBias[2]; //gres
					//float g[] = {gx, -gz, gy};
					g[0] = gx;
					g[1] = -gz;
					g[2] = gy;

					fusion_update(g, a, m, INTEGRATION_TIME);
#if MAG_ENABLED
					// Get fusion's corrected gyro data (or get gyro bias from fusion) and use it here
					float g_off[3] = {};
					fusion_get_gyro_bias(g_off);
					for (int i = 0; i < 3; i++)
					{
						g_off[i] = g[i] - g_off[i];
					}

					float gyro_speed_square = g_off[0]*g_off[0] + g_off[1]*g_off[1] + g_off[2]*g_off[2];
					// target mag ODR for ~0.25 deg error
					if (gyro_speed_square > 25*25 && mag_level < 4) // >25dps -> 200hz ODR
						mag_level = 4;
					else if (gyro_speed_square > 12*12 && mag_level < 3) // 12-25dps -> 100hz ODR
						mag_level = 3;
					else if (gyro_speed_square > 5*5 && mag_level < 2) // 5-12dps -> 50hz ODR
						mag_level = 2;
					else if (gyro_speed_square > 2*2 && mag_level < 1) // 2-5dps -> 20hz ODR
						mag_level = 1;
					// <2dps -> 10hz ODR
#endif
				}

				fusion_update_gyro_sanity(g, m);
				fusion_get_gyro_bias(gOff);
			}

			float lin_a[3] = {0};
			fusion_get_lin_a(lin_a);
			fusion_get_quat(q);

			if (fusion_get_gyro_sanity() == 0 ? quat_epsilon_coarse(q, last_q) : quat_epsilon_coarse2(q, last_q)) // Probably okay to use the constantly updating last_q
			{
				int64_t imu_timeout = CLAMP(last_data_time, 1 * 1000, 15 * 1000); // Ramp timeout from last_data_time
				if (k_uptime_get() - last_data_time > imu_timeout) // No motion in last 1s - 10s
				{
					LOG_INF("No motion from main IMUs in %llds", imu_timeout/1000);
					system_off_main = true;
				}
				else if (powerstate == 0 && k_uptime_get() - last_data_time > 500) // No motion in last 500ms
				{
					LOG_INF("No motion from main IMUs in 500ms");
					powerstate = 1;
				}
			}
			else
			{
				last_data_time = k_uptime_get();
				powerstate = 0;
			}

			if (!(quat_epsilon(q, last_q)))
			{
				memcpy(last_q, q, sizeof(q));
				float q_offset[4];
				q_multiply(q, q3, q_offset);
				connection_write_packet_0(q_offset, lin_a);
			}

#if MAG_ENABLED
			// Save magCal while idling
			if (magCal == 0b111111 && last_powerstate == 1) // TODO: i guess this is fine
			{
				k_yield(); // yield to imu thread first
				wait_for_threads(); // make sure not to interrupt anything (8ms)
				sensor_calibrate_mag();
			}
#endif
		}
		main_running = false;
		k_sleep(K_FOREVER);
		main_running = true;
	}
}

void wait_for_threads(void)
{
	if (threads_running || main_running)
		while (main_running)
			k_yield();
}

void main_imu_suspend(void)
{
	k_thread_suspend(main_imu_thread_id);
	main_running = false;
	LOG_INF("Suspended main IMU thread");
}

void main_imu_wakeup(void)
{
	k_wakeup(main_imu_thread_id);
}
