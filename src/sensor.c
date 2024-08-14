#include "globals.h"
#include "sys.h"
#include "util.h"
#include "connection.h"

#include <math.h>
#include "fusion.h"
#include "vqf.h"
#include "magneto/magneto1_4.h"

#include "sensor/ICM42688.h"
#include "sensor/MMC5983MA.h"

#include "sensor.h"

const struct i2c_dt_spec main_imu = I2C_DT_SPEC_GET(MAIN_IMU_NODE);
const struct i2c_dt_spec main_mag = I2C_DT_SPEC_GET(MAIN_MAG_NODE);

/*
Sensor:addr,reg,id

IMUs:

Bosch Sensortec
BMI160:68/69,00,D1
*BMI270:68/69,00,24
BMI323:68/69,00,43
TDK InvenSense
*ICM-42688-P:68/69,75,47
*ICM-42688-V:68/69,75,DB
STMicroelectronics
LSM6DS3:6A/6B,0F,69
LSM6DSO:6A/6B,0F,6C
*LSM6DSV:6A/6B,0F,70

Magnetometers:

QST Corporation
QMC5883L:0B,OD,FF
Bosch Sensortec
BMM150:10/11/12/13,40,32
BMM350:14/15/16/17,00,33
STMicroelectronics
IIS2MDC:1E,4F,40
LIS2MDL:1E,4F,40
LIS3MDL:1C/1E,0F,3D
memsic
MMC5603NJ:30,39,10
MMC5633NJL:30,39,10
*MMC5983MA:30,2F,30
*/

float lin_a[3] = {0};							// linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};			// vector to hold quaternion
float last_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};		// vector to hold quaternion

float gOff[3] = {0}; // runtime fusion gyro offset

int64_t last_data_time;

float accelBias[3] = {0}, gyroBias[3] = {0}; // offset biases for the accel and gyro

int magCal;
int last_magCal;
int64_t magCal_time;
double ata[100]; // init cal
double norm_sum;
double sample_count;

float magBAinv[4][3];

float max_gyro_speed_square;

float accel_actual_time;
float gyro_actual_time;
float mag_actual_time;

bool sensor_fusion_init;

typedef struct sensor_fusion {
	void (*init)(float);
	void (*load)(const void *);
	void (*save)(void *);

	void (*update_accel)(float *, float);
	void (*update)(float *, float *, float *, float);

	void (*get_gyro_bias)(float *);
	void (*set_gyro_bias)(float *);

	void (*update_gyro_sanity)(float *, float *);
	int (*get_gyro_sanity)(void);

	void (*get_lin_a)(float *);
	void (*get_quat)(float *);
} sensor_fusion_t;

sensor_fusion_t sensor_fusion_fusion = {
    *fusion_init,
    *fusion_load,
    *fusion_save,

    *fusion_update_accel,
    *fusion_update,

    *fusion_get_gyro_bias,
    *fusion_set_gyro_bias,

    *fusion_update_gyro_sanity,
    *fusion_get_gyro_sanity,

    *fusion_get_lin_a,
    *fusion_get_quat
};

sensor_fusion_t sensor_fusion_vqf = {
    *vqf_init,
    *vqf_load,
    *vqf_save,

    *vqf_update_accel,
    *vqf_update,

    *vqf_get_gyro_bias,
    *vqf_set_gyro_bias,

    *vqf_update_gyro_sanity,
    *vqf_get_gyro_sanity,

    *vqf_get_lin_a,
    *vqf_get_quat
};

//sensor_fusion_t sensor_fusion;
#define sensor_fusion sensor_fusion_fusion

typedef struct sensor_imu {
	int (*init)(struct i2c_dt_spec, float, float, float*, float*); // return update time, return 0 if success, 1 if general error, also how to deal with CLKIN???? some boards might use it
	void (*shutdown)(struct i2c_dt_spec);

	int (*update_odr)(struct i2c_dt_spec, float, float, float*, float*); // return actual update time, return 0 if success, -1 if odr is same, 1 if general error

	uint16_t (*fifo_read)(struct i2c_dt_spec, uint8_t*);
	int (*fifo_process)(uint16_t, uint8_t*, float[3]); // deg/s
	void (*accel_read)(struct i2c_dt_spec, float[3]); // m/s^2
	float (*temp_read)(struct i2c_dt_spec); // deg C

	void (*setup_WOM)(struct i2c_dt_spec);
} sensor_imu_t;

typedef struct sensor_mag {
	int (*init)(struct i2c_dt_spec, float, float*); // return update time, return 0 if success, 1 if general error
	void (*shutdown)(struct i2c_dt_spec);

	int (*update_odr)(struct i2c_dt_spec, float, float*); // return actual update time, return 0 if success, -1 if odr is same, 1 if general error

	void (*mag_read)(struct i2c_dt_spec, float[3]); // any unit
} sensor_mag_t;

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

	if (retained.fusion_data_stored)
	{
		LOG_INF("Fusion data recovered");
	}
}

// TODO: Always store quat(maybe) and just check if the fusion needs converge fast! (initialize)
// TODO: check if it is a vqf or fusion state
void sensor_retained_write(void) // TODO: move to sys?
{
	if (!sensor_fusion_init)
		return;
	(*sensor_fusion.save)(retained.fusion_data);
	retained.fusion_data_stored = true;
	retained_update();
}

void sensor_shutdown(void) // Communicate all imus to shut down
{
	icm_shutdown(main_imu);
	mmc_shutdown(main_mag);
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
	if (sensor_fusion_init)
	{ // clear fusion gyro offset
		float g_off[3] = {0};
		(*sensor_fusion.set_gyro_bias)(g_off);
		sensor_retained_write();
	}
	else
	{
		retained.fusion_data_stored = false; // Invalidate retained fusion data
	}
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

// TODO: get rid of it
void set_LN(void)
{
	tickrate = 6;
}

// TODO: get rid of it
void set_LP(void)
{
	tickrate = 33;
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
	// TODO: move detection to imus
	uint8_t ICM42688ID = icm_getChipID(main_imu);						 // Read CHIP_ID register for ICM42688
	LOG_INF("ICM: %u", ICM42688ID);
	uint8_t MMC5983ID = mmc_getChipID(main_mag);						 // Read CHIP_ID register for MMC5983MA
	LOG_INF("MMC: %u", MMC5983ID);
	if (!((ICM42688ID == 0x47 || ICM42688ID == 0xDB) && (!MAG_ENABLED || MMC5983ID == 0x30))) // check if all I2C sensors have acknowledged
		return;
	LOG_INF("Found main IMUs");
	// TODO: This may change!!
	//i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // i dont wanna wait on icm!!
	icm_shutdown(main_imu);
	// TODO: This may change!!
	//i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Reset MMC now to avoid waiting 10ms later
	mmc_shutdown(main_mag);
	//icm_reset(main_imu);												 // software reset ICM42688 to default registers
	// TODO: Does int flag need to be read at all
	//uint8_t temp;
	//i2c_reg_read_byte_dt(&main_imu, ICM42688_INT_STATUS, &temp); // clear reset done int flag

	icm_init(main_imu, tickrate / 1000.0, 1.0 / 800, &accel_actual_time, &gyro_actual_time); // configure with ~200Hz ODR, ~1000Hz ODR
// 55-66ms delta to wait, get chip ids, and setup icm (50ms spent waiting for accel and gyro to start)
#if MAG_ENABLED
	mmc_init(main_mag, tickrate / 1000.0, &mag_actual_time);								 // configure with ~200Hz ODR
// 0-1ms delta to setup mmc
#endif
	LOG_INF("Initialized main IMUs");
	main_ok = true;

	sys_read(); // In case calibrations haven't loaded yet

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
	(*sensor_fusion.init)(gyro_actual_time);
	if (retained.fusion_data_stored)
	{ // Load state if the data is valid (fusion was initialized before)
		(*sensor_fusion.load)(retained.fusion_data);
		retained.fusion_data_stored = false; // Invalidate retained fusion data
		retained_update();
	}

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
			uint8_t rawData[2080];
			uint16_t packets = icm_fifo_read(main_imu, rawData); // TODO: name this better

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
					break;
				case 1:
					set_LP();
					LOG_INF("Switching main IMUs to low power");
					mmc_update_odr(main_mag, INFINITY, &mag_actual_time); // standby/oneshot
					break;
				};
			}

			if (powerstate == 0)
			{
				float gyro_speed = sqrtf(max_gyro_speed_square);
				float mag_target_time = 1.0 / (4 * gyro_speed); // target mag ODR for ~0.25 deg error
				if (mag_target_time > 0.005) // cap at 0.005, 200hz
					mag_target_time = 0.005;
				int err = mmc_update_odr(main_mag, mag_target_time, &mag_actual_time);
				if (!err)
					LOG_DBG("Switching magnetometer ODR to %.2fHz", 1.0 / mag_actual_time);
			}
#endif

			float a[] = {ax, -az, ay};
			if (packets == 2 && powerstate == 1 && MAG_ENABLED) // why specifically 2 packets? i forgot
			{ // Fuse accelerometer only
				(*sensor_fusion.update_accel)(a, accel_actual_time);
			}
			else
			{ // Fuse all data
				float g[3] = {0};
				float m[] = {my, mz, -mx};
				max_gyro_speed_square = 0;
				for (uint16_t i = 0; i < packets; i++)
				{
					float raw_g[3];
					if (icm_fifo_process(i, rawData, raw_g))
						continue; // skip on error
					// transform and convert to float values
					float gx = raw_g[0] - gyroBias[0]; //gres
					float gy = raw_g[1] - gyroBias[1]; //gres
					float gz = raw_g[2] - gyroBias[2]; //gres
					//float g[] = {gx, -gz, gy};
					g[0] = gx;
					g[1] = -gz;
					g[2] = gy;

					(*sensor_fusion.update)(g, a, m, gyro_actual_time);
#if MAG_ENABLED
					// Get fusion's corrected gyro data (or get gyro bias from fusion) and use it here
					float g_off[3] = {};
					(*sensor_fusion.get_gyro_bias)(g_off);
					for (int i = 0; i < 3; i++)
					{
						g_off[i] = g[i] - g_off[i];
					}

					// Get the highest gyro speed
					float gyro_speed_square = g_off[0] * g_off[0] + g_off[1] * g_off[1] + g_off[2] * g_off[2];
					if (gyro_speed_square > max_gyro_speed_square)
						max_gyro_speed_square = gyro_speed_square;
#endif
				}

				(*sensor_fusion.update_gyro_sanity)(g, m);
				(*sensor_fusion.get_gyro_bias)(gOff);
			}

			float lin_a[3] = {0};
			(*sensor_fusion.get_lin_a)(lin_a);
			(*sensor_fusion.get_quat)(q);

			if ((*sensor_fusion.get_gyro_sanity)() == 0 ? quat_epsilon_coarse(q, last_q) : quat_epsilon_coarse2(q, last_q)) // Probably okay to use the constantly updating last_q
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
