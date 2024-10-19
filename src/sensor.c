#include "globals.h"
#include "system.h"
#include "util.h"
#include "connection.h"

#include <math.h>

#include "fusion.h"
#include "vqf.h"
#include "magneto/magneto1_4.h"

#include "sensor/sensors.h"

#include "sensor.h"

#if DT_NODE_EXISTS(DT_NODELABEL(imu))
#define SENSOR_IMU_EXISTS true
#define SENSOR_IMU_NODE DT_NODELABEL(imu)
static struct i2c_dt_spec sensor_imu_dev = I2C_DT_SPEC_GET(SENSOR_IMU_NODE);
#else
#error "IMU node does not exist"
static struct i2c_dt_spec sensor_imu_dev = {0};
#endif
static uint8_t sensor_imu_dev_reg = 0xFF;

#if DT_NODE_EXISTS(DT_NODELABEL(mag))
#define SENSOR_MAG_EXISTS true
#define SENSOR_MAG_NODE DT_NODELABEL(mag)
static struct i2c_dt_spec sensor_mag_dev = I2C_DT_SPEC_GET(SENSOR_MAG_NODE);
#else
#warning "Magnetometer node does not exist"
static struct i2c_dt_spec sensor_mag_dev = {0};
#endif
static uint8_t sensor_mag_dev_reg = 0xFF;

static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion
static float last_q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion

static float q3[4] = {SENSOR_QUATERNION_CORRECTION}; // correction quaternion

static int64_t last_data_time;
static int64_t last_info_time;

static float accelBias[3] = {0}, gyroBias[3] = {0}, magBias[3] = {0}; // offset biases

static int mag_progress;
static int last_mag_progress;
static int64_t mag_progress_time;
static double ata[100]; // init calibration
static double norm_sum;
static double sample_count;

static float magBAinv[4][3];

static float max_gyro_speed_square;
static bool mag_use_oneshot;

static float accel_actual_time;
static float gyro_actual_time;
static float mag_actual_time;

static bool sensor_fusion_init;
static bool sensor_sensor_init;

static bool sensor_sensor_scanning;

static bool main_suspended;

static bool mag_available;
#ifdef MAG_ENABLED
static bool mag_enabled = MAG_ENABLED; // TODO: toggle from server
#else
static bool mag_enabled = false;
#endif

static const sensor_fusion_t *sensor_fusion = &sensor_fusion_fusion; // TODO: change from server

static const sensor_imu_t *sensor_imu = &sensor_imu_none;
static const sensor_mag_t *sensor_mag = &sensor_mag_none;
static bool use_ext_fifo = false;

LOG_MODULE_REGISTER(sensor, LOG_LEVEL_INF);

K_THREAD_DEFINE(main_imu_thread_id, 4096, main_imu_thread, NULL, NULL, NULL, 7, 0, 0);

int sensor_init(void)
{
	while (sensor_sensor_scanning)
		k_usleep(1); // already scanning
	if (sensor_sensor_init)
		return 0; // already initialized
	sensor_sensor_scanning = true;

	sensor_scan_read();

#if SENSOR_IMU_EXISTS
	LOG_INF("Scanning bus for IMU");
	int imu_id = sensor_scan_imu(&sensor_imu_dev, &sensor_imu_dev_reg);
#else
	LOG_ERR("IMU node does not exist");
	int imu_id = -1;
#endif
	if (imu_id >= (int)(sizeof(dev_imu_names) / sizeof(dev_imu_names[0])))
		LOG_WRN("Found unknown device");
	else if (imu_id < 0)
		LOG_ERR("No IMU detected");
	else
		LOG_INF("Found %s", dev_imu_names[imu_id]);
	if (imu_id >= 0)
	{
		if (imu_id >= (int)(sizeof(sensor_imus) / sizeof(sensor_imus[0])) || sensor_imus[imu_id] == NULL || sensor_imus[imu_id] == &sensor_imu_none)
		{
			sensor_imu = &sensor_imu_none;
			sensor_sensor_scanning = false; // done
//			if (sensor_imu_dev.addr < 0xFF) // If for some reason there actually is a valid IMU but we found some unsupported device first
//			{
//				LOG_WRN("IMU not supported");
//				sensor_imu_dev.addr++;
//				sensor_imu_dev_reg = 0xFF;
//				sensor_scan_clear(); // clear the invalid data
//				return sensor_init(); // try again
//			}
			LOG_ERR("IMU not supported");
			set_status(SYS_STATUS_SENSOR_ERROR, true);
			return -1; // an IMU was detected but not supported
		}
		else
		{
			sensor_imu = sensor_imus[imu_id];
		}
	}
	else
	{
		sensor_imu = &sensor_imu_none;
		sensor_sensor_scanning = false; // done
		set_status(SYS_STATUS_SENSOR_ERROR, true);
		return -1; // no IMU detected! something is very wrong
	}

#if SENSOR_MAG_EXISTS
	LOG_INF("Scanning bus for magnetometer");
	int mag_id = sensor_scan_mag(&sensor_mag_dev, &sensor_mag_dev_reg);
	if (mag_id < 0)
	{
		// IMU must support passthrough mode if the magnetometer is connected through the IMU
		int err = (*sensor_imu->ext_passthrough)(&sensor_imu_dev, true);
		if (!err)
		{
			LOG_INF("Scanning bus for magnetometer through IMU passthrough");
			if (sensor_mag_dev.addr > 0x80) // marked as passthrough
			{
				sensor_mag_dev.addr &= 0x7F;
			}
			else
			{
				sensor_mag_dev.addr = 0x00; // reset magnetometer data
				sensor_mag_dev_reg = 0xFF;
			}
			mag_id = sensor_scan_mag(&sensor_mag_dev, &sensor_mag_dev_reg);
			if (mag_id >= 0)
			{
				sensor_mag_dev.addr |= 0x80; // mark as passthrough
				use_ext_fifo = true;
			}
		}
		// (*sensor_imu->ext_passthrough)(&sensor_imu_dev, false);
	}
	else
	{
		use_ext_fifo = false;
	}
#else
	LOG_WRN("Magnetometer node does not exist");
	int mag_id = -1;
#endif
	if (mag_id >= (int)(sizeof(dev_mag_names) / sizeof(dev_mag_names[0])))
		LOG_WRN("Found unknown device");
	else if (mag_id < 0)
		LOG_WRN("No magnetometer detected");
	else
		LOG_INF("Found %s", dev_mag_names[mag_id]);
	if (mag_id >= 0) // if there is no magnetometer we do not care as much
	{
		if (mag_id >= (int)(sizeof(dev_mag_names) / sizeof(dev_mag_names[0])) || sensor_mags[mag_id] == NULL || sensor_mags[mag_id] == &sensor_mag_none)
		{
			sensor_mag = &sensor_mag_none; 
			mag_available = false;
//			if (sensor_imu_dev.addr < 0xFF) // If for some reason there actually is a valid magnetometer but we found some unsupported device first
//			{
//				LOG_WRN("Magnetometer not supported");
//				sensor_mag_dev.addr++;
//				sensor_mag_dev_reg = 0xFF;
//				sensor_scan_clear(); // clear the invalid data
//				return sensor_init(); // try again
//			}
			LOG_ERR("Magnetometer not supported");
		}
		else
		{
			sensor_mag = sensor_mags[mag_id];
			mag_available = true;
		}
	}
	else
	{
		sensor_mag = &sensor_mag_none; 
		mag_available = false; // marked as not available
	}
	if (use_ext_fifo)
	{
		int err = mag_ext_setup(sensor_imu, sensor_mag, sensor_mag_dev.addr);
		if (err)
		{
			LOG_ERR("Magnetometer not supported by external interface");
			sensor_mag = &sensor_mag_none;
			mag_available = false;
		}
		else
		{
			sensor_mag = &sensor_mag_ext;
			mag_available = true;
		}
		
	}

	sensor_scan_write();
	connection_update_sensor_ids(imu_id, mag_id);

	sensor_sensor_init = true; // successfully initialized
	sensor_sensor_scanning = false; // done
	set_status(SYS_STATUS_SENSOR_ERROR, false); // clear error
	return 0;
}

void sensor_scan_read(void) // TODO: move some of this to sys?
{
	if (retained.imu_addr != 0)
	{
		sensor_imu_dev.addr = retained.imu_addr;
		sensor_imu_dev_reg = retained.imu_reg;
	}
	if (retained.mag_addr != 0)
	{
		sensor_mag_dev.addr = retained.mag_addr;
		sensor_mag_dev_reg = retained.mag_reg;
	}
	LOG_INF("IMU address: 0x%02X, register: 0x%02X", sensor_imu_dev.addr, sensor_imu_dev_reg);
	LOG_INF("Magnetometer address: 0x%02X, register: 0x%02X", sensor_mag_dev.addr, sensor_mag_dev_reg);
}

void sensor_scan_write(void) // TODO: move some of this to sys?
{
	retained.imu_addr = sensor_imu_dev.addr;
	retained.mag_addr = sensor_mag_dev.addr;
	retained.imu_reg = sensor_imu_dev_reg;
	retained.mag_reg = sensor_mag_dev_reg;
	retained_update();
}

void sensor_scan_clear(void) // TODO: move some of this to sys?
{
	retained.imu_addr = 0x00;
	retained.mag_addr = 0x00;
	retained.imu_reg = 0xFF;
	retained.mag_reg = 0xFF;
	retained_update();
}

void sensor_retained_read(void) // TODO: move some of this to sys?
{
	// Read calibration from retained
	memcpy(accelBias, retained.accelBias, sizeof(accelBias));
	memcpy(gyroBias, retained.gyroBias, sizeof(gyroBias));
	memcpy(magBias, retained.magBias, sizeof(magBias));
	memcpy(magBAinv, retained.magBAinv, sizeof(magBAinv));
	LOG_INF("Accelerometer bias: %.5f %.5f %.5f", accelBias[0], accelBias[1], accelBias[2]);
	LOG_INF("Gyroscope bias: %.5f %.5f %.5f", gyroBias[0], gyroBias[1], gyroBias[2]);
	LOG_INF("Magnetometer bridge offset: %.5f %.5f %.5f", magBias[0], magBias[1], magBias[2]);
	LOG_INF("Magnetometer matrix:");
	for (int i = 0; i < 3; i++)
		LOG_INF("%.5f %.5f %.5f %.5f", magBAinv[0][i], magBAinv[1][i], magBAinv[2][i], magBAinv[3][i]);
	sensor_calibration_validate();

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
	memcpy(retained.magBias, magBias, sizeof(magBias));
	(*sensor_fusion->save)(retained.fusion_data);
	retained.fusion_data_stored = true;
	retained_update();
}

void sensor_shutdown(void) // Communicate all imus to shut down
{
	int err = sensor_init(); // try initialization if possible
	if (!err)
		(*sensor_imu->shutdown)(&sensor_imu_dev);
	else
		LOG_ERR("Failed to shutdown sensors");
	if (mag_available)
		(*sensor_mag->shutdown)(&sensor_mag_dev);
}

void sensor_setup_WOM(void)
{
	int err = sensor_init(); // try initialization if possible
	if (!err)
		(*sensor_imu->setup_WOM)(&sensor_imu_dev);
	else
		LOG_ERR("Failed to setup wake on motion");
}

void sensor_calibrate_imu(void)
{
	LOG_INF("Calibrating main accelerometer and gyroscope zero rate offset");
	LOG_INF("Rest the device on a stable surface");

	set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_SENSOR);
	if (!wait_for_motion(&sensor_imu_dev, false, 6)) // Wait for accelerometer to settle, timeout 3s
		return; // Timeout, calibration failed

	set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_SENSOR);
	k_msleep(500); // Delay before beginning acquisition

	LOG_INF("Reading data");
	sensor_offsetBias(&sensor_imu_dev, accelBias, gyroBias); // This takes about 3s
	sys_write(MAIN_ACCEL_BIAS_ID, &retained.accelBias, accelBias, sizeof(accelBias));
	sys_write(MAIN_GYRO_BIAS_ID, &retained.gyroBias, gyroBias, sizeof(gyroBias));
	LOG_INF("Accelerometer bias: %.5f %.5f %.5f", accelBias[0], accelBias[1], accelBias[2]);
	LOG_INF("Gyroscope bias: %.5f %.5f %.5f", gyroBias[0], gyroBias[1], gyroBias[2]);
	sensor_calibration_validate();

	LOG_INF("Finished calibration");
	if (sensor_fusion_init)
	{ // clear fusion gyro offset
		float g_off[3] = {0};
		(*sensor_fusion->set_gyro_bias)(g_off);
		sensor_retained_write();
	}
	else
	{ // TODO: always clearing the fusion?
		retained.fusion_data_stored = false; // Invalidate retained fusion data
	}
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
}

void sensor_calibrate_mag(void)
{
	LOG_INF("Calibrating magnetometer hard/soft iron offset");
	magneto_current_calibration(magBAinv, ata, norm_sum, sample_count); // 25ms
	sys_write(MAIN_MAG_BIAS_ID, &retained.magBAinv, magBAinv, sizeof(magBAinv));
	LOG_INF("Magnetometer matrix:");
	for (int i = 0; i < 3; i++)
		LOG_INF("%.5f %.5f %.5f %.5f", magBAinv[0][i], magBAinv[1][i], magBAinv[2][i], magBAinv[3][i]);
	LOG_INF("Finished calibration");
	//mag_progress |= 1 << 7;
	mag_progress = 0;
	// clear data
	//memset(ata[0], 0, sizeof(ata)); // TODO: does this work??
	for (int i = 0; i < 100; i++)
		ata[i] = 0.0;
	norm_sum = 0.0;
	sample_count = 0.0;
}

void sensor_calibration_validate(void)
{
	float zero[3] = {0};
	if (!v_epsilon(accelBias, zero, 1.0) || !v_epsilon(gyroBias, zero, 100.0)) // TODO: this is using v_epsilon to compare zero. Check accel is <1G and gyro <100dps
	{
		sensor_calibration_clear();
		LOG_WRN("Invalidated calibration");
		LOG_WRN("The IMU may be damaged or calibration was not completed properly");
	}
}

void sensor_calibration_clear(void)
{
	memset(accelBias, 0, sizeof(accelBias));
	memset(gyroBias, 0, sizeof(gyroBias));
	sys_write(MAIN_ACCEL_BIAS_ID, &retained.accelBias, accelBias, sizeof(accelBias));
	sys_write(MAIN_GYRO_BIAS_ID, &retained.gyroBias, gyroBias, sizeof(gyroBias));

	if (sensor_fusion_init)
	{ // clear fusion gyro offset
		float g_off[3] = {0};
		(*sensor_fusion->set_gyro_bias)(g_off);
		sensor_retained_write();
	}
	else
	{ // TODO: always clearing the fusion?
		retained.fusion_data_stored = false; // Invalidate retained fusion data
	}
}

bool wait_for_motion(const struct i2c_dt_spec *dev_i2c, bool motion, int samples)
{
	uint8_t counts = 0;
	float a[3], last_a[3];
	(*sensor_imu->accel_read)(dev_i2c, last_a);
	for (int i = 0; i < samples + counts; i++)
	{
		LOG_INF("Accelerometer: %.5f %.5f %.5f", a[0], a[1], a[2]);
		k_msleep(500);
		(*sensor_imu->accel_read)(dev_i2c, a);
		if (v_epsilon(a, last_a, 0.1) != motion)
		{
			LOG_INF("No motion detected");
			counts++;
			if (counts == 2)
			{
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
	return false;
}

int sensor_update_time_ms = 6;

// TODO: get rid of it.. ?
static void set_update_time_ms(int time_ms)
{
	sensor_update_time_ms = time_ms; // TODO: terrible naming
}

int main_imu_init(void)
{
	int err;
	// TODO: on any errors set main_ok false and skip (make functions return nonzero)
	err = sensor_init(); // IMUs discovery
	if (err)
		return err;
	(*sensor_imu->shutdown)(&sensor_imu_dev);
	if (mag_available)
		(*sensor_mag->shutdown)(&sensor_mag_dev);

	float clock_actual_rate;
	set_sensor_clock(true, 32768, &clock_actual_rate); // enable the clock source for IMU if present
	LOG_INF("Sensor clock rate: %.2fHz", clock_actual_rate);

	k_usleep(250); // wait for sensor register reset
	float accel_initial_time = sensor_update_time_ms / 1000.0; // configure with ~200Hz ODR
#if CONFIG_FPU
	float gyro_initial_time = 1.0 / 800; // configure with ~1000Hz ODR
#else
	float gyro_initial_time = 1.0 / 800; // TODO: check if this is still okay for vqf
#endif
	err = (*sensor_imu->init)(&sensor_imu_dev, clock_actual_rate, accel_initial_time, gyro_initial_time, &accel_actual_time, &gyro_actual_time);
	LOG_INF("Accelerometer initial rate: %.2fHz", 1.0 / accel_actual_time);
	LOG_INF("Gyrometer initial rate: %.2fHz", 1.0 / gyro_actual_time);
	if (err < 0)
		return err;
// 55-66ms to wait, get chip ids, and setup icm (50ms spent waiting for accel and gyro to start)
	if (mag_available && mag_enabled)
	{
		err = (*sensor_mag->init)(&sensor_mag_dev, sensor_update_time_ms / 1000.0, &mag_actual_time); // configure with ~200Hz ODR
		LOG_INF("Magnetometer initial rate: %.2fHz", 1.0 / mag_actual_time);
		if (err < 0)
			return err;
// 0-1ms to setup mmc
	}
	LOG_INF("Initialized sensors");

	// Setup fusion
	sensor_retained_read();
	(*sensor_fusion->init)(gyro_actual_time);
	if (retained.fusion_data_stored)
	{ // Load state if the data is valid (fusion was initialized before)
		(*sensor_fusion->load)(retained.fusion_data);
		retained.fusion_data_stored = false; // Invalidate retained fusion data
		retained_update();
	}

	// Calibrate IMU
	if (accelBias[0] == 0 && accelBias[1] == 0 && accelBias[2] == 0 && gyroBias[0] == 0 && gyroBias[1] == 0 && gyroBias[2] == 0) // TODO: better way to check?
		sensor_calibrate_imu();

	LOG_INF("Initialized fusion");
	sensor_fusion_init = true;
	return 0;
}

enum sensor_sensor_mode {
//	SENSOR_SENSOR_MODE_OFF,
	SENSOR_SENSOR_MODE_LOW_NOISE,
	SENSOR_SENSOR_MODE_LOW_POWER
};

static enum sensor_sensor_mode sensor_mode = SENSOR_SENSOR_MODE_LOW_NOISE;
static enum sensor_sensor_mode last_sensor_mode = SENSOR_SENSOR_MODE_LOW_NOISE;

static bool main_running = false;
static bool main_ok = false;
static bool send_info = false;

void main_imu_thread(void)
{
	main_running = true;
	int err = main_imu_init(); // Initialize IMUs and Fusion
	// TODO: handle imu init error, maybe restart device or flash led
	if (err)
		set_status(SYS_STATUS_SENSOR_ERROR, true); // TODO: only handles general init error
	else
		main_ok = true;
	while (1)
	{
		int64_t time_begin = k_uptime_get();
		if (main_ok)
		{
			// Trigger reconfig on sensor mode change
			bool reconfig = last_sensor_mode != sensor_mode;
			last_sensor_mode = sensor_mode;

			// Reading IMUs will take between 2.5ms (~7 samples, low noise) - 7ms (~33 samples, low power)
			// Magneto sample will take ~400us
			// Fusing data will take between 100us (~7 samples, low noise) - 500us (~33 samples, low power)
			// TODO: on any errors set main_ok false and skip (make functions return nonzero)

			// At high speed, use oneshot mode to have synced magnetometer data
			// Call before FIFO and get the data after
			if (mag_available && mag_enabled && mag_use_oneshot)
				(*sensor_mag->mag_oneshot)(&sensor_mag_dev);

			// Read IMU temperature
			float temp = (*sensor_imu->temp_read)(&sensor_imu_dev); // TODO: use as calibration data
			connection_update_sensor_temp(temp);

			// Read gyroscope (FIFO)
			uint8_t rawData[2080];
			uint16_t packets = (*sensor_imu->fifo_read)(&sensor_imu_dev, rawData); // TODO: name this better?
			LOG_DBG("IMU packet count: %u", packets);

			// Read accelerometer
			float raw_a[3];
			(*sensor_imu->accel_read)(&sensor_imu_dev, raw_a);
			float ax = raw_a[0] - accelBias[0];
			float ay = raw_a[1] - accelBias[1];
			float az = raw_a[2] - accelBias[2];

			// Read magnetometer and process magneto
			float mx = 0, my = 0, mz = 0;
			if (mag_available && mag_enabled && sensor_mode == SENSOR_SENSOR_MODE_LOW_NOISE)
			{
				float m[3];
				(*sensor_mag->mag_read)(&sensor_mag_dev, m);
				for (int i = 0; i < 3; i++)
					m[i] -= magBias[i];
				magneto_sample(m[0], m[1], m[2], ata, &norm_sum, &sample_count); // 400us
				apply_BAinv(m, magBAinv);
				mx = m[0];
				my = m[1];
				mz = m[2];
				int new_mag_progress = mag_progress;
				new_mag_progress |= (-1.2 < ax && ax < -0.8 ? 1 << 0 : 0) | (1.2 > ax && ax > 0.8 ? 1 << 1 : 0) | // dumb check if all accel axes were reached for calibration, assume the user is intentionally doing this
					(-1.2 < ay && ay < -0.8 ? 1 << 2 : 0) | (1.2 > ay && ay > 0.8 ? 1 << 3 : 0) |
					(-1.2 < az && az < -0.8 ? 1 << 4 : 0) | (1.2 > az && az > 0.8 ? 1 << 5 : 0);
				if (new_mag_progress > mag_progress && new_mag_progress == last_mag_progress)
				{
					if (k_uptime_get() > mag_progress_time)
					{
						mag_progress = new_mag_progress;
						//LOG_INF("Magnetometer calibration progress: %d", new_mag_progress);
						LOG_INF("Magnetometer calibration progress: %s %s %s %s %s %s" , (new_mag_progress & 0x01) ? "X-" : "--", (new_mag_progress & 0x02) ? "X+" : "--", (new_mag_progress & 0x04) ? "Y-" : "--", (new_mag_progress & 0x08) ? "Y+" : "--", (new_mag_progress & 0x10) ? "Z-" : "--", (new_mag_progress & 0x20) ? "Z+" : "--");
						set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
					}
				}
				else
				{
					mag_progress_time = k_uptime_get() + 1000;
					last_mag_progress = new_mag_progress;
				}
				if (mag_progress == 0b111111)
					set_led(SYS_LED_PATTERN_FLASH, SYS_LED_PRIORITY_SENSOR); // Magnetometer calibration is ready to apply
			}

			if (mag_available && mag_enabled && reconfig) // TODO: get rid of reconfig?
			{
				switch (sensor_mode)
				{
				case SENSOR_SENSOR_MODE_LOW_NOISE:
					set_update_time_ms(6);
					LOG_INF("Switching sensors to low noise");
					break;
				case SENSOR_SENSOR_MODE_LOW_POWER:
					set_update_time_ms(33);
					LOG_INF("Switching sensors to low power");
					(*sensor_mag->update_odr)(&sensor_mag_dev, INFINITY, &mag_actual_time); // standby/oneshot
					break;
				};
			}

			float a[] = {SENSOR_ACCELEROMETER_AXES_ALIGNMENT};
			if (mag_available && mag_enabled && packets == 2 && sensor_mode == SENSOR_SENSOR_MODE_LOW_POWER) // why specifically 2 packets? i forgot
			{ // Fuse accelerometer only
				(*sensor_fusion->update_accel)(a, accel_actual_time);
			}
			else
			{ // Fuse all data
				float g[3] = {0};
				float m[] = {SENSOR_MAGNETOMETER_AXES_ALIGNMENT};
				max_gyro_speed_square = 0;
				for (uint16_t i = 0; i < packets; i++) // TODO: fifo_process_ext is available, need to implement it
				{
					float raw_g[3];
					if ((*sensor_imu->fifo_process)(i, rawData, raw_g))
						continue; // skip on error
					// transform and convert to float values
					float gx = raw_g[0] - gyroBias[0]; //gres
					float gy = raw_g[1] - gyroBias[1]; //gres
					float gz = raw_g[2] - gyroBias[2]; //gres
					float g_aligned[] = {SENSOR_GYROSCOPE_AXES_ALIGNMENT};
					memcpy(g, g_aligned, sizeof(g));

					// Process fusion
					(*sensor_fusion->update)(g, a, m, gyro_actual_time);

					if (mag_available && mag_enabled)
					{
						// Get fusion's corrected gyro data (or get gyro bias from fusion) and use it here
						float g_off[3] = {};
						(*sensor_fusion->get_gyro_bias)(g_off);
						for (int i = 0; i < 3; i++)
							g_off[i] = g[i] - g_off[i];

						// Get the highest gyro speed
						float gyro_speed_square = g_off[0] * g_off[0] + g_off[1] * g_off[1] + g_off[2] * g_off[2];
						if (gyro_speed_square > max_gyro_speed_square)
							max_gyro_speed_square = gyro_speed_square;
					}
				}

				// Update fusion gyro sanity?
				(*sensor_fusion->update_gyro_sanity)(g, m);
			}

			// Get updated linear acceleration and quaternion from fusion
			float lin_a[3] = {0};
			(*sensor_fusion->get_lin_a)(lin_a);
			(*sensor_fusion->get_quat)(q);
			q_normalize(q, q); // safe to use self as output

			// Check the IMU gyroscope
			if ((*sensor_fusion->get_gyro_sanity)() == 0 ? q_epsilon(q, last_q, 0.005) : q_epsilon(q, last_q, 0.05)) // Probably okay to use the constantly updating last_q
			{
				int64_t imu_timeout = CLAMP(last_data_time, 1 * 1000, 15 * 1000); // Ramp timeout from last_data_time
				if (k_uptime_get() - last_data_time > imu_timeout) // No motion in last 1s - 10s
				{
					last_data_time = INT64_MAX; // only try to suspend once
					LOG_INF("No motion from sensors in %llds", imu_timeout/1000);
					sys_request_WOM(); // TODO: this will suspend the thread, will the system still shut down properly? Otherwise this thread should queue shutdown and suspend itself
//					main_imu_suspend(); // TODO: auto suspend, the device should configure WOM ASAP but it does not
				}
				else if (sensor_mode == SENSOR_SENSOR_MODE_LOW_NOISE && k_uptime_get() - last_data_time > 500) // No motion in last 500ms
				{
					LOG_INF("No motion from sensors in 500ms");
					sensor_mode = SENSOR_SENSOR_MODE_LOW_POWER;
				}
			}
			else
			{
				last_data_time = k_uptime_get();
				sensor_mode = SENSOR_SENSOR_MODE_LOW_NOISE;
			}

			// Update magnetometer mode
			if (mag_available && mag_enabled && sensor_mode == SENSOR_SENSOR_MODE_LOW_NOISE)
			{
				float gyro_speed = sqrtf(max_gyro_speed_square);
				float mag_target_time = 1.0 / (4 * gyro_speed); // target mag ODR for ~0.25 deg error
				if (mag_target_time < 0.005) // cap at 0.005 (200hz), above this the sensor will use oneshot mode instead
				{
					mag_target_time = 0.005;
					int err = (*sensor_mag->update_odr)(&sensor_mag_dev, INFINITY, &mag_actual_time);
					if (!err)
						LOG_DBG("Switching magnetometer to oneshot");
					mag_use_oneshot = true;
				}
				if (mag_target_time >= 0.005 || mag_actual_time != INFINITY) // under 200Hz or magnetometer did not have a oneshot mode
				{
					int err = (*sensor_mag->update_odr)(&sensor_mag_dev, mag_target_time, &mag_actual_time);
					if (!err)
						LOG_DBG("Switching magnetometer ODR to %.2fHz", 1.0 / mag_actual_time);
					mag_use_oneshot = false;
				}
			}

			// Check if last status is outdated
			if (!send_info && (k_uptime_get() - last_info_time > 100))
			{
				send_info = true;
				last_info_time = k_uptime_get();
			}

			// Send packet with new orientation
			if (!q_epsilon(q, last_q, 0.001))
			{
				bool send_precise_quat = q_epsilon(q, last_q, 0.005);
				memcpy(last_q, q, sizeof(q));
				float q_offset[4];
				q_multiply(q, q3, q_offset);
				connection_update_sensor_data(q_offset, lin_a);
				if (send_info && !send_precise_quat) // prioritize quat precision
				{
					connection_write_packet_2();
					send_info = false;
				}
				else
				{
					connection_write_packet_1();
				}
			}
			else if (send_info)
			{
				connection_write_packet_0();
				send_info = false;
			}

			// Handle magnetometer calibration or bridge offset calibration
			if (mag_available && mag_enabled && last_sensor_mode == SENSOR_SENSOR_MODE_LOW_POWER && sensor_mode == SENSOR_SENSOR_MODE_LOW_POWER)
			{
				if (mag_progress == 0b111111) // Save magnetometer calibration while idling
				{
					sensor_calibrate_mag();
					set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
				}
				else // only enough time to do one of the two
				{
					(*sensor_mag->temp_read)(&sensor_mag_dev, magBias); // for some applicable magnetometer, calibrates bridge offsets
					sensor_retained_write();
				}
			}
		}
		main_running = false;
//		k_sleep(K_FOREVER);
		int64_t time_delta = k_uptime_get() - time_begin;
//		led_clock_offset += time_delta;
		if (time_delta > sensor_update_time_ms)
			k_yield();
		else
			k_msleep(sensor_update_time_ms - time_delta);
		main_running = true;
	}
}

void wait_for_threads(void)
{
	while (main_running)
		k_usleep(1); // bane of my existence. don't use k_yield()!!!!!!
}

void main_imu_suspend(void)
{
	main_suspended = true;
	while (sensor_sensor_scanning)
		k_usleep(1); // try not to interrupt scanning
	while (main_running) // TODO: change to detect if i2c is busy
		k_usleep(1); // try not to interrupt anything actually
	k_thread_suspend(main_imu_thread_id);
	main_running = false; // TODO: redundant
	LOG_INF("Suspended sensor thread");
}

void main_imu_wakeup(void)
{
	if (!main_suspended) // don't wake up if pending suspension
		k_wakeup(main_imu_thread_id);
}

// TODO: move to a calibration file
// TODO: setup 6 sided calibration (bias and scale, and maybe gyro ZRO?), setup temp calibration (particulary for gyro ZRO)
void sensor_offsetBias(const struct i2c_dt_spec *dev_i2c, float *dest1, float *dest2)
{
	float rawData[3];
	for (int i = 0; i < 500; i++)
	{
		(*sensor_imu->accel_read)(dev_i2c, &rawData[0]);
		dest1[0] += rawData[0];
		dest1[1] += rawData[1];
		dest1[2] += rawData[2];
		(*sensor_imu->gyro_read)(dev_i2c, &rawData[0]);
		dest2[0] += rawData[0];
		dest2[1] += rawData[1];
		dest2[2] += rawData[2];
		k_msleep(5);
	}

	dest1[0] /= 500.0f;
	dest1[1] /= 500.0f;
	dest1[2] /= 500.0f;
	dest2[0] /= 500.0f;
	dest2[1] /= 500.0f;
	dest2[2] /= 500.0f;
// need better accel calibration
	if(dest1[0] > 0.9f) {dest1[0] -= 1.0f;} // Remove gravity from the x-axis accelerometer bias calculation
	if(dest1[0] < -0.9f) {dest1[0] += 1.0f;} // Remove gravity from the x-axis accelerometer bias calculation
	if(dest1[1] > 0.9f) {dest1[1] -= 1.0f;} // Remove gravity from the y-axis accelerometer bias calculation
	if(dest1[1] < -0.9f) {dest1[1] += 1.0f;} // Remove gravity from the y-axis accelerometer bias calculation
	if(dest1[2] > 0.9f) {dest1[2] -= 1.0f;} // Remove gravity from the z-axis accelerometer bias calculation
	if(dest1[2] < -0.9f) {dest1[2] += 1.0f;} // Remove gravity from the z-axis accelerometer bias calculation
}
