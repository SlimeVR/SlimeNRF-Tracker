#ifndef SLIMENRF_SENSOR
#define SLIMENRF_SENSOR

#include <zephyr/drivers/i2c.h>

const char *sensor_get_sensor_imu_name(void);
const char *sensor_get_sensor_mag_name(void);
const char *sensor_get_sensor_fusion_name(void);

int sensor_init(void);

void sensor_scan_read(void);
void sensor_scan_write(void);
void sensor_scan_clear(void);

void sensor_retained_read(void);
void sensor_retained_write(void);

void sensor_shutdown(void);
void sensor_setup_WOM(void);

void sensor_calibrate_imu(void);
void sensor_calibrate_mag(void);
void sensor_calibrate_6side(void);

int sensor_calibration_validate(void);
void sensor_calibration_clear(void);

bool wait_for_motion(const struct i2c_dt_spec *dev_i2c, bool motion, int samples);

int main_imu_init(void);
void main_imu_thread(void);
void wait_for_threads(void);
void main_imu_suspend(void);
void main_imu_wakeup(void);

void sensor_offsetBias(const struct i2c_dt_spec *dev_i2c, float *dest1, float *dest2);

void sensor_6SideBias(const struct i2c_dt_spec *dev_i2c);

typedef struct sensor_fusion {
	void (*init)(float, float, float); // gyro_time, accel_time, mag_time
	void (*load)(const void *);
	void (*save)(void *);

	void (*update_gyro)(float *, float); // deg/s
	void (*update_accel)(float *, float); // g
	void (*update_mag)(float *, float); // any unit (usually gauss)
	void (*update)(float *, float *, float *, float);

	void (*get_gyro_bias)(float *);
	void (*set_gyro_bias)(float *);

	void (*update_gyro_sanity)(float *, float *);
	int (*get_gyro_sanity)(void);

	void (*get_lin_a)(float *);
	void (*get_quat)(float *);
} sensor_fusion_t;

typedef struct sensor_imu {
	int (*init)(const struct i2c_dt_spec*, float, float, float, float*, float*); // first float is clock_rate, nonzero means use CLKIN, return update time, return 0 if success, -1 if general error
	void (*shutdown)(const struct i2c_dt_spec*);

	int (*update_odr)(const struct i2c_dt_spec*, float, float, float*, float*); // return actual update time, return 0 if success, 1 if odr is same, -1 if general error

	uint16_t (*fifo_read)(const struct i2c_dt_spec*, uint8_t*, uint16_t);
	int (*fifo_process)(uint16_t, uint8_t*, float[3]); // deg/s TODO: is support accel needed?
	void (*accel_read)(const struct i2c_dt_spec*, float[3]); // g
	void (*gyro_read)(const struct i2c_dt_spec*, float[3]); // deg/s
	float (*temp_read)(const struct i2c_dt_spec*); // deg C

	void (*setup_WOM)(const struct i2c_dt_spec*);

	int (*ext_setup)(uint8_t, uint8_t); // setup external magnetometer
	int (*fifo_process_ext)(uint16_t, uint8_t*, float[3], float[3], uint8_t*); // deg/s, g, raw magnetometer data
	void (*ext_read)(const struct i2c_dt_spec*, uint8_t*); // raw data, to be processed in magnetometer driver
	int (*ext_passthrough)(const struct i2c_dt_spec*, bool); // enable/disable passthrough mode, return 0 if success, -1 if error or not available
} sensor_imu_t;

typedef struct sensor_mag {
	int (*init)(const struct i2c_dt_spec*, float, float*); // return update time, return 0 if success, 1 if general error
	void (*shutdown)(const struct i2c_dt_spec*);

	int (*update_odr)(const struct i2c_dt_spec*, float, float*); // return actual update time, return 0 if success, 1 if odr is same, -1 if general error

	void (*mag_oneshot)(const struct i2c_dt_spec*); // trigger oneshot if exists
	void (*mag_read)(const struct i2c_dt_spec*, float[3]); // any unit (usually gauss)
	float (*temp_read)(const struct i2c_dt_spec*, float[3]); // deg C

	void (*mag_process)(uint8_t*, float[3]); // use if magnetometer is present as an auxiliary sensor, from data read by IMU
	uint8_t ext_reg; // register for auxiliary interface to read magnetometer data
} sensor_mag_t;

#endif