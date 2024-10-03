#ifndef SLIMENRF_SENSOR
#define SLIMENRF_SENSOR

#include <zephyr/drivers/i2c.h>

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

void sensor_calibration_validate(void);
void sensor_calibration_clear(void);

bool wait_for_motion(const struct i2c_dt_spec *dev_i2c, bool motion, int samples);

int main_imu_init(void);
void main_imu_thread(void);
void wait_for_threads(void);
void main_imu_suspend(void);
void main_imu_wakeup(void);

void sensor_offsetBias(const struct i2c_dt_spec *dev_i2c, float *dest1, float *dest2);

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

typedef struct sensor_imu {
	int (*init)(const struct i2c_dt_spec*, float, float, float, float*, float*); // first float is clock_rate, nonzero means use CLKIN, return update time, return 0 if success, -1 if general error
	void (*shutdown)(const struct i2c_dt_spec*);

	int (*update_odr)(const struct i2c_dt_spec*, float, float, float*, float*); // return actual update time, return 0 if success, 1 if odr is same, -1 if general error

	uint16_t (*fifo_read)(const struct i2c_dt_spec*, uint8_t*);
	int (*fifo_process)(uint16_t, uint8_t*, float[3]); // deg/s TODO: is support accel needed?
	void (*accel_read)(const struct i2c_dt_spec*, float[3]); // m/s^2
	void (*gyro_read)(const struct i2c_dt_spec*, float[3]); // deg/s
	float (*temp_read)(const struct i2c_dt_spec*); // deg C

	void (*setup_WOM)(const struct i2c_dt_spec*);
} sensor_imu_t;

typedef struct sensor_mag {
	int (*init)(const struct i2c_dt_spec*, float, float*); // return update time, return 0 if success, 1 if general error
	void (*shutdown)(const struct i2c_dt_spec*);

	int (*update_odr)(const struct i2c_dt_spec*, float, float*); // return actual update time, return 0 if success, 1 if odr is same, -1 if general error

	void (*mag_oneshot)(const struct i2c_dt_spec*); // trigger oneshot if exists
	void (*mag_read)(const struct i2c_dt_spec*, float[3]); // any unit
	float (*temp_read)(const struct i2c_dt_spec*); // deg C
} sensor_mag_t;

#endif