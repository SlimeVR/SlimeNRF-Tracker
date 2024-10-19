#ifndef SLIMENRF_SENSOR_NONE
#define SLIMENRF_SENSOR_NONE

#include "../sensor.h"

int imu_none_init(const struct i2c_dt_spec *dev_i2c, float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time);
void imu_none_shutdown(const struct i2c_dt_spec *dev_i2c);

int imu_none_update_odr(const struct i2c_dt_spec *dev_i2c, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time);

uint16_t imu_none_fifo_read(const struct i2c_dt_spec *dev_i2c, uint8_t *data);
int imu_none_fifo_process(uint16_t index, uint8_t *data, float g[3]);
void imu_none_accel_read(const struct i2c_dt_spec *dev_i2c, float a[3]);
void imu_none_gyro_read(const struct i2c_dt_spec *dev_i2c, float g[3]);
float imu_none_temp_read(const struct i2c_dt_spec *dev_i2c);

void imu_none_setup_WOM(const struct i2c_dt_spec *dev_i2c);

int imu_none_ext_setup(uint8_t ext_addr, uint8_t ext_reg);
int imu_none_fifo_process_ext(uint16_t index, uint8_t *data, float g[3], float a[3], uint8_t *raw_m);
void imu_none_ext_read(const struct i2c_dt_spec *dev_i2c, uint8_t *raw_m);
int imu_none_ext_passthrough(const struct i2c_dt_spec *dev_i2c, bool passthrough);

extern const sensor_imu_t sensor_imu_none;

int mag_none_init(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);
void mag_none_shutdown(const struct i2c_dt_spec *dev_i2c);

int mag_none_update_odr(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);

void mag_none_mag_oneshot(const struct i2c_dt_spec *dev_i2c);
void mag_none_mag_read(const struct i2c_dt_spec *dev_i2c, float m[3]);
float mag_none_temp_read(const struct i2c_dt_spec *dev_i2c, float bias[3]);

void mag_none_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_none;

#endif
