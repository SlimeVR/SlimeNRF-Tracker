#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

#include "sensor_none.h"

LOG_MODULE_REGISTER(sensor_none, LOG_LEVEL_DBG);

int imu_none_init(const struct i2c_dt_spec *dev_i2c, float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	LOG_ERR("imu_none_init, sensor has no IMU");
	return -1;
}

void imu_none_shutdown(const struct i2c_dt_spec *dev_i2c)
{
	LOG_ERR("imu_none_shutdown, sensor has no IMU");
	return;
}

int imu_none_update_odr(const struct i2c_dt_spec *dev_i2c, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	LOG_ERR("imu_none_update_odr, sensor has no IMU");
	return -1;
}

uint16_t imu_none_fifo_read(const struct i2c_dt_spec *dev_i2c, uint8_t *data)
{
	LOG_ERR("imu_none_fifo_read, sensor has no IMU");
	return 0;
}

int imu_none_fifo_process(uint16_t index, uint8_t *data, float g[3])
{
	LOG_ERR("imu_none_fifo_process, sensor has no IMU");
	return -1;
}

void imu_none_accel_read(const struct i2c_dt_spec *dev_i2c, float a[3])
{
	LOG_ERR("imu_none_accel_read, sensor has no IMU");
	return;
}

void imu_none_gyro_read(const struct i2c_dt_spec *dev_i2c, float g[3])
{
	LOG_ERR("imu_none_gyro_read, sensor has no IMU");
	return;
}

float imu_none_temp_read(const struct i2c_dt_spec *dev_i2c)
{
	LOG_ERR("imu_none_temp_read, sensor has no IMU");
	return 0;
}

void imu_none_setup_WOM(const struct i2c_dt_spec *dev_i2c)
{
	LOG_ERR("imu_none_setup_WOM, sensor has no IMU");
	return;
}

const sensor_imu_t sensor_imu_none = {
	*imu_none_init,
	*imu_none_shutdown,

	*imu_none_update_odr,

	*imu_none_fifo_read,
	*imu_none_fifo_process,
	*imu_none_accel_read,
	*imu_none_gyro_read,
	*imu_none_temp_read,

	*imu_none_setup_WOM
};

int mag_none_init(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time)
{
	LOG_ERR("mag_none_init, sensor has no magnetometer");
	return -1;
}

void mag_none_shutdown(const struct i2c_dt_spec *dev_i2c)
{
	LOG_ERR("mag_none_shutdown, sensor has no magnetometer");
	return;
}

int mag_none_update_odr(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time)
{
	LOG_ERR("mag_none_update_odr, sensor has no magnetometer");
	return -1;
}

void mag_none_mag_oneshot(const struct i2c_dt_spec *dev_i2c)
{
	LOG_ERR("mag_none_mag_oneshot, sensor has no magnetometer");
	return;
}

void mag_none_mag_read(const struct i2c_dt_spec *dev_i2c, float m[3])
{
	LOG_ERR("mag_none_mag_read, sensor has no magnetometer");
	return;
}

float mag_none_temp_read(const struct i2c_dt_spec *dev_i2c)
{
	LOG_ERR("mag_none_temp_read, sensor has no magnetometer");
	return 0;
}

const sensor_mag_t sensor_mag_none = {
	*mag_none_init,
	*mag_none_shutdown,

	*mag_none_update_odr,

	*mag_none_mag_oneshot,
	*mag_none_mag_read,
	*mag_none_temp_read
};
