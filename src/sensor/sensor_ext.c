#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

#include "sensor_none.h"
#include "sensor_ext.h"

static const sensor_imu_t *ext_imu = &sensor_imu_none;
static const sensor_mag_t *ext_mag = &sensor_mag_none;

LOG_MODULE_REGISTER(sensor_ext, LOG_LEVEL_INF);

void mag_ext_setup(const sensor_imu_t *imu, const sensor_mag_t *mag)
{
	ext_imu = imu;
	ext_mag = mag;
}

int mag_ext_init(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time)
{
	(*ext_imu->ext_passthrough)(dev_i2c, true);
	int err = (*ext_mag->init)(dev_i2c, time, actual_time);
	(*ext_imu->ext_passthrough)(dev_i2c, false);
	return err;
}

void mag_ext_shutdown(const struct i2c_dt_spec *dev_i2c)
{
	(*ext_imu->ext_passthrough)(dev_i2c, true);
	(*ext_mag->shutdown)(dev_i2c);
	(*ext_imu->ext_passthrough)(dev_i2c, false);
}

int mag_ext_update_odr(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time)
{
	(*ext_imu->ext_passthrough)(dev_i2c, true);
	int err = (*ext_mag->update_odr)(dev_i2c, time, actual_time);
	(*ext_imu->ext_passthrough)(dev_i2c, false);
	return err;
}

void mag_ext_mag_oneshot(const struct i2c_dt_spec *dev_i2c)
{
	(*ext_imu->ext_passthrough)(dev_i2c, true);
	(*ext_mag->mag_oneshot)(dev_i2c);
	(*ext_imu->ext_passthrough)(dev_i2c, false);
}

void mag_ext_mag_read(const struct i2c_dt_spec *dev_i2c, float m[3])
{
	uint8_t raw_m[10] = {0};
	(*ext_imu->ext_read)(dev_i2c, raw_m);
	(*ext_mag->mag_process)(raw_m, m);
}

float mag_ext_temp_read(const struct i2c_dt_spec *dev_i2c)
{
	(*ext_imu->ext_passthrough)(dev_i2c, true);
	float temp = (*ext_mag->temp_read)(dev_i2c);
	(*ext_imu->ext_passthrough)(dev_i2c, false);
	return temp;
}

void mag_ext_mag_process(uint8_t *raw_m, float m[3])
{
	(*ext_mag->mag_process)(raw_m, m);
}

const sensor_mag_t sensor_mag_ext = {
	*mag_ext_init,
	*mag_ext_shutdown,

	*mag_ext_update_odr,

	*mag_ext_mag_oneshot,
	*mag_ext_mag_read,
	*mag_ext_temp_read,

	*mag_ext_mag_process
};
