#ifndef SLIMENRF_SENSOR_EXT
#define SLIMENRF_SENSOR_EXT

#include "../sensor.h"

int mag_ext_setup(const sensor_imu_t *imu, const sensor_mag_t *mag, uint8_t addr);

int mag_ext_init(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);
void mag_ext_shutdown(const struct i2c_dt_spec *dev_i2c);

int mag_ext_update_odr(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);

void mag_ext_mag_oneshot(const struct i2c_dt_spec *dev_i2c);
void mag_ext_mag_read(const struct i2c_dt_spec *dev_i2c, float m[3]);
float mag_ext_temp_read(const struct i2c_dt_spec *dev_i2c);

void mag_ext_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_ext;

#endif
