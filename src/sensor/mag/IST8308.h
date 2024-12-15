#ifndef IST8308_h
#define IST8308_h

#include "../../sensor.h"

#define IST8308_CNTL4   0x33

#define DR_500 0b00
#define DR_200 0b1

int ist8308_init(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);
void ist8308_shutdown(const struct i2c_dt_spec *dev_i2c);

int ist8308_update_odr(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);

void ist8308_mag_oneshot(const struct i2c_dt_spec *dev_i2c);
void ist8308_mag_read(const struct i2c_dt_spec *dev_i2c, float m[3]);

void ist8308_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_ist8308;

#endif
