#ifndef BMM150_h
#define BMM150_h

#include "../../sensor.h"

// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm150-ds001.pdf
#define BMM150_DATAX_LSB 0x42

#define BMM150_RHALL_LSB 0x48

#define BMM150_POWER_CTRL 0x4B
#define BMM150_OP_CTRL    0x4C

#define BMM150_REP_XY 0x51 // 1+2(REP_XY)
#define BMM150_REP_Z  0x52 // 1+REP_Z

// taken from boschsensortec BMM150_SensorAPI
#define BMM150_DIG_X1     0x5D
#define BMM150_DIG_Z4_LSB 0x62
#define BMM150_DIG_Z2_LSB 0x68

#define DR_ODR_10Hz 0x00
#define DR_ODR_2Hz 0x01
#define DR_ODR_6Hz 0x02
#define DR_ODR_8Hz 0x03
#define DR_ODR_15Hz 0x04
#define DR_ODR_20Hz 0x05
#define DR_ODR_25Hz 0x06
#define DR_ODR_30Hz 0x07

#define OPMODE_NORMAL 0x00
#define OPMODE_FORCED 0x01
#define OPMODE_SLEEP 0x03

int bmm1_init(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);
void bmm1_shutdown(const struct i2c_dt_spec *dev_i2c);

int bmm1_update_odr(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);

void bmm1_mag_oneshot(const struct i2c_dt_spec *dev_i2c);
void bmm1_mag_read(const struct i2c_dt_spec *dev_i2c, float m[3]);

void bmm1_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_bmm150;

#endif
