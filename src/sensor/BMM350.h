#ifndef BMM350_h
#define BMM350_h

#include "../sensor.h"

// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm350-ds00.pdf
#define BMM350_PMU_CMD_AGGR_SET 0x04
#define BMM350_PMU_CMD          0x06
#define BMM350_PMU_CMD_STATUS_0 0x07

#define BMM350_MAG_X_XLSB 0x31

#define BMM350_TEMP_XLSB 0x3A

#define BMM350_OTP_CMD_REG 0x50
#define BMM350_CMD 0x7E

#define AGGR_ODR_400Hz    0x02
#define AGGR_ODR_200Hz    0x03
#define AGGR_ODR_100Hz    0x04
#define AGGR_ODR_50Hz     0x05
#define AGGR_ODR_25Hz     0x06
#define AGGR_ODR_12_5Hz   0x07
#define AGGR_ODR_6_25Hz   0x08
#define AGGR_ODR_3_125Hz  0x09
#define AGGR_ODR_1_5625Hz 0x0A

#define AGGR_NO_AVG 0x00
#define AGGR_AVG_2  0x01
#define AGGR_AVG_4  0x02
#define AGGR_AVG_8  0x03

#define PMU_CMD_SUS     0x00
#define PMU_CMD_NM      0x01
#define PMU_CMD_UPD_OAE 0x02
#define PMU_CMD_FM      0x03
#define PMU_CMD_FM_FAST 0x04

int bmm3_init(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);
void bmm3_shutdown(const struct i2c_dt_spec *dev_i2c);

int bmm3_update_odr(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);

void bmm3_mag_oneshot(const struct i2c_dt_spec *dev_i2c);
void bmm3_mag_read(const struct i2c_dt_spec *dev_i2c, float m[3]);
float bmm3_temp_read(const struct i2c_dt_spec *dev_i2c, float bias[3]);

void bmm3_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_bmm350;

#endif
