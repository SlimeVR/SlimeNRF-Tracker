#ifndef LIS3MDL_h
#define LIS3MDL_h

#include "../sensor.h"

// https://www.st.com/resource/en/datasheet/lis3mdl.pdf
#define LIS3MDL_CTRL_REG1 0x20
#define LIS3MDL_CTRL_REG2 0x21
#define LIS3MDL_CTRL_REG3 0x22
#define LIS3MDL_CTRL_REG4 0x23

#define LIS3MDL_OUT_X_L 0x28

#define LIS3MDL_TEMP_OUT_L 0x2E

// X/Y and Z (OMZ)
#define OM_LP  0x00 // Low power, 1000Hz with FAST_ODR
#define OM_MP  0x01 // Medium performance, 560Hz with FAST_ODR
#define OM_HP  0x02 // High performance, 300Hz with FAST_ODR
#define OM_UHP 0x03 // Ultrahigh performance, 155Hz with FAST_ODR

#define DO_0_625Hz  0x00
#define DO_1_25Hz   0x01
#define DO_2_5Hz    0x02
#define DO_5Hz      0x03
#define DO_10Hz     0x04
#define DO_20Hz     0x05
#define DO_40Hz     0x06
#define DO_80Hz     0x07

#define FS_4G  0x00
#define FS_8G  0x01
#define FS_12G 0x02
#define FS_16G 0x03

#define MD_CONTINUOUS_CONV 0x00
#define MD_SINGLE_CONV     0x01 // <= 80Hz only
#define MD_POWER_DOWN      0x03

int lis3_init(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);
void lis3_shutdown(const struct i2c_dt_spec *dev_i2c);

int lis3_update_odr(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);

void lis3_mag_oneshot(const struct i2c_dt_spec *dev_i2c);
void lis3_mag_read(const struct i2c_dt_spec *dev_i2c, float m[3]);
float lis3_temp_read(const struct i2c_dt_spec *dev_i2c, float bias[3]);

void lis3_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_lis3mdl;

#endif
