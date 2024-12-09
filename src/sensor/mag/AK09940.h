#ifndef AK09940_h
#define AK09940_h

#include "../../sensor.h"

// https://www.akm.com/content/dam/documents/products/tri-axis-magnetic-sensor/ak09940a/ak09940a-en-datasheet-myakm.pdf
#define AK09940_ST   0x0F

#define AK09940_ST1  0x10

#define AK09940_HXL  0x11

#define AK09940_TMPS 0x1A

#define AK09940_ST2  0x1B

#define AK09940_CNTL1  0x30
#define AK09940_CNTL3  0x32
#define AK09940_CNTL4  0x33

#define MODE_PDM         0b00000 // Power-down mode
#define MODE_SMM         0b00001 // Single measurement mode
#define MODE_CMM1_10Hz   0b00010 // Continuous measurement modes
#define MODE_CMM2_20Hz   0b00100
#define MODE_CMM3_50Hz   0b00110
#define MODE_CMM4_100Hz  0b01000
#define MODE_CMM5_200Hz  0b01010
// Low power drive 1, 2, or Ultra low power drive
#define MODE_CMM6_400Hz  0b01100
// Low power drive 1, or Ultra low power drive
#define MODE_CMM7_1000Hz 0b01110
// Ultra low power drive
#define MODE_CMM8_2500Hz 0b01111

#define MT_LPD1 0b00 // Low power drive 1
#define MT_LPD2 0b01 // Low power drive 2
#define MT_LND1 0b10 // Low noise drive 1
#define MT_LND2 0b11 // Low noise drive 2

int ak_init(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);
void ak_shutdown(const struct i2c_dt_spec *dev_i2c);

int ak_update_odr(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);

void ak_mag_oneshot(const struct i2c_dt_spec *dev_i2c);
void ak_mag_read(const struct i2c_dt_spec *dev_i2c, float m[3]);
float ak_temp_read(const struct i2c_dt_spec *dev_i2c, float bias[3]);

void ak_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_ak09940;

#endif
