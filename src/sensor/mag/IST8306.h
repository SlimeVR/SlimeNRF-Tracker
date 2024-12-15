#ifndef IST8306_h
#define IST8306_h

#include "../../sensor.h"

#define IST8306_STAT 0x10

#define IST8306_DATAXL 0x11

#define IST8306_ACTR 0x20
#define IST8306_CNTL1   0x30
#define IST8306_CNTL2   0x31
#define IST8306_CNTL3   0x32
#define IST8306_OSRCNTL 0x41

#define NSF_Disable 0b00
#define NSF_Low     0b01
#define NSF_Medium  0b10
#define NSF_High    0b11

#define MODE_STANDBY   0x00
#define MODE_SINGLE    0x01
#define MODE_CMM_10Hz  0x02
#define MODE_CMM_20Hz  0x04
#define MODE_CMM_50Hz  0x06
#define MODE_CMM_100Hz 0x08
#define MODE_CMM_200Hz 0x0A
#define MODE_CMM_8Hz   0x0B
#define MODE_CMM_1Hz   0x0C
#define MODE_CMM_0_5Hz 0x0D

#define OSR_1  0b000000
#define OSR_2  0b001001
#define OSR_4  0b010010
#define OSR_8  0b011011
#define OSR_16 0b100100
#define OSR_32 0b101101

int ist8306_init(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);
void ist8306_shutdown(const struct i2c_dt_spec *dev_i2c);

int ist8306_update_odr(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);

void ist8306_mag_oneshot(const struct i2c_dt_spec *dev_i2c);
void ist8306_mag_read(const struct i2c_dt_spec *dev_i2c, float m[3]);

void ist8306_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_ist8306;

#endif
