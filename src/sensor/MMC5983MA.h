/* 06/14/2020 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively, and it uses the Ladybug STM32L432 Breakout Board.
  The MMC5983MA is a low power magnetometer, here used as 3 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#ifndef MMC5983MA_h
#define MMC5983MA_h

#include "../sensor.h"

// http://www.memsic.com/userfiles/files/DataSheets/Magnetic-Sensors-Datasheets/MMC5983MA_Datasheet.pdf
#define MMC5983MA_XOUT_0        0x00
#define MMC5983MA_TOUT          0x07

#define MMC5983MA_CONTROL_0     0x09
#define MMC5983MA_CONTROL_1     0x0A
#define MMC5983MA_CONTROL_2     0x0B

// Sample rates
#define MODR_ONESHOT   0x00
#define MODR_1Hz       0x01
#define MODR_10Hz      0x02
#define MODR_20Hz      0x03
#define MODR_50Hz      0x04
#define MODR_100Hz     0x05
#define MODR_200Hz     0x06 // BW = 0x01 only
#define MODR_1000Hz    0x07 // BW = 0x11 only

//Bandwidths
#define MBW_100Hz 0x00  // 8 ms measurement time
#define MBW_200Hz 0x01  // 4 ms
#define MBW_400Hz 0x02  // 2 ms
#define MBW_800Hz 0x03  // 0.5 ms

// Set/Reset as a function of measurements
#define MSET_1     0x00 // Set/Reset each data measurement
#define MSET_25    0x01 // each 25 data measurements
#define MSET_75    0x02
#define MSET_100   0x03
#define MSET_250   0x04
#define MSET_500   0x05
#define MSET_1000  0x06
#define MSET_2000  0x07

int mmc_init(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);
void mmc_shutdown(const struct i2c_dt_spec *dev_i2c);

int mmc_update_odr(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time);

void mmc_mag_oneshot(const struct i2c_dt_spec *dev_i2c);
void mmc_mag_read(const struct i2c_dt_spec *dev_i2c, float m[3]);
float mmc_temp_read(const struct i2c_dt_spec *dev_i2c);

extern const sensor_mag_t sensor_mag_mmc5983ma;

#endif
