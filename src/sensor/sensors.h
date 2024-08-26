#ifndef SLIMENRF_SENSOR_SENSORS
#define SLIMENRF_SENSOR_SENSORS

#include <zephyr/kernel.h>

#include "ICM42688.h"

#include "MMC5983MA.h"

#include "scan.h"
#include "../sensor.h"

/*
Sensor:addr,reg,id

IMUs:

Bosch Sensortec
-BMI160:68/69,00,D1
+BMI270:68/69,00,24
-BMI323:68/69,00,43
TDK InvenSense
*ICM-42688-P:68/69,75,47
*ICM-42688-V:68/69,75,DB
STMicroelectronics
-LSM6DS3:6A/6B,0F,69
-LSM6DSO:6A/6B,0F,6C
+LSM6DSV:6A/6B,0F,70

68/69,6A/6B
00,75 (D1:BMI160,24:BMI270,43:BMI323;47:ICM-42688-P,DB:ICM-42688-V)
0F (69:LSM6DS3,6C:LSM6DSO,70:LSM6DSV)

Magnetometers:

QST Corporation
-QMC5883L:0B,OD,FF
Bosch Sensortec
-BMM150:10/11/12/13,40,32
-BMM350:14/15/16/17,00,33
STMicroelectronics
-IIS2MDC:1E,4F,40
-LIS2MDL:1E,4F,40
-LIS3MDL:1C/1E,0F,3D
memsic
-MMC5603NJ:30,39,10
-MMC5633NJL:30,39,10
*MMC5983MA:30,2F,30

0B,10/11/12/13,14/15/16/17,1C,1E,30
0D (FF:QMC5883L)
40 (32:BMM150)
00 (33:BMM350)
0F (3D:LIS3MDL)
0F,4F (3D:LIS3MDL;40:IIS2MDC/LIS2MDL)
39,2F (10:MMC5603NJ/MMC5633NJL;30:MMC5983MA)
*/

enum dev_imu {
	IMU_BMI160,
	IMU_BMI270,
	IMU_BMI323,
	IMU_ICM42688,
	IMU_LSM6DS3,
	IMU_LSM6DSO,
	IMU_LSM6DSV
};
const char *dev_imu_names[] = {
	"BMI160",
	"BMI270",
	"BMI323",
	"ICM-42688-P/ICM-42688-V",
	"LSM6DS3",
	"LSM6DSO",
	"LSM6DSV"
};
const sensor_imu_t *sensor_imus[] = {
	NULL, // not implemented
	NULL,
	NULL,
	&sensor_imu_icm42688,
	NULL,
	NULL,
	NULL
};
const int i2c_dev_imu_addr_count = 2;
const uint8_t i2c_dev_imu_addr[] = {
	2, 0x68,0x69,
	2, 0x6A,0x6B
};
const uint8_t i2c_dev_imu_reg[] = {
	2, 0x00,0x75,
	1, 0x0F
};
const uint8_t i2c_dev_imu_id[] = {
	3, 0xD1,0x24,0x43, // 0x00
	2, 0x47,0xDB, // 0x75
	3, 0x69,0x6C,0x70 // 0x0F
};
const int i2c_dev_imu[] = {
	IMU_BMI160, IMU_BMI270, IMU_BMI323,
	IMU_ICM42688, IMU_ICM42688, // ICM-42688-P, ICM-42688-V
	IMU_LSM6DS3, IMU_LSM6DSO, IMU_LSM6DSV
};

enum dev_mag {
	MAG_QMC5883L,
	MAG_BMM150,
	MAG_BMM350,
	MAG_LIS3MDL,
	MAG_LIS2MDL, // IIS2MDC/LIS2MDL
	MAG_MMC5633NJL, // MMC5603NJ/MMC5633NJL
	MAG_MMC5983MA
};
const char *dev_mag_names[] = {
	"QMC5883L",
	"BMM150",
	"BMM350",
	"LIS3MDL",
	"IIS2MDC/LIS2MDL",
	"MMC5603NJ/MMC5633NJL",
	"MMC5983MA"
};
const sensor_mag_t *sensor_mags[] = {
	NULL, // not implemented
	NULL,
	NULL,
	&sensor_mag_mmc5983ma,
	NULL,
	NULL,
	NULL
};
const int i2c_dev_mag_addr_count = 6;
const uint8_t i2c_dev_mag_addr[] = {
	1, 0x0B,
	4, 0x10,0x11,0x12,0x13,
	4, 0x14,0x15,0x16,0x17,
	1, 0x1C,
	1, 0x1E,
	1, 0x30
};
const uint8_t i2c_dev_mag_reg[] = {
	1, 0x0D,
	1, 0x40,
	1, 0x00,
	1, 0x0F,
	2, 0x0F,0x4F,
	2, 0x39,0x2F
};
const uint8_t i2c_dev_mag_id[] = {
	1, 0xFF, // 0x0D
	1, 0x32, // 0x40
	1, 0x33, // 0x00
	1, 0x3D, // 0x0F
	1, 0x3D, // 0x0F
	1, 0x40, // 0x4F
	1, 0x10, // 0x39
	1, 0x30 // 0x2F
};
const int i2c_dev_mag[] = {
	MAG_QMC5883L,
	MAG_BMM150,
	MAG_BMM350,
	MAG_LIS3MDL,
	MAG_LIS3MDL,
	MAG_LIS2MDL, // IIS2MDC/LIS2MDL
	MAG_MMC5633NJL, // MMC5603NJ/MMC5633NJL
	MAG_MMC5983MA
};

int sensor_scan_imu(struct i2c_dt_spec i2c_dev)
{
	return sensor_scan(i2c_dev, i2c_dev_imu_addr_count, i2c_dev_imu_addr, i2c_dev_imu_reg, i2c_dev_imu_id, i2c_dev_imu);
}

int sensor_scan_mag(struct i2c_dt_spec i2c_dev)
{
	return sensor_scan(i2c_dev, i2c_dev_mag_addr_count, i2c_dev_mag_addr, i2c_dev_mag_reg, i2c_dev_mag_id, i2c_dev_mag);
}

#endif