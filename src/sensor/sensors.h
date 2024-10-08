#ifndef SLIMENRF_SENSOR_SENSORS
#define SLIMENRF_SENSOR_SENSORS

#include <zephyr/kernel.h>

#include "sensor_none.h"

#include "BMI270.h"
#include "ICM42688.h"
#include "LSM6DSV.h"
#include "LSM6DSO.h"

#include "BMM150.h"
#include "BMM350.h"
#include "LIS2MDL.h"
#include "LIS3MDL.h"
#include "MMC5983MA.h"

#include "scan.h"
#include "../sensor.h"

/*
Sensor:addr,reg,id

IMUs:

Bosch Sensortec
-BMI160:68/69,00,D1
*BMI270:68/69,00,24
-BMI323:68/69,00,43
InvenSense
-MPU-6000:68/69,75,68
-MPU-6050:68/69,75,68
-MPU-6500:68/69,75,70
-MPU-9250:68/69,75,71
TDK InvenSense
-ICM-20948:68/69,0,EA
*ICM-42688-P:68/69,75,47
*ICM-42688-V:68/69,75,DB
+ICM-45686:68/69,72,E9
STMicroelectronics
-LSM6DS3:6A/6B,0F,69
+LSM6DSR:6A/6B,0F,6B
*LSM6DSO:6A/6B,0F,6C
*LSM6DSV:6A/6B,0F,70

68/69,6A/6B
00,75 (EA:ICM-20948,D1:BMI160,24:BMI270,43:BMI323;68:MPU-6000/MPU-6050,70:MPU-6500,71:MPU-9250,47:ICM-42688-P,DB:ICM-42688-V)
0F (69:LSM6DS3,6B:LSM6DSR,6C:LSM6DSO,70:LSM6DSV)

Magnetometers:

QST Corporation
-QMC5883L:0B,OD,FF
Bosch Sensortec
*BMM150:10/11/12/13,40,32
*BMM350:14/15/16/17,00,33
STMicroelectronics
*IIS2MDC:1E,4F,40
*LIS2MDL:1E,4F,40
*LIS3MDL:1C/1E,0F,3D
memsic
-MMC34160PJ:30,20,06
-MMC3630KJ:30,2F,0A
-MMC5603NJ:30,39,10
-MMC5633NJL:30,39,10
-MMC5616WA:30,39,11
*MMC5983MA:30,2F,30

0B,10/11/12/13,14/15/16/17,1C,1E,30
0D (FF:QMC5883L)
40 (32:BMM150)
00 (33:BMM350)
0F (3D:LIS3MDL)
0F,4F (3D:LIS3MDL;40:IIS2MDC/LIS2MDL)
20,39,2F (06:MMC34160PJ;10:MMC5603NJ/MMC5633NJL,11:MMC5616WA;0A:MMC3630KJ,30:MMC5983MA)
*/

enum dev_imu {
	IMU_BMI160,
	IMU_BMI270,
	IMU_BMI323,
	IMU_MPU6050,
	IMU_MPU6500,
	IMU_MPU9250,
	IMU_ICM20948,
	IMU_ICM42688,
	IMU_ICM45686,
	IMU_LSM6DS3,
	IMU_LSM6DSR,
	IMU_LSM6DSO,
	IMU_LSM6DSV
};
const char *dev_imu_names[] = {
	"BMI160",
	"BMI270",
	"BMI323",
	"MPU-6000/MPU-6050",
	"MPU-6500",
	"MPU-9250",
	"ICM-20948",
	"ICM-42688-P/ICM-42688-V",
	"ICM-45686",
	"LSM6DS3",
	"LSM6DSR",
	"LSM6DSO",
	"LSM6DSV"
};
const sensor_imu_t *sensor_imus[] = {
	&sensor_imu_none, // will not implement, too low quality
	&sensor_imu_bmi270,
	&sensor_imu_none,
	&sensor_imu_none, // cardinal sin
	&sensor_imu_none, // cardinal sin
	&sensor_imu_none, // cardinal sin
	&sensor_imu_none,
	&sensor_imu_icm42688,
	&sensor_imu_none,
	&sensor_imu_none, // will not implement, too low quality
	&sensor_imu_none,
	&sensor_imu_lsm6dso,
	&sensor_imu_lsm6dsv
};
const int i2c_dev_imu_addr_count = 2;
const uint8_t i2c_dev_imu_addr[] = {
	2,	0x68,0x69,
	2,	0x6A,0x6B
};
const uint8_t i2c_dev_imu_reg[] = {
	3,	0x00,
		0x72,
		0x75,
	1,	0x0F
};
const uint8_t i2c_dev_imu_id[] = {
	4,	0xEA,0xD1,0x24,0x43, // reg 0x00
	1,	0xE9, // reg 0x72
	5,	0x68,0x70,0x71,0x47,0xDB, // reg 0x75
	4,	0x69,0x6B,0x6C,0x70 // reg 0x0F
};
const int i2c_dev_imu[] = {
	IMU_ICM20948, IMU_BMI160, IMU_BMI270, IMU_BMI323,
	IMU_ICM45686,
	IMU_MPU6050, IMU_MPU6500, IMU_MPU9250, IMU_ICM42688, IMU_ICM42688, // ICM-42688-P, ICM-42688-V
	IMU_LSM6DS3, IMU_LSM6DSR, IMU_LSM6DSO, IMU_LSM6DSV
};

enum dev_mag {
	MAG_QMC5883L,
	MAG_BMM150,
	MAG_BMM350,
	MAG_LIS2MDL, // IIS2MDC/LIS2MDL
	MAG_LIS3MDL,
	MAG_MMC34160PJ,
	MAG_MMC3630KJ,
	MAG_MMC5633NJL, // MMC5603NJ/MMC5633NJL
	MAG_MMC5616WA,
	MAG_MMC5983MA
};
const char *dev_mag_names[] = {
	"QMC5883L",
	"BMM150",
	"BMM350",
	"IIS2MDC/LIS2MDL",
	"LIS3MDL",
	"MMC34160PJ",
	"MMC3630KJ",
	"MMC5603NJ/MMC5633NJL",
	"MMC5616WA",
	"MMC5983MA"
};
const sensor_mag_t *sensor_mags[] = {
	&sensor_mag_none, // not implemented
	&sensor_mag_bmm150,
	&sensor_mag_bmm350,
	&sensor_mag_lis2mdl,
	&sensor_mag_lis3mdl,
	&sensor_mag_none,
	&sensor_mag_none,
	&sensor_mag_none,
	&sensor_mag_none,
	&sensor_mag_mmc5983ma
};
const int i2c_dev_mag_addr_count = 6;
const uint8_t i2c_dev_mag_addr[] = {
	1,	0x0B,
	4,	0x10,0x11,0x12,0x13, // why bosch
	4,	0x14,0x15,0x16,0x17,
	1,	0x1C,
	1,	0x1E,
	1,	0x30
};
const uint8_t i2c_dev_mag_reg[] = {
	1,	0x0D,
	1,	0x40,
	1,	0x00,
	1,	0x0F,
	2,	0x0F,
		0x4F,
	3,	0x20,
		0x2F,
		0x39
};
const uint8_t i2c_dev_mag_id[] = {
	1,	0xFF, // reg 0x0D
	1,	0x32, // reg 0x40
	1,	0x33, // reg 0x00
	1,	0x3D, // reg 0x0F
	1,	0x3D, // reg 0x0F
	1,	0x40, // reg 0x4F
	1,	0x06, // reg 0x20
	2,	0x0A,0x30, // reg 0x2F
	2,	0x10,0x11 // reg 0x39
};
const int i2c_dev_mag[] = {
	MAG_QMC5883L,
	MAG_BMM150,
	MAG_BMM350,
	MAG_LIS3MDL,
	MAG_LIS3MDL,
	MAG_LIS2MDL, // IIS2MDC/LIS2MDL
	MAG_MMC34160PJ,
	MAG_MMC3630KJ, MAG_MMC5983MA,
	MAG_MMC5633NJL, MAG_MMC5616WA // MMC5603NJ/MMC5633NJL
};

int sensor_scan_imu(struct i2c_dt_spec *i2c_dev, uint8_t *i2c_dev_reg)
{
	return sensor_scan(i2c_dev, i2c_dev_reg, i2c_dev_imu_addr_count, i2c_dev_imu_addr, i2c_dev_imu_reg, i2c_dev_imu_id, i2c_dev_imu);
}

int sensor_scan_mag(struct i2c_dt_spec *i2c_dev, uint8_t *i2c_dev_reg)
{
	return sensor_scan(i2c_dev, i2c_dev_reg, i2c_dev_mag_addr_count, i2c_dev_mag_addr, i2c_dev_mag_reg, i2c_dev_mag_id, i2c_dev_mag);
}

#endif