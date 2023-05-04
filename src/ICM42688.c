/* 01/14/2022 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively, and it uses the Ladybug STM32L432 Breakout Board.
  The ICM42688 is a combo sensor with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/types.h>
#include <zephyr/drivers/i2c.h>
#include "ICM42688.h"

float _aRes, _gRes;

uint8_t icm_getChipID(struct i2c_dt_spec dev_i2c)
{
    uint8_t temp;
    i2c_reg_read_byte_dt(&dev_i2c, ICM42688_WHO_AM_I, &temp);
    return temp;
}

float icm_getAres(uint8_t Ascale) {
    switch (Ascale)
    {
        // Possible accelerometer scales (and their register bit settings) are:
        case AFS_2G:
            _aRes = 2.0f/32768.0f;
            return _aRes;
            break;
        case AFS_4G:
            _aRes = 4.0f/32768.0f;
            return _aRes;
            break;
        case AFS_8G:
            _aRes = 8.0f/32768.0f;
            return _aRes;
            break;
        case AFS_16G:
            _aRes = 16.0f/32768.0f;
            return _aRes;
            break;
    }
}

float icm_getGres(uint8_t Gscale) {
    switch (Gscale)
    {
        // Possible gyro scales (and their register bit settings) are:
        case GFS_15_625DPS:
            _gRes = 15.625f/32768.0f;
            return _gRes;
            break;
        case GFS_31_25DPS:
            _gRes = 31.25f/32768.0f;
            return _gRes;
            break;
        case GFS_62_50DPS:
            _gRes = 62.5f/32768.0f;
            return _gRes;
            break;
        case GFS_125DPS:
            _gRes = 125.0f/32768.0f;
            return _gRes;
            break;
        case GFS_250DPS:
            _gRes = 250.0f/32768.0f;
            return _gRes;
            break;
        case GFS_500DPS:
            _gRes = 500.0f/32768.0f;
            return _gRes;
            break;
        case GFS_1000DPS:
            _gRes = 1000.0f/32768.0f;
            return _gRes;
            break;
        case GFS_2000DPS:
            _gRes = 2000.0f/32768.0f;
            return _gRes;
            break;
    }
}


void icm_reset(struct i2c_dt_spec dev_i2c)
{
    // reset device
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_DEVICE_CONFIG, 0x01); // Set bit 0 to 1 to reset ICM42688
    k_msleep(2); // Wait 1 ms for all registers to reset
}


uint8_t icm_DRStatus(struct i2c_dt_spec dev_i2c)
{
    uint8_t temp;
    i2c_reg_read_byte_dt(&dev_i2c, ICM42688_INT_STATUS, &temp);
    return temp;
}


void icm_init(struct i2c_dt_spec dev_i2c, uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR, uint8_t aMode, uint8_t gMode, bool CLKIN)
{
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_INT_SOURCE0, 0x00); // temporary disable interrupts
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_PWR_MGMT0, gMode << 2 | aMode); // set accel and gyro modes
    k_busy_wait(250); // wait >200us (datasheet 14.36)
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_ACCEL_CONFIG0, Ascale << 5 | AODR); // set accel ODR and FS
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_GYRO_CONFIG0, Gscale << 5 | GODR); // set gyro ODR and FS
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_GYRO_ACCEL_CONFIG0, 0x44); // set gyro and accel bandwidth to ODR/10
//    // interrupt handling
//    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_INT_CONFIG, 0x18 | 0x03 ); // push-pull, pulsed, active HIGH interrupts
//    uint8_t temp;
//    i2c_reg_read_byte_dt(&dev_i2c, ICM42688_INT_CONFIG1, &temp); // clear bit 4 to allow async interrupt reset (required for proper interrupt operation)
//    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_INT_CONFIG1, temp & ~(0x10)); // clear bit 4 to allow async interrupt reset (required for proper interrupt operation)
//    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_INT_SOURCE0, 0x08); // data ready interrupt routed to INT1
//    // Use external clock source
//    if(CLKIN) {
//        i2c_reg_write_byte_dt(&dev_i2c, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
//
//        i2c_reg_write_byte_dt(&dev_i2c, ICM42688_INTF_CONFIG1, 0x95); // enable RTC
//
//        i2c_reg_write_byte_dt(&dev_i2c, ICM42688_REG_BANK_SEL, 0x01); // select register bank 1
//
//        i2c_reg_write_byte_dt(&dev_i2c, ICM42688_INTF_CONFIG5, 0x04); // use CLKIN as clock source
//    }
	/*
	 * Accelerometer sensor need at least 10ms startup time
	 * Gyroscope sensor need at least 30ms startup time
	 */
	k_msleep(50);
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_FIFO_CONFIG, 0x00); // FIFO bypass mode
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_FSYNC_CONFIG, 0x00); // disable FSYNC
    i2c_reg_update_byte_dt(&dev_i2c, ICM42688_TMST_CONFIG, 0x02, 0x00); // disable FSYNC
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_FIFO_CONFIG1, 0x02); // enable FIFO gyro only
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_FIFO_CONFIG, 1<<6); // begin FIFO stream
}


void icm_offsetBias(struct i2c_dt_spec dev_i2c, float * dest1, float * dest2) // need to make this external
{
    int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
    int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};

    for (int ii = 0; ii < 500; ii++)
    {
        icm_readData(dev_i2c, temp);
        sum[1] += temp[1];
        sum[2] += temp[2];
        sum[3] += temp[3];
        sum[4] += temp[4];
        sum[5] += temp[5];
        sum[6] += temp[6];
        k_msleep(5);
    }

    dest1[0] = sum[1]*_aRes/500.0f;
    dest1[1] = sum[2]*_aRes/500.0f;
    dest1[2] = sum[3]*_aRes/500.0f;
    dest2[0] = sum[4]*_gRes/500.0f;
    dest2[1] = sum[5]*_gRes/500.0f;
    dest2[2] = sum[6]*_gRes/500.0f;

    if(dest1[0] > 0.8f) {dest1[0] -= 1.0f;} // Remove gravity from the x-axis accelerometer bias calculation
    if(dest1[0] < -0.8f) {dest1[0] += 1.0f;} // Remove gravity from the x-axis accelerometer bias calculation
    if(dest1[1] > 0.8f) {dest1[1] -= 1.0f;} // Remove gravity from the y-axis accelerometer bias calculation
    if(dest1[1] < -0.8f) {dest1[1] += 1.0f;} // Remove gravity from the y-axis accelerometer bias calculation
    if(dest1[2] > 0.8f) {dest1[2] -= 1.0f;} // Remove gravity from the z-axis accelerometer bias calculation
    if(dest1[2] < -0.8f) {dest1[2] += 1.0f;} // Remove gravity from the z-axis accelerometer bias calculation
    /*
    // load offset biases into offset registers (optional, comment out if not desired)
    temp[0] = (int16_t) (-dest1[0] / 0.00048828125f); // Ax 0.5 mg resolution
    temp[1] = (int16_t) (-dest1[1] / 0.00048828125f); // Ay
    temp[2] = (int16_t) (-dest1[2] / 0.00048828125f); // Az
    temp[3] = (int16_t) (-dest2[0] / 0.03125f); // Gx 1/32 dps resolution
    temp[4] = (int16_t) (-dest2[1] / 0.03125f); // Gy
    temp[5] = (int16_t) (-dest2[2] / 0.03125f); // Gz

    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_REG_BANK_SEL, 0x04); // select register bank 4

    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_OFFSET_USER5, temp[0] & 0x00FF); // lower Ax byte
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_OFFSET_USER6, temp[1] & 0x00FF); // lower Ay byte
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_OFFSET_USER8, temp[2] & 0x00FF); // lower Az byte
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_OFFSET_USER2, temp[4] & 0x00FF); // lower Gy byte
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_OFFSET_USER3, temp[5] & 0x00FF); // lower Gz byte
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_OFFSET_USER0, temp[3] & 0x00FF); // lower Gx byte
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_OFFSET_USER4, (temp[0] & 0x0F00) >> 4 | (temp[5] & 0x0F00) >> 8); // upper Ax and Gz bytes
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_OFFSET_USER7, (temp[2] & 0x0F00) >> 4 | (temp[1] & 0x0F00) >> 8); // upper Az and Ay bytes
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_OFFSET_USER1, (temp[4] & 0x0F00) >> 4 | (temp[3] & 0x0F00) >> 8); // upper Gy and Gx bytes
    
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
    */
}

void icm_readData(struct i2c_dt_spec dev_i2c, int16_t * destination)
{
    uint8_t rawData[14]; // x/y/z accel register data stored here
    i2c_burst_read_dt(&dev_i2c, ICM42688_TEMP_DATA1, &rawData[0], 14); // Read the 14 raw data registers into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ; // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
    destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
    destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
    destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
    destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
}
