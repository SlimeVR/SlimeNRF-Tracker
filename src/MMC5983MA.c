/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively, and it uses the Ladybug STM32L432 Breakout Board.
  The MMC5983MA is a low power magnetometer, here used as 3 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/types.h>
#include <zephyr/drivers/i2c.h>
#include "MMC5983MA.h"

uint8_t mmc_getChipID(struct i2c_dt_spec dev_i2c)
{
    uint8_t c;
    i2c_reg_read_byte_dt(&dev_i2c, MMC5983MA_PRODUCT_ID, &c);
    return c;
}

void mmc_reset(struct i2c_dt_spec dev_i2c)
{
    // reset device
    i2c_reg_write_byte_dt(&dev_i2c, MMC5983MA_CONTROL_1, 0x80); // Set bit 7 to 1 to reset MMC5983MA
    k_msleep(11); // Wait 10 ms for all registers to reset 
}

void mmc_init(struct i2c_dt_spec dev_i2c, uint8_t MODR, uint8_t MBW, uint8_t MSET)
{
    // enable auto set/reset (bit 5 == 1)
    i2c_reg_write_byte_dt(&dev_i2c, MMC5983MA_CONTROL_0, 0x20);

    // set magnetometer bandwidth
    i2c_reg_write_byte_dt(&dev_i2c, MMC5983MA_CONTROL_1, MBW);

    // enable continuous measurement mode (bit 3 == 1), set sample rate
    // enable automatic Set/Reset (bit 8 == 1), set set/reset rate
    i2c_reg_write_byte_dt(&dev_i2c, MMC5983MA_CONTROL_2, 0x80 | (MSET << 4) | 0x08 | MODR);
}

void mmc_selfTest(struct i2c_dt_spec dev_i2c)
{
    uint8_t rawData[6] = {0}; // x/y/z mag register data stored here
    uint16_t data_set[3] ={0}, data_reset[3] = {0};
    uint32_t delta_data[3] = {0};

    // clear control registers
    i2c_reg_write_byte_dt(&dev_i2c, MMC5983MA_CONTROL_0, 0x00);
    i2c_reg_write_byte_dt(&dev_i2c, MMC5983MA_CONTROL_1, 0x00);
    i2c_reg_write_byte_dt(&dev_i2c, MMC5983MA_CONTROL_2, 0x00);

    mmc_SET(dev_i2c); // enable set current
    i2c_reg_write_byte_dt(&dev_i2c, MMC5983MA_CONTROL_0, 0x01); //enable one-time mag measurement
    k_busy_wait(1000 * 10);

    i2c_burst_read_dt(&dev_i2c, MMC5983MA_XOUT_0, &rawData[0], 6); // Read the 6 raw data registers into data array
    data_set[0] = (uint16_t) (((uint16_t) rawData[0] << 8) | rawData[1]); // x-axis
    data_set[1] = (uint16_t) (((uint16_t) rawData[2] << 8) | rawData[3]); // y-axis
    data_set[2] = (uint16_t) (((uint16_t) rawData[4] << 8) | rawData[5]); // z-axis

    mmc_RESET(dev_i2c); // enable reset current
    i2c_reg_write_byte_dt(&dev_i2c, MMC5983MA_CONTROL_0, 0x01); //enable one-time mag measurement
    k_busy_wait(1000 * 10);

    i2c_burst_read_dt(&dev_i2c, MMC5983MA_XOUT_0, &rawData[0], 6); // Read the 6 raw data registers into data array
    data_reset[0] = (uint16_t) (((uint16_t) rawData[0] << 8) | rawData[1]); // x-axis
    data_reset[1] = (uint16_t) (((uint16_t) rawData[2] << 8) | rawData[3]); // y-axis
    data_reset[2] = (uint16_t) (((uint16_t) rawData[4] << 8) | rawData[5]); // z-axis
 
    for (uint8_t ii = 0; ii < 3; ii++)
    {
        if(data_set[ii] > data_reset[ii])
        {
            delta_data[ii] = data_set[ii] - data_reset[ii];
        }
        else
        {
            delta_data[ii] = data_reset[ii] - data_set[ii];
        }
    }

    //Serial.print("x-axis self test = "); //Serial.print(delta_data[0]); //Serial.println(", should be >100");
    //Serial.print("y-axis self test = "); //Serial.print(delta_data[1]); //Serial.println(", should be >100");
    //Serial.print("z-axis self test = "); //Serial.print(delta_data[2]); //Serial.println(", should be >100");
}


void mmc_getOffset(struct i2c_dt_spec dev_i2c, float * destination)
{
    uint8_t rawData[6] = {0}; // x/y/z mag register data stored here
    uint16_t data_set[3] ={0}, data_reset[3] = {0};

    mmc_powerDown(dev_i2c);
 
    mmc_SET(dev_i2c); // enable set current
    i2c_reg_write_byte_dt(&dev_i2c, MMC5983MA_CONTROL_0, 0x01); //enable one-time mag measurement
    k_msleep(10);

    i2c_burst_read_dt(&dev_i2c, MMC5983MA_XOUT_0, &rawData[0], 6); // Read the 6 raw data registers into data array
    data_set[0] = (uint16_t) (((uint16_t) rawData[0] << 8) | rawData[1]); // x-axis
    data_set[1] = (uint16_t) (((uint16_t) rawData[2] << 8) | rawData[3]); // y-axis
    data_set[2] = (uint16_t) (((uint16_t) rawData[4] << 8) | rawData[5]); // z-axis

    mmc_RESET(dev_i2c); // enable reset current
    i2c_reg_write_byte_dt(&dev_i2c, MMC5983MA_CONTROL_0, 0x01); //enable one-time mag measurement
    k_msleep(10);

    i2c_burst_read_dt(&dev_i2c, MMC5983MA_XOUT_0, &rawData[0], 6); // Read the 6 raw data registers into data array
    data_reset[0] = (uint16_t) (((uint16_t) rawData[0] << 8) | rawData[1]); // x-axis
    data_reset[1] = (uint16_t) (((uint16_t) rawData[2] << 8) | rawData[3]); // y-axis
    data_reset[2] = (uint16_t) (((uint16_t) rawData[4] << 8) | rawData[5]); // z-axis
 
    for (uint8_t ii = 0; ii < 3; ii++)
    {
        destination[ii] = ((float)data_set[ii] + (float)data_reset[ii])/2.0f;
    }
}

void mmc_SET(struct i2c_dt_spec dev_i2c)
{
    i2c_reg_write_byte_dt(&dev_i2c, MMC5983MA_CONTROL_0, 0x08);
    k_busy_wait(1); // self clearing after 500 ns
}

void mmc_RESET(struct i2c_dt_spec dev_i2c)
{
    i2c_reg_write_byte_dt(&dev_i2c, MMC5983MA_CONTROL_0, 0x10);
    k_busy_wait(1); // self clearing after 500 ns
}

uint8_t mmc_status(struct i2c_dt_spec dev_i2c)
{
    // Read status register
    uint8_t temp;
    i2c_reg_read_byte_dt(&dev_i2c, MMC5983MA_STATUS, &temp);
    return temp;
}

void mmc_clearInt(struct i2c_dt_spec dev_i2c)   
{
    // Clear data ready interrupts
    i2c_reg_write_byte_dt(&dev_i2c, MMC5983MA_STATUS, 0x01);
}

void mmc_readData(struct i2c_dt_spec dev_i2c, uint32_t * destination)
{
    uint8_t rawData[7]; // x/y/z mag register data stored here
    i2c_burst_read_dt(&dev_i2c, MMC5983MA_XOUT_0, &rawData[0], 7); // Read the 7 raw data registers into data array
    destination[0] = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 | (rawData[6] & 0xC0) >> 6); // Turn the 18 bits into a unsigned 32-bit value
    destination[1] = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 | (rawData[6] & 0x30) >> 4); // Turn the 18 bits into a unsigned 32-bit value
    destination[2] = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 | (rawData[6] & 0x0C) >> 2); // Turn the 18 bits into a unsigned 32-bit value
}

uint8_t mmc_readTemperature(struct i2c_dt_spec dev_i2c)
{
    uint8_t temp;
    i2c_reg_read_byte_dt(&dev_i2c, MMC5983MA_CONTROL_0, &temp); // preserve register contents
    i2c_reg_write_byte_dt(&dev_i2c, MMC5983MA_CONTROL_0, temp | 0x02); //enable one-time temp measurement
    k_busy_wait(1000 * 10);
    i2c_reg_read_byte_dt(&dev_i2c, MMC5983MA_TOUT, &temp); // Read the raw temperature register
    return temp;
}

void mmc_offsetBias(struct i2c_dt_spec dev_i2c, float * dest1, float * dest2)
{
    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int32_t mag_max[3] = {-262143, -262143, -262143}, mag_min[3] = {262143, 262143, 262143};
    uint32_t mag_temp[3] = {0, 0, 0}, magOffset = 131072;
    float _mRes = 1.0f/16384.0f; // mag sensitivity if using 18 bit data

    //Serial.println("Calculate mag offset bias: move all around to sample the complete response surface!");
    //k_busy_wait(1000 * 4000);

    for (int ii = 0; ii < 2000; ii++) // About 20 seconds
    {
        mmc_readData(dev_i2c, mag_temp);
        for (int jj = 0; jj < 3; jj++) {
            if((int32_t)(mag_temp[jj] - magOffset) > mag_max[jj]) mag_max[jj] = (int32_t)(mag_temp[jj] - magOffset);
            if((int32_t)(mag_temp[jj] - magOffset) < mag_min[jj]) mag_min[jj] = (int32_t)(mag_temp[jj] - magOffset);
        }
        k_msleep(10);
    }

    // Get hard iron correction
    mag_bias[0] = (mag_max[0] + mag_min[0])/2; // get average x mag bias in counts
    mag_bias[1] = (mag_max[1] + mag_min[1])/2; // get average y mag bias in counts
    mag_bias[2] = (mag_max[2] + mag_min[2])/2; // get average z mag bias in counts

    dest1[0] = (float) (mag_bias[0]) * _mRes; // save mag biases in G for main program
    dest1[1] = (float) (mag_bias[1]) * _mRes;
    dest1[2] = (float) (mag_bias[2]) * _mRes;

    // Get soft iron correction estimate
    mag_scale[0] = (mag_max[0] - mag_min[0])/2; // get average x axis max chord length in counts
    mag_scale[1] = (mag_max[1] - mag_min[1])/2; // get average y axis max chord length in counts
    mag_scale[2] = (mag_max[2] - mag_min[2])/2; // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0f;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);

    //Serial.println("Mag Calibration done!");
}

void mmc_powerDown(struct i2c_dt_spec dev_i2c)
{
    i2c_reg_update_byte_dt(&dev_i2c, MMC5983MA_CONTROL_2, 0x07, 0); // clear lowest four bits
    k_msleep(20); // make sure to finish the last measurement
}

void mmc_powerUp(struct i2c_dt_spec dev_i2c, uint8_t MODR)
{
    i2c_reg_update_byte_dt(&dev_i2c, MMC5983MA_CONTROL_2, MODR, MODR); // start continuous mode
}
