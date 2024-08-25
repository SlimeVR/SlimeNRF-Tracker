#ifndef SLIMENRF_SENSOR_SCAN
#define SLIMENRF_SENSOR_SCAN

#include <zephyr/drivers/i2c.h>

int sensor_scan(struct i2c_dt_spec i2c_dev, int dev_addr_count, const uint8_t dev_addr[], const uint8_t dev_reg[], const uint8_t dev_id[], const int dev_ids[]);

#endif