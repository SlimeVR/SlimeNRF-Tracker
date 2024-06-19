#ifndef SLIMENRF_SYS
#define SLIMENRF_SYS

#include "globals.h"

void configure_system_off_WOM(const struct i2c_dt_spec imu);
void configure_system_off_chgstat(void);
void configure_system_off_dock(void);
void power_check(void);
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

#endif