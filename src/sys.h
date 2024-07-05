#ifndef SLIMENRF_SYS
#define SLIMENRF_SYS

#include "globals.h"

enum sys_led_pattern {
    SYS_LED_PATTERN_OFF,
    SOLID_ON,
};

extern void (*extern_main_imu_suspend)(void);
void configure_system_off_WOM(const struct i2c_dt_spec imu);
void configure_system_off_chgstat(void);
void configure_system_off_dock(void);
void power_check(void);
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

void set_led(enum sys_led_pattern led_pattern);
void led_thread(void);

#endif