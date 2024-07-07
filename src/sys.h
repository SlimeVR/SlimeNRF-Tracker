#ifndef SLIMENRF_SYS
#define SLIMENRF_SYS

#include "globals.h"

// TODO: these patterns are kinda funky
enum sys_led_pattern {
    SYS_LED_PATTERN_OFF,
    SYS_LED_PATTERN_ON,
    SYS_LED_PATTERN_SHORT, // 100ms on 900ms off
    SYS_LED_PATTERN_LONG, // 500ms on 500ms off
    SYS_LED_PATTERN_ACTIVE, // 300ms on 9700ms off
    SYS_LED_PATTERN_ONESHOT_POWERON, // 200ms on 200ms off, 3 times
    SYS_LED_PATTERN_ONESHOT_POWEROFF, // 250ms off, 1000ms fade to off
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