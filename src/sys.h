#ifndef SLIMENRF_SYS
#define SLIMENRF_SYS

#define RBT_CNT_ID 1
#define PAIRED_ID 2
#define MAIN_ACCEL_BIAS_ID 3
#define MAIN_GYRO_BIAS_ID 4
#define MAIN_MAG_BIAS_ID 5

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

void configure_system_off_WOM(void);
void configure_system_off_chgstat(void);
void configure_system_off_dock(void);
void power_check(void);
// TODO: temporary move button to main
//void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

void set_led(enum sys_led_pattern led_pattern);
void led_thread(void);

void sys_retained_init(void);
void sys_nvs_init(void);

uint8_t reboot_counter_read(void);
void reboot_counter_write(uint8_t reboot_counter);

void sys_read(void);
void sys_write(uint16_t id, void *ptr, const void *data, size_t len);

int set_sensor_clock(bool enable, float rate, float *actual_rate);

void sys_button_init(void);

bool button_read(void);
void button_thread(void);

#endif