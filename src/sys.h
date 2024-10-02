#ifndef SLIMENRF_SYS
#define SLIMENRF_SYS

#define RBT_CNT_ID 1
#define PAIRED_ID 2
#define MAIN_ACCEL_BIAS_ID 3
#define MAIN_GYRO_BIAS_ID 4
#define MAIN_MAG_BIAS_ID 5

// TODO: these patterns are kinda funky
enum sys_led_pattern {
	SYS_LED_PATTERN_OFF_FORCE, // ignores lower priority patterns

	SYS_LED_PATTERN_OFF, // yield to lower priority patterns
	SYS_LED_PATTERN_ON,
	SYS_LED_PATTERN_SHORT, // 100ms on 900ms off
	SYS_LED_PATTERN_LONG, // 500ms on 500ms off
	SYS_LED_PATTERN_ONESHOT_POWERON, // 200ms on 200ms off, 3 times
	SYS_LED_PATTERN_ONESHOT_POWEROFF, // 250ms off, 1000ms fade to off

	SYS_LED_PATTERN_ON_PERSIST, // 20% duty cycle
	SYS_LED_PATTERN_LONG_PERSIST, // 20% duty cycle, 500ms on 500ms off
	SYS_LED_PATTERN_PULSE_PERSIST, // 5000ms pulsing
	SYS_LED_PATTERN_ACTIVE_PERSIST, // 300ms on 9700ms off
};

void sys_request_WOM(void);
void sys_request_system_off(void);

void set_led(enum sys_led_pattern led_pattern, int priority);
void led_thread(void);

uint8_t reboot_counter_read(void);
void reboot_counter_write(uint8_t reboot_counter);

void sys_write(uint16_t id, void *ptr, const void *data, size_t len);

int set_sensor_clock(bool enable, float rate, float *actual_rate);

bool button_read(void);
void button_thread(void);

bool dock_read(void);
bool chg_read(void);
bool stby_read(void);
bool vin_read(void);

void power_thread(void);

#endif