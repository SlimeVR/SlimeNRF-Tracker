#ifndef SLIMENRF_SYSTEM_LED
#define SLIMENRF_SYSTEM_LED

/*
LED priorities (0 is highest)
0: boot/power
1: sensor
2: connection (esb)
3: status
4: system (persist)
*/

#define SYS_LED_PRIORITY_HIGHEST 0
#define SYS_LED_PRIORITY_BOOT 0
#define SYS_LED_PRIORITY_SENSOR 1
#define SYS_LED_PRIORITY_CONNECTION 2
#define SYS_LED_PRIORITY_STATUS 3
#define SYS_LED_PRIORITY_SYSTEM 4
#define SYS_LED_PATTERN_DEPTH 5

// TODO: these patterns are kinda funky
enum sys_led_pattern {
	SYS_LED_PATTERN_OFF_FORCE, // ignores lower priority patterns

	SYS_LED_PATTERN_OFF, // yield to lower priority patterns
	SYS_LED_PATTERN_ON,
	SYS_LED_PATTERN_SHORT, // 100ms on 900ms off
	SYS_LED_PATTERN_LONG, // 500ms on 500ms off

	SYS_LED_PATTERN_ONESHOT_POWERON, // 200ms on 200ms off, 3 times
	SYS_LED_PATTERN_ONESHOT_POWEROFF, // 250ms off, 1000ms fade to off
	SYS_LED_PATTERN_ONESHOT_PAIRED, // 200ms on 200ms off, 4 times

	SYS_LED_PATTERN_ON_PERSIST, // 20% duty cycle
	SYS_LED_PATTERN_LONG_PERSIST, // 20% duty cycle, 500ms on 500ms off
	SYS_LED_PATTERN_PULSE_PERSIST, // 5000ms pulsing
	SYS_LED_PATTERN_ACTIVE_PERSIST, // 300ms on 9700ms off

	SYS_LED_PATTERN_ERROR_A, // 500ms on 500ms off, 2 times, every 5000ms
	SYS_LED_PATTERN_ERROR_B, // 500ms on 500ms off, 3 times, every 5000ms
	SYS_LED_PATTERN_ERROR_C, // 500ms on 500ms off, 4 times, every 5000ms
	SYS_LED_PATTERN_ERROR_D, // 500ms on 500ms off (same as SYS_LED_PATTERN_LONG)
};

void set_led(enum sys_led_pattern led_pattern, int priority);

#endif