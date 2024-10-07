#ifndef SLIMENRF_SYSTEM_LED
#define SLIMENRF_SYSTEM_LED

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

void set_led(enum sys_led_pattern led_pattern, int priority);

#endif