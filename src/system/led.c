#include "..\globals.h"
#include "..\util.h"

#include <math.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include "led.h"

LOG_MODULE_REGISTER(led, LOG_LEVEL_INF);

static void led_thread(void);
K_THREAD_DEFINE(led_thread_id, 512, led_thread, NULL, NULL, NULL, 6, 0, 0);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define LED0_NODE DT_NODELABEL(pwm_led0)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, led_gpios)
#define LED_EXISTS true
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_gpios);
#elif DT_NODE_EXISTS(DT_ALIAS(led0))
#define LED_EXISTS true
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#else
#warning "LED GPIO does not exist"
static const struct gpio_dt_spec led = {0};
#endif
#if DT_NODE_EXISTS(LED0_NODE)
#define PWM_LED_EXISTS true
static const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET(LED0_NODE);
#else
#warning "PWM LED node does not exist"
static const struct pwm_dt_spec pwm_led = {0};
#endif

static enum sys_led_pattern led_patterns[SYS_LED_PATTERN_DEPTH] = {[0 ... (SYS_LED_PATTERN_DEPTH - 1)] = SYS_LED_PATTERN_OFF};
static enum sys_led_pattern current_led_pattern;
static int led_pattern_state;

void set_led(enum sys_led_pattern led_pattern, int priority)
{
#if LED_EXISTS
	led_patterns[priority] = led_pattern;
	for (int i = 0; i < SYS_LED_PATTERN_DEPTH; i++)
	{
		if (led_patterns[i] == SYS_LED_PATTERN_OFF)
			continue;
		led_pattern = led_patterns[i];
		break;
	}
	if (led_pattern == current_led_pattern && led_pattern > SYS_LED_PATTERN_OFF)
		return;
	current_led_pattern = led_pattern;
	led_pattern_state = 0;
	if (current_led_pattern <= SYS_LED_PATTERN_OFF)
	{
		pwm_set_pulse_dt(&pwm_led, 0);
		gpio_pin_set_dt(&led, 0);
		k_thread_suspend(led_thread_id);
	}
	else
	{
		k_thread_suspend(led_thread_id);
		k_thread_resume(led_thread_id);
	}
#else
	LOG_WRN("LED GPIO does not exist");
	return;
#endif
}

static void led_thread(void)
{
#if !LED_EXISTS
	LOG_WRN("LED GPIO does not exist");
	return;
#endif
	while (1)
	{
		pwm_set_pulse_dt(&pwm_led, 0);
		gpio_pin_set_dt(&led, 0);
		switch (current_led_pattern)
		{
		case SYS_LED_PATTERN_ON:
			gpio_pin_set_dt(&led, 1);
			k_thread_suspend(led_thread_id);
			break;
		case SYS_LED_PATTERN_SHORT:
			led_pattern_state = (led_pattern_state + 1) % 2;
			gpio_pin_set_dt(&led, led_pattern_state);
			k_msleep(led_pattern_state == 1 ? 100 : 900);
			break;
		case SYS_LED_PATTERN_LONG:
		case SYS_LED_PATTERN_ERROR_D:
			led_pattern_state = (led_pattern_state + 1) % 2;
			gpio_pin_set_dt(&led, led_pattern_state);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_FLASH:
			led_pattern_state = (led_pattern_state + 1) % 2;
			gpio_pin_set_dt(&led, led_pattern_state);
			k_msleep(200);
			break;

		case SYS_LED_PATTERN_ONESHOT_POWERON:
			led_pattern_state++;
			gpio_pin_set_dt(&led, !(led_pattern_state % 2));
			if (led_pattern_state == 7)
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST); // Sets highest priority to OFF, better not set ONESHOT_POWERON on another priority
			else
				k_msleep(200);
			break;
		case SYS_LED_PATTERN_ONESHOT_POWEROFF:
			if (led_pattern_state++ > 0)
				pwm_set_pulse_dt(&pwm_led, PWM_MSEC(22 - led_pattern_state));
			else
				gpio_pin_set_dt(&led, 0);
			if (led_pattern_state == 22)
				set_led(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_HIGHEST);
			else if (led_pattern_state == 1)
				k_msleep(250);
			else
				k_msleep(50);
			break;
		case SYS_LED_PATTERN_ONESHOT_PROGRESS:
			led_pattern_state++;
			gpio_pin_set_dt(&led, !(led_pattern_state % 2));
			if (led_pattern_state == 5)
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST); // Sets highest priority to OFF, better not set SYS_LED_PATTERN_ONESHOT_PROGRESS on another priority
			else
				k_msleep(200);
			break;
		case SYS_LED_PATTERN_ONESHOT_COMPLETE:
			led_pattern_state++;
			gpio_pin_set_dt(&led, !(led_pattern_state % 2));
			if (led_pattern_state == 9)
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST); // Sets highest priority to OFF, better not set SYS_LED_PATTERN_ONESHOT_COMPLETE on another priority
			else
				k_msleep(200);
			break;

		case SYS_LED_PATTERN_ON_PERSIST:
			pwm_set_pulse_dt(&pwm_led, PWM_MSEC(4)); // 20% duty cycle, should look like ~50% brightness
			k_thread_suspend(led_thread_id);
			break;
		case SYS_LED_PATTERN_LONG_PERSIST:
			led_pattern_state = (led_pattern_state + 1) % 2;
			pwm_set_pulse_dt(&pwm_led, led_pattern_state ? PWM_MSEC(4) : 0); // 20% duty cycle, should look like ~50% brightness
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_PULSE_PERSIST:
			led_pattern_state = (led_pattern_state + 1) % 100;
			float led_value = sinf(led_pattern_state * (M_PI / 100));
			pwm_set_pulse_dt(&pwm_led, PWM_MSEC(20 * led_value));
			k_msleep(50);
			break;
		case SYS_LED_PATTERN_ACTIVE_PERSIST: // off duration first because the device may turn on multiple times rapidly and waste battery power
			led_pattern_state = (led_pattern_state + 1) % 2;
			gpio_pin_set_dt(&led, !led_pattern_state);
			k_msleep(led_pattern_state ? 9700 : 300);
			break;

		case SYS_LED_PATTERN_ERROR_A: // TODO: should this use 20% duty cycle?
			led_pattern_state = (led_pattern_state + 1) % 10;
			gpio_pin_set_dt(&led, led_pattern_state < 4 && led_pattern_state % 2);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_ERROR_B:
			led_pattern_state = (led_pattern_state + 1) % 10;
			gpio_pin_set_dt(&led, led_pattern_state < 6 && led_pattern_state % 2);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_ERROR_C:
			led_pattern_state = (led_pattern_state + 1) % 10;
			gpio_pin_set_dt(&led, led_pattern_state < 8 && led_pattern_state % 2);
			k_msleep(500);
			break;

		default:
			k_thread_suspend(led_thread_id);
		}
	}
}

static int led_gpio_init(void)
{
#if LED_EXISTS
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
#endif
	return 0;
}

SYS_INIT(led_gpio_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
