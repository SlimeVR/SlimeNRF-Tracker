#include "globals.h"
#include "sys.h"

LOG_MODULE_REGISTER(sys, 4);

K_THREAD_DEFINE(led_thread_id, 512, led_thread, NULL, NULL, NULL, 6, 0, 0);

void (*extern_main_imu_suspend)(void);

void configure_system_off_WOM(const struct i2c_dt_spec imu)
{
	(*extern_main_imu_suspend)();
	// Configure WOM interrupt
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios), NRF_GPIO_PIN_SENSE_LOW);
	// Store the last quats
	for (uint8_t i = 0; i < 4; i++){
		retained.q[i] = q[i];
	}
	retained.stored_quats = true;
	// Store fusion gyro offset
	for (uint8_t i = 0; i < 3; i++){
		retained.gOff[i] = gOff[i];
	}
	retained_update();
	LOG_INF("System off (WOM)");
	// Set system off
	icm_setup_WOM(imu); // enable WOM feature
	sys_poweroff();
}

void configure_system_off_chgstat(void){
	(*extern_main_imu_suspend)();
//	// Configure chgstat interrupt
//	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL);
//	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chgstat_gpios), NRF_GPIO_PIN_PULLUP);
//	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chgstat_gpios), NRF_GPIO_PIN_SENSE_LOW);
	// Configure dock interrupt
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_PULLUP); // Still works
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_LOW);
	// Store fusion gyro offset
	for (uint8_t i = 0; i < 3; i++){
		retained.gOff[i] = gOff[i];
	}
	retained_update();
	LOG_INF("System off (chgstat)");
	// Set system off
	sys_poweroff();
}

void configure_system_off_dock(void){
	(*extern_main_imu_suspend)();
	// Configure dock interrupt
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL); // Still works
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_HIGH);
	// Store fusion gyro offset
	for (uint8_t i = 0; i < 3; i++){
		retained.gOff[i] = gOff[i];
	}
	retained_update();
	LOG_INF("System off (dock)");
	// Set system off
	sys_poweroff();
}

void power_check(void) {
	bool docked = gpio_pin_get_dt(&dock);
	int batt_mV;
	uint32_t batt_pptt = read_batt_mV(&batt_mV);
	LOG_INF("Battery %u%% (%dmV)", batt_pptt/100, batt_mV);
	if (batt_pptt == 0 && !docked) {
		(*extern_main_imu_suspend)();
		// Communicate all imus to shut down
		i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
		LOG_INF("imu reset");
		i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
		LOG_INF("mag reset");
		set_led(SYS_LED_PATTERN_OFF); // Turn off LED
		configure_system_off_chgstat();
	} else if (docked) {
		(*extern_main_imu_suspend)();
		// Communicate all imus to shut down
		i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
		LOG_INF("imu reset");
		i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
		LOG_INF("mag reset");
		set_led(SYS_LED_PATTERN_OFF); // Turn off LED
		configure_system_off_dock(); // usually charging, i would flash LED but that will drain the battery while it is charging..
	}
}

#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	sys_reboot(SYS_REBOOT_COLD); // treat like pin reset but without pin reset reason
}
#endif

enum sys_led_pattern current_led_pattern;
int led_pattern_state;

const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(ZEPHYR_USER_NODE, led_gpios, led0);
const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET_OR(LED0_NODE, {0});

void set_led(enum sys_led_pattern led_pattern)
{
	if (led_pattern == SYS_LED_PATTERN_OFF)
	{
		k_thread_suspend(led_thread_id);
		gpio_pin_set_dt(&led, 0);
	}
	else
	{
		current_led_pattern = led_pattern;
		led_pattern_state = 0;
		k_thread_resume(led_thread_id);
	}
}

void led_thread(void)
{
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	while (1)
	{
		switch (current_led_pattern)
		{
			case SYS_LED_PATTERN_ON:
				gpio_pin_set_dt(&led, 1);
				k_thread_suspend(led_thread_id);
				break;
			case SYS_LED_PATTERN_FAST:
				led_pattern_state = (led_pattern_state + 1) % 2;
				gpio_pin_set_dt(&led, led_pattern_state);
				k_msleep(500);
				break;
			case SYS_LED_PATTERN_SLOW:
				led_pattern_state = (led_pattern_state + 1) % 2;
				gpio_pin_set_dt(&led, led_pattern_state);
				k_msleep(led_pattern_state == 1 ? 100 : 900);
				break;
			case SYS_LED_PATTERN_ACTIVE:
				led_pattern_state = (led_pattern_state + 1) % 2;
				gpio_pin_set_dt(&led, led_pattern_state);
				k_msleep(led_pattern_state == 1 ? 300 : 9700);
				break;
			case SYS_LED_PATTERN_ONESHOT_POWERON:
				led_pattern_state++;
				gpio_pin_set_dt(&led, led_pattern_state % 2);
				if (led_pattern_state == 6)
					k_thread_suspend(led_thread_id);
				else
					k_msleep(200);
				break;
			case SYS_LED_PATTERN_ONESHOT_POWEROFF:
				if (led_pattern_state++ > 0 && led_pattern_state < 22)
					pwm_set_pulse_dt(&pwm_led, PWM_MSEC(22 - led_pattern_state));
				else
					gpio_pin_set_dt(&led, 0);
				if (led_pattern_state == 22)
					k_thread_suspend(led_thread_id);
				else if (led_pattern_state == 1)
					k_msleep(250);
				else
					k_msleep(50);
				break;
			default:
				k_thread_suspend(led_thread_id);
		}
	}
}
