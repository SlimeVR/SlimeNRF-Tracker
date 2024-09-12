#include "globals.h"
#include "sensor.h"
#include "battery.h"

#include <math.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <hal/nrf_gpio.h>

#include "sys.h"

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884f
#endif

struct nvs_fs fs;

#define NVS_PARTITION		storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define LED0_NODE DT_NODELABEL(pwm_led0)
#define CLKOUT_NODE DT_NODELABEL(pwmclock)

LOG_MODULE_REGISTER(sys, LOG_LEVEL_INF);

K_THREAD_DEFINE(led_thread_id, 512, led_thread, NULL, NULL, NULL, 6, 0, 0);

#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
K_THREAD_DEFINE(button_thread_id, 256, button_thread, NULL, NULL, NULL, 6, 0, 0);
#endif

void configure_system_off_WOM()
{
	LOG_INF("System off requested (WOM)");
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, int0_gpios)
	main_imu_suspend();
	sensor_shutdown();
	set_led(SYS_LED_PATTERN_OFF);
	set_led(SYS_LED_PATTERN_OFF_PERSIST);
	float actual_clock_rate;
	set_sensor_clock(false, 0, &actual_clock_rate);
	// Configure dock
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dock_gpios)
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL);
#endif
	// Configure sw0 interrupt
#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured sw0 interrupt");
#endif
	// Configure WOM interrupt
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios), NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured WOM interrupt");
	sensor_retained_write();
	// Set system off
	sensor_setup_WOM(); // enable WOM feature
	LOG_INF("Powering off nRF");
	sys_poweroff();
#else
	LOG_INF("WOM not available");
#endif
}

void configure_system_off_chgstat(void)
{
	LOG_INF("System off requested (chgstat)");
	main_imu_suspend();
	sensor_shutdown();
	set_led(SYS_LED_PATTERN_OFF);
	set_led(SYS_LED_PATTERN_OFF_PERSIST);
	float actual_clock_rate;
	set_sensor_clock(false, 0, &actual_clock_rate);
//	// Configure chgstat interrupt
//	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL);
//	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chgstat_gpios), NRF_GPIO_PIN_PULLUP);
//	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chgstat_gpios), NRF_GPIO_PIN_SENSE_LOW);
	// Configure dock interrupt
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dock_gpios)
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_PULLUP); // Still works
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured dock interrupt");
#endif
	// Configure sw0 interrupt
#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured sw0 interrupt");
#endif
	// Clear sensor addresses
	LOG_INF("Requested sensor scan on next boot");
	sensor_scan_clear();
	sensor_retained_write();
	// Set system off
	LOG_INF("Powering off nRF");
	sys_poweroff();
}

void configure_system_off_dock(void)
{
	LOG_INF("System off requested (dock)");
	main_imu_suspend();
	sensor_shutdown();
	set_led(SYS_LED_PATTERN_OFF);
	set_led(SYS_LED_PATTERN_OFF_PERSIST);
	float actual_clock_rate;
	set_sensor_clock(false, 0, &actual_clock_rate);
	// Configure dock interrupt
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dock_gpios)
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL); // Still works
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_HIGH);
	LOG_INF("Configured dock interrupt");
#endif
	// Configure sw0 interrupt
#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured sw0 interrupt");
#endif
	// Clear sensor addresses
	LOG_INF("Requested sensor scan on next boot");
	sensor_scan_clear();
	sensor_retained_write();
	// Set system off
	LOG_INF("Powering off nRF");
	sys_poweroff();
}

void power_check(void)
{
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dock_gpios)
	bool docked = gpio_pin_get_dt(&dock);
#else
	bool docked = false;
#endif
	int batt_mV;
	uint32_t batt_pptt = read_batt_mV(&batt_mV);
	bool battery_available = batt_mV > 1500; // Keep working without the battery connected, otherwise it is obviously too dead to boot system
	if (battery_available)
		LOG_INF("Battery %u%% (%dmV)", batt_pptt/100, batt_mV);
	else
		LOG_INF("Battery not available (%dmV)", batt_mV);
	if (battery_available && batt_pptt == 0 && !docked)
		configure_system_off_chgstat();
	else if (docked) // TODO: on some boards there is actual power path, try to use the LED in this case
		configure_system_off_dock(); // usually charging, i would flash LED but that will drain the battery while it is charging..
}

// TODO: temporary move button to main

enum sys_led_pattern current_led_pattern;
enum sys_led_pattern persistent_led_pattern = SYS_LED_PATTERN_OFF_PERSIST;
int led_pattern_state;
int led_pattern_state_persist;

const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(ZEPHYR_USER_NODE, led_gpios, led0);
const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET_OR(LED0_NODE, {0});

void set_led(enum sys_led_pattern led_pattern)
{
	if (led_pattern == current_led_pattern || led_pattern == persistent_led_pattern)
		return;
	pwm_set_pulse_dt(&pwm_led, 0);
	gpio_pin_set_dt(&led, 0);
	if (led_pattern >= SYS_LED_PATTERN_OFF_PERSIST)
	{
		persistent_led_pattern = led_pattern;
		led_pattern_state_persist = 0;
	}
	else
	{
		current_led_pattern = led_pattern;
		led_pattern_state = 0;
	}
	if (current_led_pattern == SYS_LED_PATTERN_OFF && persistent_led_pattern == SYS_LED_PATTERN_OFF_PERSIST)
		k_thread_suspend(led_thread_id);
	else
		k_thread_resume(led_thread_id);
}

void led_thread(void)
{
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	while (1)
	{
		switch (current_led_pattern != SYS_LED_PATTERN_OFF ? current_led_pattern : persistent_led_pattern)
		{
		case SYS_LED_PATTERN_ON:
		case SYS_LED_PATTERN_ON_PERSIST:
			gpio_pin_set_dt(&led, 1);
			k_thread_suspend(led_thread_id);
			break;
		case SYS_LED_PATTERN_SHORT:
			led_pattern_state = (led_pattern_state + 1) % 2;
			gpio_pin_set_dt(&led, led_pattern_state);
			k_msleep(led_pattern_state == 1 ? 100 : 900);
			break;
		case SYS_LED_PATTERN_LONG:
			led_pattern_state = (led_pattern_state + 1) % 2;
			gpio_pin_set_dt(&led, led_pattern_state);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_ONESHOT_POWERON:
			led_pattern_state++;
			gpio_pin_set_dt(&led, led_pattern_state % 2);
			if (led_pattern_state == 6)
				current_led_pattern = SYS_LED_PATTERN_OFF;
			else
				k_msleep(200);
			break;
		case SYS_LED_PATTERN_ONESHOT_POWEROFF:
			if (led_pattern_state++ > 0)
				pwm_set_pulse_dt(&pwm_led, PWM_MSEC(22 - led_pattern_state));
			else
				gpio_pin_set_dt(&led, 0);
			if (led_pattern_state == 22)
				current_led_pattern = SYS_LED_PATTERN_OFF;
			else if (led_pattern_state == 1)
				k_msleep(250);
			else
				k_msleep(50);
			break;
		case SYS_LED_PATTERN_PULSE_PERSIST:
			led_pattern_state_persist = (led_pattern_state_persist + 1) % 100;
			float led_value = sinf(led_pattern_state_persist * (M_PI / 100));
			pwm_set_pulse_dt(&pwm_led, PWM_MSEC(20 * led_value));
			k_msleep(50);
			break;
		case SYS_LED_PATTERN_ACTIVE_PERSIST:
			led_pattern_state_persist = (led_pattern_state_persist + 1) % 2;
			gpio_pin_set_dt(&led, led_pattern_state_persist);
			k_msleep(led_pattern_state_persist == 1 ? 300 : 9700);
			break;
		default:
			k_thread_suspend(led_thread_id);
		}
	}
}

bool ram_validated;
bool ram_retention;
bool nvs_init;

inline void sys_retained_init(void)
{
	if (!ram_validated)
	{
		ram_retention = retained_validate(); // Check ram retention
		ram_validated = true;
	}
}

inline void sys_nvs_init(void)
{
	if (!nvs_init)
	{
		struct flash_pages_info info;
		fs.flash_device = NVS_PARTITION_DEVICE;
		fs.offset = NVS_PARTITION_OFFSET; // Start NVS FS here
		flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
		fs.sector_size = info.size; // Sector size equal to page size
		fs.sector_count = 4U; // 4 sectors
		nvs_mount(&fs);
		nvs_init = true;
	}
}

// read from retained
uint8_t reboot_counter_read(void)
{
	sys_retained_init();
	return retained.reboot_counter;
}

// write to retained
void reboot_counter_write(uint8_t reboot_counter)
{
	sys_retained_init();
	retained.reboot_counter = reboot_counter;
	retained_update();
}

// read from nvs to retained
void sys_read(void)
{
	sys_retained_init();
	// All contents of NVS was stored in RAM to not need initializing NVS often
	if (!ram_retention)
	{ 
		LOG_INF("Invalidated RAM");
		sys_nvs_init();
		nvs_read(&fs, PAIRED_ID, &retained.paired_addr, sizeof(retained.paired_addr));
		nvs_read(&fs, MAIN_ACCEL_BIAS_ID, &retained.accelBias, sizeof(retained.accelBias));
		nvs_read(&fs, MAIN_GYRO_BIAS_ID, &retained.gyroBias, sizeof(retained.gyroBias));
		nvs_read(&fs, MAIN_MAG_BIAS_ID, &retained.magBAinv, sizeof(retained.magBAinv));
		retained_update();
		ram_retention = true;
	}
	else
	{
		LOG_INF("Validated RAM");
	}
}

// write to retained and nvs
void sys_write(uint16_t id, void *retained_ptr, const void *data, size_t len)
{
	sys_retained_init();
	sys_nvs_init();
	memcpy(retained_ptr, data, len);
	nvs_write(&fs, id, data, len);
	retained_update();
}

const struct gpio_dt_spec clk_en = GPIO_DT_SPEC_GET_OR(ZEPHYR_USER_NODE, clk_gpios, {0});
const struct pwm_dt_spec clk_out = PWM_DT_SPEC_GET_OR(CLKOUT_NODE, {0});

// return 0 if clock applied, -1 if failed (because there is no clk_en or clk_out)
int set_sensor_clock(bool enable, float rate, float *actual_rate)
{
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, clk_gpios)
	gpio_pin_set_dt(&clk_en, enable); // if enabling some external oscillator is available
//	*actual_rate = enable ? (float)NSEC_PER_SEC / clk_out.period : 0; // assume pwm period is the same as an equivalent external oscillator
	*actual_rate = enable ? 32768 : 0; // default
	return 0;
#endif
	*actual_rate = 0; // rate is 0 if there will be no clock source available
	if (!device_is_ready(clk_out.dev))
		return -1;
	int err = pwm_set_dt(&clk_out, PWM_HZ(rate), enable ? PWM_HZ(rate * 2) : 0); // if clk_out is used
	if (!err)
		*actual_rate = enable ? rate : 0; // the system probably could provide the correct rate
	return err;
}


#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

int64_t press_time;

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (button_read())
	{
		press_time = k_uptime_get();
	}
	else
	{
		if (press_time != 0 && k_uptime_get() - press_time > 50) // Debounce
			sys_reboot(SYS_REBOOT_COLD); // treat like pin reset but without pin reset reason
		press_time = 0;
	}
}
#endif

bool button_init;
struct gpio_callback button_cb_data;

void sys_button_init(void)
{
#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
	if (!button_init)
	{
		gpio_pin_configure_dt(&button0, GPIO_INPUT);
		gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_BOTH);
		gpio_init_callback(&button_cb_data, button_pressed, BIT(button0.pin));
		gpio_add_callback(button0.port, &button_cb_data);
		button_init = true;
	}
#endif
}

bool button_read(void)
{
#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
	sys_button_init();
	return gpio_pin_get_dt(&button0);
#else
	return false;
#endif
}

void button_thread(void)
{
#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
	sys_button_init();
	while (1)
	{
		k_msleep(10);
		if (press_time != 0 && k_uptime_get() - press_time > 50 && button_read()) // Button is being pressed
			sys_reboot(SYS_REBOOT_COLD);
	}
#endif
}
