#include "globals.h"
#include "sensor.h"
#include "battery.h"
#include "connection.h"

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

static struct nvs_fs fs;

#define NVS_PARTITION		storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define LED0_NODE DT_NODELABEL(pwm_led0)
#define CLKOUT_NODE DT_NODELABEL(pwmclock)

LOG_MODULE_REGISTER(system, LOG_LEVEL_INF);

K_THREAD_DEFINE(led_thread_id, 512, led_thread, NULL, NULL, NULL, 6, 0, 0);

#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
#define BUTTON_EXISTS true
K_THREAD_DEFINE(button_thread_id, 256, button_thread, NULL, NULL, NULL, 6, 0, 0);
#else
#pragma message "Button GPIO does not exist"
#endif

K_THREAD_DEFINE(power_thread_id, 1024, power_thread, NULL, NULL, NULL, 6, 0, 0);

// TODO: well now sys file is kinda crowded
// TODO: the gpio sense is weird, maybe the device will turn back on immediately after shutdown or after (attempting to) enter WOM
// TODO: there should be a better system of how to handle all system_off cases and all the sense pins
// TODO: just changed it make sure to test it thanks

// TODO: should the tracker start again if docking state changes?
// TODO: keep sending battery state while plugged and docked?
// TODO: on some boards there is actual power path, try to use the LED in this case
// TODO: usually charging, i would flash LED but that will drain the battery while it is charging..
// TODO: should not really shut off while plugged in

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dock_gpios)
#define DOCK_EXISTS true
static const struct gpio_dt_spec dock = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, dock_gpios);
#else
#pragma message "Dock sense GPIO does not exist"
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, chg_gpios)
#define CHG_EXISTS true
static const struct gpio_dt_spec chg = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, chg_gpios);
#else
#pragma message "Charge sense GPIO does not exist"
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, stby_gpios)
#define STBY_EXISTS true
static const struct gpio_dt_spec stby = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, stby_gpios);
#else
#pragma message "Standby sense GPIO does not exist"
#endif

static void configure_sense_pins(void)
{
	// Configure dock interrupt
#if DOCK_EXISTS
	if (dock_read())
	{
		nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL); // Still works
		nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_HIGH);
	}
	else
	{
		nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_PULLUP); // Still works
		nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_LOW);
	}
	LOG_INF("Configured dock interrupt");
#endif
	// Configure chgstat interrupt
#if CHG_EXISTS
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chg_gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chg_gpios), chg_read() ? NRF_GPIO_PIN_SENSE_HIGH : NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured chg interrupt");
#endif
#if STBY_EXISTS
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, stby_gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, stby_gpios), stby_read() ? NRF_GPIO_PIN_SENSE_HIGH : NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured stby interrupt");
#endif
	// Configure sw0 interrupt
#if BUTTON_EXISTS // Alternate button if available to use as "reset key"
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured sw0 interrupt");
#endif
}

static void configure_system_off(void)
{
	main_imu_suspend(); // TODO: when the thread is suspended, its possibly suspending in the middle of an i2c transaction and this is bad. Instead sensor should be suspended at a different time
	sensor_shutdown();
	set_led(SYS_LED_PATTERN_OFF_FORCE, 0);
	float actual_clock_rate;
	set_sensor_clock(false, 0, &actual_clock_rate);
	// Configure interrupts
	configure_sense_pins();
}

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, int0_gpios)
#define IMU_INT_EXISTS true
#else
#warning "IMU interrupt GPIO does not exist"
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dcdc_gpios)
#define DCDC_EN_EXISTS true
static const struct gpio_dt_spec dcdc_en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, dcdc_gpios);
#else
#pragma message "DCDC enable GPIO does not exist"
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, ldo_gpios)
#define LDO_EN_EXISTS true
static const struct gpio_dt_spec ldo_en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, ldo_gpios);
#else
#pragma message "LDO enable GPIO does not exist"
#endif

enum sys_regulator {
	SYS_REGULATOR_DCDC,
	SYS_REGULATOR_LDO
};

static void set_regulator(enum sys_regulator regulator)
{
#if DCDC_EN_EXISTS
	bool use_dcdc = regulator == SYS_REGULATOR_DCDC;
	if (use_dcdc)
	{
		gpio_pin_set_dt(&dcdc_en, 1);
		LOG_INF("Enabled DCDC");
	}
#endif
#if LDO_EN_EXISTS
	bool use_ldo = regulator == SYS_REGULATOR_LDO;
	gpio_pin_set_dt(&ldo_en, use_ldo);
	LOG_INF("%s", use_ldo ? "Enabled LDO" : "Disabled LDO");
#endif
#if DCDC_EN_EXISTS
	if (!use_dcdc)
	{
		gpio_pin_set_dt(&dcdc_en, 0);
		LOG_INF("Disabled DCDC");
	}
#endif
}

void sys_request_WOM() // TODO: if IMU interrupt does not exist what does the system do?
{
	LOG_INF("IMU wake up requested");
#if IMU_INT_EXISTS
	configure_system_off(); // Common subsystem shutdown and prepare sense pins
	// Configure WOM interrupt
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios), NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured IMU interrupt");
	sensor_retained_write();
#if WOM_USE_DCDC // In case DCDC is more efficient in the 10-100uA range
	set_regulator(SYS_REGULATOR_DCDC); // Make sure DCDC is selected
#else
	set_regulator(SYS_REGULATOR_LDO); // Switch to LDO
#endif
	// Set system off
	sensor_setup_WOM(); // enable WOM feature
	LOG_INF("Powering off nRF");
	sys_poweroff();
#else
	LOG_WRN("IMU interrupt GPIO does not exist");
	LOG_WRN("IMU wake up not available");
#endif
}

void sys_request_system_off(void)
{
	LOG_INF("System off requested");
	configure_system_off(); // Common subsystem shutdown and prepare sense pins
	// Clear sensor addresses
	sensor_scan_clear();
	LOG_INF("Requested sensor scan on next boot");
	sensor_retained_write();
	set_regulator(SYS_REGULATOR_LDO); // Switch to LDO
	// Set system off
	LOG_INF("Powering off nRF");
	sys_poweroff();
}

/*
LED priorities (0 is highest)
0: boot/power
1: sensor
2: esb
3: system (persist)
*/

#define SYS_LED_PATTERN_DEPTH 4
static enum sys_led_pattern led_patterns[SYS_LED_PATTERN_DEPTH] = {SYS_LED_PATTERN_OFF};
static enum sys_led_pattern current_led_pattern = SYS_LED_PATTERN_OFF;
static int led_pattern_state;

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
	if (led_pattern == current_led_pattern)
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
		k_thread_resume(led_thread_id);
	}
#else
	LOG_WRN("LED GPIO does not exist");
	return;
#endif
}

void led_thread(void)
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
			led_pattern_state = (led_pattern_state + 1) % 2;
			gpio_pin_set_dt(&led, led_pattern_state);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_ONESHOT_POWERON:
			led_pattern_state++;
			gpio_pin_set_dt(&led, led_pattern_state % 2);
			if (led_pattern_state == 6)
				set_led(SYS_LED_PATTERN_OFF, 0); // Sets highest priority to OFF, better not set ONESHOT_POWERON on another priority
			else
				k_msleep(200);
			break;
		case SYS_LED_PATTERN_ONESHOT_POWEROFF:
			if (led_pattern_state++ > 0)
				pwm_set_pulse_dt(&pwm_led, PWM_MSEC(22 - led_pattern_state));
			else
				gpio_pin_set_dt(&led, 0);
			if (led_pattern_state == 22)
				set_led(SYS_LED_PATTERN_OFF_FORCE, 0);
			else if (led_pattern_state == 1)
				k_msleep(250);
			else
				k_msleep(50);
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
		default:
			k_thread_suspend(led_thread_id);
		}
	}
}

static bool nvs_init = false;

static inline void sys_nvs_init(void)
{
	if (nvs_init)
		return;
	struct flash_pages_info info;
	fs.flash_device = NVS_PARTITION_DEVICE;
	fs.offset = NVS_PARTITION_OFFSET; // Start NVS FS here
	flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	fs.sector_size = info.size; // Sector size equal to page size
	fs.sector_count = 4U; // 4 sectors
	nvs_mount(&fs);
	nvs_init = true;
}

static int sys_retained_init(void)
{
	bool ram_retention = retained_validate(); // Check ram retention
	// All contents of NVS was stored in RAM to not need initializing NVS often
	if (!ram_retention)
	{ 
		LOG_INF("Invalidated RAM");
		sys_nvs_init();
		// read from nvs to retained
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
	return 0;
}

SYS_INIT(sys_retained_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// read from retained
uint8_t reboot_counter_read(void)
{
	return retained.reboot_counter;
}

// write to retained
void reboot_counter_write(uint8_t reboot_counter)
{
	retained.reboot_counter = reboot_counter;
	retained_update();
}

// write to retained and nvs
void sys_write(uint16_t id, void *retained_ptr, const void *data, size_t len)
{
	sys_nvs_init();
	memcpy(retained_ptr, data, len);
	nvs_write(&fs, id, data, len);
	retained_update();
}

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, clk_gpios)
#define CLK_EN_EXISTS true
static const struct gpio_dt_spec clk_en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, clk_gpios);
#endif
static const struct pwm_dt_spec clk_out = PWM_DT_SPEC_GET_OR(CLKOUT_NODE, {0});

// return 0 if clock applied, -1 if failed (because there is no clk_en or clk_out)
int set_sensor_clock(bool enable, float rate, float *actual_rate)
{
#if CLK_EN_EXISTS
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

#if BUTTON_EXISTS // Alternate button if available to use as "reset key"
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static int64_t press_time;

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
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

static struct gpio_callback button_cb_data;

static int sys_button_init(void)
{
	gpio_pin_configure_dt(&button0, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button0.pin));
	gpio_add_callback(button0.port, &button_cb_data);
	return 0;
}

SYS_INIT(sys_button_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif

bool button_read(void)
{
#if BUTTON_EXISTS // Alternate button if available to use as "reset key"
	return gpio_pin_get_dt(&button0);
#else
	return false;
#endif
}

void button_thread(void)
{
#if BUTTON_EXISTS // Alternate button if available to use as "reset key"
	while (1)
	{
		k_msleep(10);
		if (press_time != 0 && k_uptime_get() - press_time > 50 && button_read()) // Button is being pressed
			sys_reboot(SYS_REBOOT_COLD);
	}
#endif
}

static int sys_gpio_init(void)
{
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
#if DOCK_EXISTS // configure if exists
	gpio_pin_configure_dt(&dock, GPIO_INPUT);
#endif
#if CHG_EXISTS
	gpio_pin_configure_dt(&chg, GPIO_INPUT);
#endif
#if STBY_EXISTS
	gpio_pin_configure_dt(&stby, GPIO_INPUT);
#endif
#if CLK_EN_EXISTS
	gpio_pin_configure_dt(&clk_en, GPIO_OUTPUT);
#endif
#if DCDC_EN_EXISTS
	gpio_pin_configure_dt(&dcdc_en, GPIO_OUTPUT);
#endif
#if LDO_EN_EXISTS
	gpio_pin_configure_dt(&ldo_en, GPIO_OUTPUT);
#endif
	return 0;
}

SYS_INIT(sys_gpio_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

bool dock_read(void)
{
#if DOCK_EXISTS
	return gpio_pin_get_dt(&dock);
#else
	return false;
#endif
}

bool chg_read(void)
{
#if CHG_EXISTS
	return gpio_pin_get_dt(&chg);
#else
	return false;
#endif
}

bool stby_read(void)
{
#if STBY_EXISTS
	return gpio_pin_get_dt(&stby);
#else
	return false;
#endif
}

static bool plugged = false;
static bool power_init = false;

bool vin_read(void) // blocking
{
	while (!power_init)
		k_usleep(1); // wait for first battery read
	return plugged;
}

static uint32_t last_battery_pptt[16] = {10001};
static int last_battery_pptt_index = 0;
static bool battery_low = false;

void power_thread(void)
{
	while (1)
	{
		bool docked = dock_read();
		bool charging = chg_read();
		bool charged = stby_read();

		int battery_mV;
		uint32_t battery_pptt = read_batt_mV(&battery_mV);

		bool battery_available = battery_mV > 1500; // Keep working without the battery connected, otherwise it is obviously too dead to boot system
		plugged = battery_mV > 4300; // Separate detection of vin

		if (!power_init)
		{
			// log battery state once
			if (battery_available)
				LOG_INF("Battery %u%% (%dmV)", battery_pptt/100, battery_mV);
			else
				LOG_INF("Battery not available (%dmV)", battery_mV);
			set_regulator(SYS_REGULATOR_DCDC); // Switch to DCDC
			power_init = true;
		}

		if ((battery_available && battery_pptt == 0) || docked)
			sys_request_system_off();

		if (!battery_low && battery_pptt < 1000)
			battery_low = true;
		else if (battery_low && battery_pptt > 1500) // hysteresis
			battery_low = false;

		// Average battery readings across 16 samples (last reading is first sample)
		uint32_t average_battery_pptt = battery_pptt;
		for (uint8_t i = 0; i < 15; i++)
		{
			if (last_battery_pptt[i] == 10001)
				average_battery_pptt += battery_pptt / (i + 1);
			else
				average_battery_pptt += last_battery_pptt[i];
		}
		average_battery_pptt /= 16;

		// Now add the last reading to the sample array
		last_battery_pptt[last_battery_pptt_index] = battery_pptt;
		last_battery_pptt_index++;
		last_battery_pptt_index %= 15;

		// Store the average battery level with hysteresis (Effectively 100-10000 -> 1-100%)
		if (average_battery_pptt + 100 < last_battery_pptt[15]) // Lower bound -100pptt
			last_battery_pptt[15] = average_battery_pptt + 100;
		else if (average_battery_pptt > last_battery_pptt[15]) // Upper bound +0pptt
			last_battery_pptt[15] = average_battery_pptt;

		connection_update_battery(battery_available, charging || charged || plugged, last_battery_pptt[15], battery_mV);

		if (charging)
			set_led(SYS_LED_PATTERN_PULSE_PERSIST, 3);
		else if (charged)
			set_led(SYS_LED_PATTERN_ON_PERSIST, 3);
		else if (plugged)
			set_led(SYS_LED_PATTERN_PULSE_PERSIST, 3);
		else if (battery_low)
			set_led(SYS_LED_PATTERN_LONG_PERSIST, 3);
		else
			set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, 3);
//			set_led(SYS_LED_PATTERN_OFF, 3);

		if (system_off_main) // System off on extended no movement
			sys_request_WOM();

		k_msleep(100);
	}
}
