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

static struct nvs_fs fs;

#define NVS_PARTITION		storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define LED0_NODE DT_NODELABEL(pwm_led0)
#define CLKOUT_NODE DT_NODELABEL(pwmclock)

LOG_MODULE_REGISTER(system, LOG_LEVEL_INF);

K_THREAD_DEFINE(led_thread_id, 512, led_thread, NULL, NULL, NULL, 6, 0, 0);

#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
K_THREAD_DEFINE(button_thread_id, 256, button_thread, NULL, NULL, NULL, 6, 0, 0);
#endif

K_THREAD_DEFINE(power_thread_id, 1024, power_thread, NULL, NULL, NULL, 6, 0, 0);

// TODO: well now sys file is kinda crowded
// TODO: the gpio sense is weird, maybe the device will turn back on immediately after shutdown or after (attempting to) enter WOM
// there should be a better system of how to handle all system_off cases and all the sense pins

// TODO: configuring system off should be consolidated
static void configure_sense_pins(void)
{
	// Configure chgstat interrupt
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, chg_gpios)
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chg_gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chg_gpios), NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured chg interrupt");
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, stby_gpios)
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, stby_gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, stby_gpios), NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured stby interrupt");
#endif
	// Configure dock interrupt
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dock_gpios)
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
	// Configure sw0 interrupt
#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured sw0 interrupt");
#endif
}

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dcdc_gpios)
static const struct gpio_dt_spec dcdc_en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, dcdc_gpios);
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, ldo_gpios)
static const struct gpio_dt_spec ldo_en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, ldo_gpios);
#endif

void configure_system_off_WOM() // TODO: should not really shut off while plugged in
{
	LOG_INF("System off requested (WOM)");
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, int0_gpios)
	main_imu_suspend(); // TODO: when the thread is suspended, its possibly suspending in the middle of an i2c transaction and this is bad. Instead sensor should be suspended at a different time
	sensor_shutdown();
	set_led(SYS_LED_PATTERN_OFF_FORCE);
	float actual_clock_rate;
	set_sensor_clock(false, 0, &actual_clock_rate);
	// Configure interrupts
	configure_sense_pins();
	// Configure WOM interrupt
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios), NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured WOM interrupt");
	sensor_retained_write();
#if WOM_USE_DCDC // In case DCDC is more efficient in the 10-100uA range
	// Make sure DCDC is selected
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dcdc_gpios)
	gpio_pin_set_dt(&dcdc_en, 1);
	LOG_INF("Enabled DCDC");
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, ldo_gpios)
	gpio_pin_set_dt(&ldo_en, 0);
	LOG_INF("Disabled LDO");
#endif
#else
	// Switch to LDO
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, ldo_gpios)
	gpio_pin_set_dt(&ldo_en, 1);
	LOG_INF("Enabled LDO");
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dcdc_gpios)
	gpio_pin_set_dt(&dcdc_en, 0);
	LOG_INF("Disabled DCDC");
#endif
#endif
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
	set_led(SYS_LED_PATTERN_OFF_FORCE);
	float actual_clock_rate;
	set_sensor_clock(false, 0, &actual_clock_rate);
	// Configure interrupts
	configure_sense_pins();
	// Clear sensor addresses
	sensor_scan_clear();
	LOG_INF("Requested sensor scan on next boot");
	sensor_retained_write();
	// Switch to LDO
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, ldo_gpios)
	gpio_pin_set_dt(&ldo_en, 1);
	LOG_INF("Enabled LDO");
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dcdc_gpios)
	gpio_pin_set_dt(&dcdc_en, 0);
	LOG_INF("Disabled DCDC");
#endif
	// Set system off
	LOG_INF("Powering off nRF");
	sys_poweroff();
}

void configure_system_off_dock(void)
{
	LOG_INF("System off requested (dock)");
	main_imu_suspend();
	sensor_shutdown();
	set_led(SYS_LED_PATTERN_OFF_FORCE);
	float actual_clock_rate;
	set_sensor_clock(false, 0, &actual_clock_rate);
	// Configure interrupts
	configure_sense_pins();
	// Clear sensor addresses
	sensor_scan_clear();
	LOG_INF("Requested sensor scan on next boot");
	sensor_retained_write();
	// Switch to LDO
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, ldo_gpios)
	gpio_pin_set_dt(&ldo_en, 1);
	LOG_INF("Enabled LDO");
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dcdc_gpios)
	gpio_pin_set_dt(&dcdc_en, 0);
	LOG_INF("Disabled DCDC");
#endif
	// Set system off
	LOG_INF("Powering off nRF");
	sys_poweroff();
}

static enum sys_led_pattern current_led_pattern = SYS_LED_PATTERN_OFF;
static enum sys_led_pattern persistent_led_pattern = SYS_LED_PATTERN_OFF_PERSIST;
static int led_pattern_state;
static int led_pattern_state_persist;

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, led_gpios)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_gpios);
#else
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
#endif
static const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET_OR(LED0_NODE, {0});

void set_led(enum sys_led_pattern led_pattern)
{
	if (led_pattern == current_led_pattern || led_pattern == persistent_led_pattern)
		return;
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
	if (current_led_pattern == SYS_LED_PATTERN_OFF_FORCE || (current_led_pattern == SYS_LED_PATTERN_OFF && persistent_led_pattern == SYS_LED_PATTERN_OFF_PERSIST))
	{
		pwm_set_pulse_dt(&pwm_led, 0);
		gpio_pin_set_dt(&led, 0);
		k_thread_suspend(led_thread_id);
	}
	else
	{
		k_thread_resume(led_thread_id);
	}
}

void led_thread(void)
{
	while (1)
	{
		pwm_set_pulse_dt(&pwm_led, 0);
		gpio_pin_set_dt(&led, 0);
		switch (current_led_pattern != SYS_LED_PATTERN_OFF ? current_led_pattern : persistent_led_pattern)
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
				set_led(SYS_LED_PATTERN_OFF);
			else
				k_msleep(200);
			break;
		case SYS_LED_PATTERN_ONESHOT_POWEROFF:
			if (led_pattern_state++ > 0)
				pwm_set_pulse_dt(&pwm_led, PWM_MSEC(22 - led_pattern_state));
			else
				gpio_pin_set_dt(&led, 0);
			if (led_pattern_state == 22)
				set_led(SYS_LED_PATTERN_OFF_FORCE);
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
			led_pattern_state_persist = (led_pattern_state_persist + 1) % 2;
			pwm_set_pulse_dt(&pwm_led, led_pattern_state_persist == 1 ? PWM_MSEC(4) : 0); // 20% duty cycle, should look like ~50% brightness
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_PULSE_PERSIST:
			led_pattern_state_persist = (led_pattern_state_persist + 1) % 100;
			float led_value = sinf(led_pattern_state_persist * (M_PI / 100));
			pwm_set_pulse_dt(&pwm_led, PWM_MSEC(20 * led_value));
			k_msleep(50);
			break;
		case SYS_LED_PATTERN_ACTIVE_PERSIST: // off duration first because the device may turn on multiple times rapidly and waste battery power
			led_pattern_state_persist = (led_pattern_state_persist + 1) % 2;
			gpio_pin_set_dt(&led, !led_pattern_state_persist);
			k_msleep(led_pattern_state_persist == 1 ? 9700 : 300);
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
static const struct gpio_dt_spec clk_en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, clk_gpios);
#endif
static const struct pwm_dt_spec clk_out = PWM_DT_SPEC_GET_OR(CLKOUT_NODE, {0});

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
#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
	return gpio_pin_get_dt(&button0);
#else
	return false;
#endif
}

void button_thread(void)
{
#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
	while (1)
	{
		k_msleep(10);
		if (press_time != 0 && k_uptime_get() - press_time > 50 && button_read()) // Button is being pressed
			sys_reboot(SYS_REBOOT_COLD);
	}
#endif
}

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dock_gpios)
static const struct gpio_dt_spec dock = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, dock_gpios);
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, chg_gpios)
static const struct gpio_dt_spec chg = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, chg_gpios);
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, stby_gpios)
static const struct gpio_dt_spec stby = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, stby_gpios);
#endif

static int sys_gpio_init(void)
{
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dock_gpios) // configure if exists
	gpio_pin_configure_dt(&dock, GPIO_INPUT);
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, chg_gpios)
	gpio_pin_configure_dt(&chg, GPIO_INPUT);
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, stby_gpios)
	gpio_pin_configure_dt(&stby, GPIO_INPUT);
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, clk_gpios)
	gpio_pin_configure_dt(&clk_en, GPIO_OUTPUT);
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dcdc_gpios)
	gpio_pin_configure_dt(&dcdc_en, GPIO_OUTPUT);
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, ldo_gpios)
	gpio_pin_configure_dt(&ldo_en, GPIO_OUTPUT);
#endif
	return 0;
}

SYS_INIT(sys_gpio_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

bool dock_read(void)
{
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dock_gpios)
	return gpio_pin_get_dt(&dock);
#else
	return false;
#endif
}

bool chg_read(void)
{
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, chg_gpios)
	return gpio_pin_get_dt(&chg);
#else
	return false;
#endif
}

bool stby_read(void)
{
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, stby_gpios)
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

static unsigned int last_batt_pptt[16] = {10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001};
static int8_t last_batt_pptt_i = 0;

void power_thread(void)
{
	while (1)
	{
		bool docked = dock_read();
		bool charging = chg_read();
		bool charged = stby_read();

		int batt_mV;
		uint32_t batt_pptt = read_batt_mV(&batt_mV);

		bool battery_available = batt_mV > 1500; // Keep working without the battery connected, otherwise it is obviously too dead to boot system
		plugged = batt_mV > 4300; // Separate detection of vin

		if (!power_init)
		{
			// log battery state once
			if (battery_available)
				LOG_INF("Battery %u%% (%dmV)", batt_pptt/100, batt_mV);
			else
				LOG_INF("Battery not available (%dmV)", batt_mV);
			// Switch to DCDC
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dcdc_gpios)
			gpio_pin_set_dt(&dcdc_en, 1);
			LOG_INF("Enabled DCDC");
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, ldo_gpios)
			gpio_pin_set_dt(&ldo_en, 0);
			LOG_INF("Disabled LDO");
#endif
			power_init = true;
		}

		if (battery_available && batt_pptt == 0 && !docked)
			configure_system_off_chgstat();
		else if (docked) // TODO: keep sending battery state while plugged and docked?
		// TODO: move to interrupts? (Then you do not need to do the above)
		// TODO: on some boards there is actual power path, try to use the LED in this case
			configure_system_off_dock(); // usually charging, i would flash LED but that will drain the battery while it is charging..

		last_batt_pptt[last_batt_pptt_i] = batt_pptt;
		last_batt_pptt_i++;
		last_batt_pptt_i %= 15;
		for (uint8_t i = 0; i < 15; i++)
		{ // Average battery readings across 16 samples
			if (last_batt_pptt[i] == 10001)
				batt_pptt += batt_pptt / (i + 1);
			else
				batt_pptt += last_batt_pptt[i];
		}
		batt_pptt /= 16;
		if (batt_pptt + 100 < last_batt_pptt[15]) // Lower bound -100pptt
			last_batt_pptt[15] = batt_pptt + 100;
		else if (batt_pptt > last_batt_pptt[15]) // Upper bound +0pptt
			last_batt_pptt[15] = batt_pptt;
		else // Effectively 100-10000 -> 1-100%
			batt_pptt = last_batt_pptt[15];

		// format for packet send
		batt = batt_pptt / 100;
		if (battery_available && batt < 1) // Clamp to 1% (because server sees 0% as "no battery")
			batt = 1;
		batt_mV /= 10;
		batt_mV -= 245;
		if (batt_mV < 0) // Very dead but it is what it is
			batt_v = 0;
		else if (batt_mV > 255)
			batt_v = 255;
		else
			batt_v = batt_mV; // 0-255 -> 2.45-5.00V

		if (charging)
			set_led(SYS_LED_PATTERN_PULSE_PERSIST);
		else if (charged)
			set_led(SYS_LED_PATTERN_ON_PERSIST);
		else if (plugged)
			set_led(SYS_LED_PATTERN_PULSE_PERSIST);
		else if (batt < 10)
			set_led(SYS_LED_PATTERN_LONG_PERSIST);
		else
			set_led(SYS_LED_PATTERN_ACTIVE_PERSIST);
//		else
//			set_led(SYS_LED_PATTERN_OFF_PERSIST);

		if (system_off_main) // System off on extended no movement
			configure_system_off_WOM();

		k_msleep(100);
	}
}
