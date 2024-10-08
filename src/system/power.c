#include "..\globals.h"
#include "..\sensor.h"
#include "battery.h"
#include "..\connection.h"
#include "..\system.h"
#include "led.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/reboot.h>
#include <hal/nrf_gpio.h>

#include "power.h"

enum sys_regulator {
	SYS_REGULATOR_DCDC,
	SYS_REGULATOR_LDO
};

static uint32_t last_battery_pptt[16] = {[0 ... 15] = 10001};
static int last_battery_pptt_index = 0;
static bool battery_low = false;

static bool plugged = false;
static bool power_init = false;

LOG_MODULE_REGISTER(power, LOG_LEVEL_INF);

static void power_thread(void);
K_THREAD_DEFINE(power_thread_id, 1024, power_thread, NULL, NULL, NULL, 6, 0, 0);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

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

// TODO: the gpio sense is weird, maybe the device will turn back on immediately after shutdown or after (attempting to) enter WOM
// TODO: there should be a better system of how to handle all system_off cases and all the sense pins
// TODO: just changed it make sure to test it thanks

// TODO: should the tracker start again if docking state changes?
// TODO: keep sending battery state while plugged and docked?
// TODO: on some boards there is actual power path, try to use the LED in this case
// TODO: usually charging, i would flash LED but that will drain the battery while it is charging..
// TODO: should not really shut off while plugged in

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
	// TODO: not calling suspend here, because sensor can call it and stop the system from shutting down since it suspended itself
//	main_imu_suspend(); // TODO: when the thread is suspended, its possibly suspending in the middle of an i2c transaction and this is bad. Instead sensor should be suspended at a different time
	sensor_shutdown();
	set_led(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_BOOT);
	float actual_clock_rate;
	set_sensor_clock(false, 0, &actual_clock_rate);
	// Configure interrupts
	configure_sense_pins();
}

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
	LOG_INF("Configured IMU wake up");
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
	main_imu_suspend(); // TODO: should be a common shutdown step
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

bool vin_read(void) // blocking
{
	while (!power_init)
		k_usleep(1); // wait for first battery read
	return plugged;
}

static void power_thread(void)
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
			set_led(SYS_LED_PATTERN_PULSE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
		else if (charged)
			set_led(SYS_LED_PATTERN_ON_PERSIST, SYS_LED_PRIORITY_SYSTEM);
		else if (plugged)
			set_led(SYS_LED_PATTERN_PULSE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
		else if (battery_low)
			set_led(SYS_LED_PATTERN_LONG_PERSIST, SYS_LED_PRIORITY_SYSTEM);
		else
			set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
//			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SYSTEM);

		k_msleep(100);
	}
}
