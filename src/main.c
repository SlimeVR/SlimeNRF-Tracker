#include "globals.h"
#include "sys.h"
//#include "timer.h"
#include "esb.h"
#include "sensor.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>

#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e

uint32_t* dbl_reset_mem = ((uint32_t*) DFU_DBL_RESET_MEM);

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
#if IGNORE_RESET && DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios)
	bool reset_pin_reset = false;
#else
	bool reset_pin_reset = NRF_POWER->RESETREAS & 0x01;
#endif
	NRF_POWER->RESETREAS = NRF_POWER->RESETREAS; // Clear RESETREAS

//	start_time = k_uptime_get(); // Need to get start time for imu startup delay
	set_led(SYS_LED_PATTERN_ON); // Boot LED

#if CONFIG_BUILD_OUTPUT_UF2 && !(IGNORE_RESET && DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios)) // Using Adafruit bootloader
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
	ram_range_retain(dbl_reset_mem, sizeof(dbl_reset_mem), true);
#endif

	uint8_t reboot_counter = reboot_counter_read();
	bool booting_from_shutdown = reboot_counter == 0; // 0 means from user shutdown or failed ram validation;

	if (booting_from_shutdown)
		set_led(SYS_LED_PATTERN_ONESHOT_POWERON);

	bool docked = dock_read();

	if ((reset_pin_reset || button_read()) && !docked) // Count pin resets while not docked
	{
		if (reboot_counter == 0)
			reboot_counter = 100;
		else if (reboot_counter > 200)
			reboot_counter = 200; // How did you get here
		reset_mode = reboot_counter - 100;
		reboot_counter++;
		reboot_counter_write(reboot_counter);
		LOG_INF("Reset count: %u", reboot_counter);
		k_msleep(1000); // Wait before clearing counter and continuing
		reboot_counter_write(100);
		if (!reset_pin_reset && !button_read() && reset_mode == 0) // Only need to check once, if the button is pressed again an interrupt is triggered from before
			reset_mode = -1; // Cancel reset_mode (shutdown)
	}
// 0ms or 1000ms for reboot counter

#if USER_SHUTDOWN_ENABLED
	bool charging = chg_read();
	bool charged = stby_read();
	bool plugged = vin_read();

	if (reset_mode == 0 && !booting_from_shutdown && !charging && !charged && !plugged) // Reset mode user shutdown, only if unplugged and undocked
	{
		LOG_INF("User shutdown requested");
		reboot_counter_write(0);
		set_led(SYS_LED_PATTERN_ONESHOT_POWEROFF);
		// TODO: scheduled power off
		k_msleep(1500);
		if (button_read()) // If alternate button is available and still pressed, wait for the user to stop pressing the button
		{
			set_led(SYS_LED_PATTERN_LONG);
			while (button_read())
				k_msleep(1);
			set_led(SYS_LED_PATTERN_OFF);
		}
		bool docked = dock_read();
		if (!docked) // TODO: should the tracker start again if docking state changes?
			configure_system_off_chgstat();
		else
			configure_system_off_dock(); // usually charging, i would flash LED but that will drain the battery while it is charging..
	}
// How long user shutdown take does not matter really ("0ms")
#endif

	set_led(SYS_LED_PATTERN_OFF);

	if (reset_mode == 1)
	{
		LOG_INF("IMU calibration requested");
	}

	if (reset_mode == 2) // Reset mode pairing reset
	{
		LOG_INF("Pairing reset requested");
		esb_reset_pair();
		reset_mode = 0; // Clear reset mode
	}

#if CONFIG_BUILD_OUTPUT_UF2 // Using Adafruit bootloader
	if (reset_mode == 3 || reset_mode == 4) // DFU_MAGIC_UF2_RESET, Reset mode DFU
	{
		LOG_INF("DFU requested");
		NRF_POWER->GPREGRET = 0x57;
		sys_reboot(SYS_REBOOT_COLD);
	}
#endif

	clocks_start();

	esb_pair();

	int err = esb_initialize();
	if (err)
		LOG_ERR("ESB initialization failed: %d", err);
	//timer_init();
// 1ms to start ESB

	return 0;
}
