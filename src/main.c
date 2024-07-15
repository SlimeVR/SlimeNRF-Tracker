#include "globals.h"
#include "sys.h"
//#include "timer.h"
//#include "util.h"
#include "esb.h"
#include "sensor.h"
#include "battery.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>

#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e

uint32_t* dbl_reset_mem = ((uint32_t*) DFU_DBL_RESET_MEM);

LOG_MODULE_REGISTER(main, 4);

int main(void)
{
	int32_t reset_reason = NRF_POWER->RESETREAS;
	NRF_POWER->RESETREAS = NRF_POWER->RESETREAS; // Clear RESETREAS
	uint8_t reboot_counter = 0;
	bool booting_from_shutdown = false;

	gpio_pin_configure_dt(&dock, GPIO_INPUT);

	power_check(); // check the battery and dock first before continuing (4ms delta to read from ADC)

//	start_time = k_uptime_get(); // Need to get start time for imu startup delta
	set_led(SYS_LED_PATTERN_ON); // Boot LED

#if CONFIG_BOARD_SUPERMINI // Using Adafruit bootloader
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
#endif

#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
	const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
	gpio_pin_configure_dt(&button0, GPIO_INPUT);
	reset_reason |= gpio_pin_get_dt(&button0);
	gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button0.pin));
	gpio_add_callback(button0.port, &button_cb_data);
#endif

	if (reset_reason & 0x01) { // Count pin resets
		reboot_counter = reboot_counter_read();
		if (reboot_counter > 200) reboot_counter = 200; // How did you get here
		booting_from_shutdown = reboot_counter == 0 ? true : false; // 0 means from user shutdown or failed ram validation
		if (reboot_counter == 0) reboot_counter = 100;
		reset_mode = reboot_counter - 100;
		reboot_counter++;
		reboot_counter_write(reboot_counter);
		LOG_INF("Reset Count: %u", reboot_counter);
		if (booting_from_shutdown)
			set_led(SYS_LED_PATTERN_ONESHOT_POWERON);
		k_msleep(1000); // Wait before clearing counter and continuing
		reboot_counter = 100;
		reboot_counter_write(reboot_counter);
	}
// 0ms or 1000ms delta for reboot counter

#if USER_SHUTDOWN_ENABLED
	if (reset_mode == 0 && !booting_from_shutdown) { // Reset mode user shutdown
		reboot_counter = 0;
		reboot_counter_write(reboot_counter);
		set_led(SYS_LED_PATTERN_ONESHOT_POWEROFF);
		// TODO: scheduled power off
		k_msleep(1250);
		bool docked = gpio_pin_get_dt(&dock);
		if (!docked) // TODO: should the tracker start again if docking state changes?
			configure_system_off_chgstat();
		else
			configure_system_off_dock(); // usually charging, i would flash LED but that will drain the battery while it is charging..
	}
// How long user shutdown take does not matter really ("0ms")
#endif

	sys_read();
// 5-6ms delta to initialize NVS (only done when needed)

	set_led(SYS_LED_PATTERN_OFF);

// TODO: if reset counter is 0 but reset reason was 1 then perform imu scanning (pressed reset once)
	if (reset_mode == 0) { // Reset mode scan imus
	}
// ?? delta

	if (reset_mode == 3) { // Reset mode pairing reset
		LOG_INF("Enter pairing reset");
		esb_reset_pair();
		reset_mode = 0; // Clear reset mode
	}

#if CONFIG_BOARD_SUPERMINI // Using Adafruit bootloader
	if (reset_mode >= 4) { // DFU_MAGIC_UF2_RESET, Reset mode DFU
		NRF_POWER->GPREGRET = 0x57;
		sys_reboot(SYS_REBOOT_COLD);
	}
#endif

	clocks_start();

	esb_pair();

	sensor_retained_read();
// 0ms delta to read calibration and configure pins

	esb_initialize();
	tx_payload.noack = false;
	//timer_init();
// 1ms delta to start ESB

	while (1)
	{
		// Get start time
		int64_t time_begin = k_uptime_get();

		//charging = gpio_pin_get_dt(&chgstat); // TODO: Charging detect doesn't work (hardware issue)
		bool docked = gpio_pin_get_dt(&dock);

		int batt_mV;
		batt_pptt = read_batt_mV(&batt_mV);

		if (batt_pptt == 0 && !docked)
			configure_system_off_chgstat();
		last_batt_pptt[last_batt_pptt_i] = batt_pptt;
		last_batt_pptt_i++;
		last_batt_pptt_i %= 15;
		for (uint8_t i = 0; i < 15; i++) {  // Average battery readings across 16 samples
			if (last_batt_pptt[i] == 10001) {
				batt_pptt += batt_pptt / (i + 1);
			} else {
				batt_pptt += last_batt_pptt[i];
			}
		}
		batt_pptt /= 16;
		if (batt_pptt + 100 < last_batt_pptt[15]) {last_batt_pptt[15] = batt_pptt + 100;} // Lower bound -100pptt
		else if (batt_pptt > last_batt_pptt[15]) {last_batt_pptt[15] = batt_pptt;} // Upper bound +0pptt
		else {batt_pptt = last_batt_pptt[15];} // Effectively 100-10000 -> 1-100%

		// format for packet send
		batt = batt_pptt / 100;
		if (batt < 1) {batt = 1;} // Clamp to 1%
		batt_mV /= 10;
		batt_mV -= 245;
		if (batt_mV < 0) {batt_v = 0;} // Very dead but it is what it is
		else if (batt_mV > 255) {batt_v = 255;}
		else {batt_v = batt_mV;} // 0-255 -> 2.45-5.00V

//		pwm_set_pulse_dt(&pwm_led, 0);

		if (docked) // TODO: keep sending battery state while plugged and docked?
		// TODO: move to interrupts? (Then you do not need to do the above)
			configure_system_off_dock();

		if (system_off_main) // System off on extended no movement
			configure_system_off_WOM();

		reconfig = last_powerstate != powerstate ? true : false;
		last_powerstate = powerstate;
		main_data = false;

		wait_for_threads(); // TODO:
		main_imu_wakeup();
		threads_running = true;

		// Get time elapsed and sleep/yield until next tick
		int64_t time_delta = k_uptime_get() - time_begin;
		led_clock_offset += time_delta;
		if (time_delta > tickrate)
		{
			k_yield();
		}
		else
		{
			k_msleep(tickrate - time_delta);
		}
	}
}
