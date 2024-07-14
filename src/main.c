#include "globals.h"
#include "sys.h"
//#include "timer.h"
//#include "util.h"
#include "esb.h"
#include "sensor.h"

LOG_MODULE_REGISTER(main, 4);

int main(void)
{
	extern_main_imu_suspend = &main_imu_suspend;
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
		if (!docked) { // TODO: should the tracker start again if docking state changes?
			sensor_shutdown();
			set_led(SYS_LED_PATTERN_OFF); // redundant
			configure_system_off_chgstat();
		} else {
			sensor_shutdown();
			set_led(SYS_LED_PATTERN_OFF); // redundant
			configure_system_off_dock(); // usually charging, i would flash LED but that will drain the battery while it is charging..
		}
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
		sys_write(PAIRED_ID, &retained.paired_addr, paired_addr, sizeof(paired_addr)); // write zeroes (addr not copied yet)
		reset_mode = 0; // Clear reset mode
	} else {
		// Read paired address from retained
		memcpy(paired_addr, retained.paired_addr, sizeof(paired_addr));
	}

#if CONFIG_BOARD_SUPERMINI // Using Adafruit bootloader
	if (reset_mode >= 4) { // DFU_MAGIC_UF2_RESET, Reset mode DFU
		NRF_POWER->GPREGRET = 0x57;
		sys_reboot(SYS_REBOOT_COLD);
	}
#endif

	clocks_start();

	if (paired_addr[0] == 0x00) { // No dongle paired
		for (int i = 0; i < 4; i++) {
			base_addr_0[i] = discovery_base_addr_0[i];
			base_addr_1[i] = discovery_base_addr_1[i];
		}
		for (int i = 0; i < 8; i++) {
			addr_prefix[i] = discovery_addr_prefix[i];
		}
		esb_initialize();
//	timer_init();
		tx_payload_pair.noack = false;
		uint64_t addr = (((uint64_t)(NRF_FICR->DEVICEADDR[1]) << 32) | NRF_FICR->DEVICEADDR[0]) & 0xFFFFFF;
		uint8_t check = addr & 255;
		if (check == 0) check = 8;
		LOG_INF("Check Code: %02X", paired_addr[0]);
		tx_payload_pair.data[0] = check; // Use int from device address to make sure packet is for this device
		for (int i = 0; i < 6; i++) {
			tx_payload_pair.data[i+2] = (addr >> (8 * i)) & 0xFF;
		}
		set_led(SYS_LED_PATTERN_SHORT);
		while (paired_addr[0] != check) {
			if (paired_addr[0] != 0x00) {
				LOG_INF("Incorrect check code: %02X", paired_addr[0]);
				paired_addr[0] = 0x00; // Packet not for this device
			}
			esb_flush_rx();
			esb_flush_tx();
			esb_write_payload(&tx_payload_pair); // Still fails after a while
			esb_start_tx();
			k_msleep(1000);
			power_check();
		}
		set_led(SYS_LED_PATTERN_OFF);
		LOG_INF("Paired");
		sys_write(PAIRED_ID, retained.paired_addr, paired_addr, sizeof(paired_addr)); // Write new address and tracker id
		esb_disable();
	}
	LOG_INF("Read pairing data");
	LOG_INF("Check Code: %02X", paired_addr[0]);
	LOG_INF("Tracker ID: %u", paired_addr[1]);
	LOG_INF("Address: %02X %02X %02X %02X %02X %02X", paired_addr[2], paired_addr[3], paired_addr[4], paired_addr[5], paired_addr[6], paired_addr[7]);

	tracker_id = paired_addr[1];

	// Recreate dongle address
	uint8_t buf2[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	for (int i = 0; i < 4; i++) {
		buf2[i] = paired_addr[i+2];
		buf2[i+4] = paired_addr[i+2] + paired_addr[6];
	}
	for (int i = 0; i < 8; i++) {
		buf2[i+8] = paired_addr[7] + i;
	}
	for (int i = 0; i < 16; i++) {
		if (buf2[i] == 0x00 || buf2[i] == 0x55 || buf2[i] == 0xAA) {
			buf2[i] += 8;
		};
	}
	for (int i = 0; i < 4; i++) {
		base_addr_0[i] = buf2[i];
		base_addr_1[i] = buf2[i+4];
	}
	for (int i = 0; i < 8; i++) {
		addr_prefix[i] = buf2[i+8];
	}

	sensor_read_retained();

	// Recover quats if present
	// TODO: move this
	if (retained.stored_quats) {
		for (uint8_t i = 0; i < 4; i++){
			q[i] = retained.q[i];
		}
		LOG_INF("Recovered quaternions\nMain: %.2f %.2f %.2f %.2f", q[0], q[1], q[2], q[3]);
		retained.stored_quats = false; // Invalidate the retained quaternions
		retained_update();
	}
	for (uint8_t i = 0; i < 3; i++){
		gOff[i] = retained.gOff[i];
	}
	LOG_INF("Recovered fusion gyro offset\nMain: %.2f %.2f %.2f", gOff[0], gOff[1], gOff[2]);
// 0ms delta to read calibration and configure pins (unknown time to read retained data but probably negligible)
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
		{
			LOG_INF("Waiting for system off (Low battery)");
			wait_for_threads();
			LOG_INF("Shutdown");
			sensor_shutdown();
			// Turn off LED
			set_led(SYS_LED_PATTERN_OFF);
			configure_system_off_chgstat();
		}
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
		{ // TODO: move to interrupts? (Then you do not need to do the above)
			LOG_INF("Waiting for system off (Docked)");
			wait_for_threads();
			LOG_INF("Shutdown");
			sensor_shutdown();
			// Turn off LED
			set_led(SYS_LED_PATTERN_OFF);
			configure_system_off_dock();
		}

		if (system_off_main) { // System off on extended no movement
			LOG_INF("Waiting for system off (No movement)");
			wait_for_threads();
			LOG_INF("Shutdown");
			sensor_shutdown(); // TODO: Wait for icm reset?
			// Turn off LED
			set_led(SYS_LED_PATTERN_OFF);
			configure_system_off_WOM(main_imu);
		}

		reconfig = last_powerstate != powerstate ? true : false;
		last_powerstate = powerstate;
		main_data = false;

		wait_for_threads();
		main_imu_wakeup();
		threads_running = true;

#if MAG_ENABLED
		// Save magCal while idling
		if (magCal == 0b111111 && last_powerstate == 1) {
//			k_usleep(1); // yield to imu thread first
			k_yield(); // yield to imu thread first
			wait_for_threads(); // make sure not to interrupt anything (8ms)
			LOG_INF("Calibrating magnetometer");
			magneto_current_calibration(magBAinv, ata, norm_sum, sample_count); // 25ms
			sys_write(MAIN_MAG_BIAS_ID, &retained.magBAinv, magBAinv, sizeof(magBAinv));
			retained_update();
			for (int i = 0; i < 3; i++) {
				LOG_INF("%.5f %.5f %.5f %.5f", magBAinv[0][i], magBAinv[1][i], magBAinv[2][i], magBAinv[3][i]);
			}
			LOG_INF("Finished mag hard/soft iron offset calibration");
			//magCal |= 1 << 7;
			magCal = 0;
			// clear data
			//memset(ata[0], 0, sizeof(ata)); // does this work??
			for (int i = 0; i < 100; i++) {
				ata[i] = 0.0;
			}
			norm_sum = 0.0;
			sample_count = 0.0;
		}
#endif

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
