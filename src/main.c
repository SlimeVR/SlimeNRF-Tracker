#include "globals.h"
#include "sys.h"
//#include "timer.h"
#include "util.h"
#include "esb.h"

LOG_MODULE_REGISTER(main, 4);

void set_LN(void) {
	tickrate = 6;
	// TODO: This becomes part of the sensor
//	aMode = aMode_LN;
#if MAG_ENABLED
//	gMode = gMode_LN;
//	MBW = MBW_400Hz;
	MODR = MODR_200Hz;
#endif
}

void set_LP(void) {
	tickrate = 33;
	// TODO: This becomes part of the sensor
//	aMode = aMode_LP;
#if MAG_ENABLED
//	gMode = gMode_SBY;
//	MBW = MBW_800Hz;
	MODR = MODR_ONESHOT;
#endif
}

void reconfigure_imu(const struct i2c_dt_spec imu) {
	i2c_reg_write_byte_dt(&imu, ICM42688_ACCEL_CONFIG0, Ascale << 5 | AODR); // set accel ODR and FS
	i2c_reg_write_byte_dt(&imu, ICM42688_GYRO_CONFIG0, Gscale << 5 | GODR); // set gyro ODR and FS
	i2c_reg_write_byte_dt(&imu, ICM42688_PWR_MGMT0, gMode << 2 | aMode); // set accel and gyro modes
}

void reconfigure_mag(const struct i2c_dt_spec mag) {
	//i2c_reg_write_byte_dt(&mag, MMC5983MA_CONTROL_1, MBW); // set mag bandwidth
	i2c_reg_write_byte_dt(&mag, MMC5983MA_CONTROL_2, 0x80 | (MSET << 4) | 0x08 | MODR); // set mag ODR
}

bool wait_for_motion(const struct i2c_dt_spec imu, bool motion, int samples) {
	uint8_t counts = 0;
	float a[3], last_a[3];
	icm_accel_read(imu, last_a);
	for (int i = 0; i < samples + counts; i++) {
gpio_pin_toggle_dt(&led); // scuffed led
		LOG_INF("Accel: %.5f %.5f %.5f", a[0], a[1], a[2]);
		k_msleep(500);
		icm_accel_read(imu, a);
		if (vec_epsilon(a, last_a) != motion) {
			LOG_INF("Pass");
			counts++;
			if (counts == 2) {
				return true;
			}
		} else {
			counts = 0;
		}
		memcpy(last_a, a, sizeof(a));
	}
	LOG_INF("Fail");
	return false;
}

// TODO: make threads more abstract, pass in imus n stuff instead
void main_imu_thread(void) {
	main_running = true;
	while (1) {
		if (main_ok)
		{
			// Reading IMUs will take between 2.5ms (~7 samples, low noise) - 7ms (~33 samples, low power)
			// Magneto sample will take ~400us
			// Fusing data will take between 100us (~7 samples, low noise) - 500us (~33 samples, low power)
			// TODO: on any errors set main_ok false and skip (make functions return nonzero)
			// Read main FIFO
			uint8_t rawCount[2];
			i2c_burst_read_dt(&main_imu, ICM42688_FIFO_COUNTH, &rawCount[0], 2);
			uint16_t count = (uint16_t)(rawCount[0] << 8 | rawCount[1]); // Turn the 16 bits into a unsigned 16-bit value
//						LOG_INF("packs %u", count);
			count += 32; // Add a few read buffer packets (4 ms)
			uint16_t packets = count / 8;								 // Packet size 8 bytes
			uint8_t rawData[2080];
			uint16_t stco = 0;
			uint8_t addr = ICM42688_FIFO_DATA;
			i2c_write_dt(&main_imu, &addr, 1); // Start read buffer
			while (count > 0) {
				i2c_read_dt(&main_imu, &rawData[stco], count > 248 ? 248 : count); // Read less than 255 at a time (for nRF52832)
				stco += 248;
				count = count > 248 ? count - 248 : 0;
//						LOG_INF("left %u", count);
			}

			float a[3];
			icm_accel_read(main_imu, a);
			float ax = a[0] - accelBias[0];
			float ay = a[1] - accelBias[1];
			float az = a[2] - accelBias[2];

			float mx = 0, my = 0, mz = 0;
#if MAG_ENABLED
			if (last_powerstate == 0) {
				float m[3];
				mmc_mag_read(main_mag, m);
				magneto_sample(m[0], m[1], m[2], ata, &norm_sum, &sample_count); // 400us
				apply_BAinv(m, magBAinv);
				mx = m[0];
				my = m[1];
				mz = m[2];
				int new_magCal = magCal;
				new_magCal |= (-1.2 < ax && ax < -0.8 ? 1 << 0 : 0) | (1.2 > ax && ax > 0.8 ? 1 << 1 : 0) | // dumb check if all accel axes were reached for cal, assume the user is intentionally doing this
					(-1.2 < ay && ay < -0.8 ? 1 << 2 : 0) | (1.2 > ay && ay > 0.8 ? 1 << 3 : 0) |
					(-1.2 < az && az < -0.8 ? 1 << 4 : 0) | (1.2 > az && az > 0.8 ? 1 << 5 : 0);
				if (new_magCal > magCal && new_magCal == last_magCal) {
					if (k_uptime_get() > magCal_time) {
						magCal = new_magCal;
						LOG_INF("Progress magCal: %d", new_magCal);
					}
				} else {
					magCal_time = k_uptime_get() + 1000;
					last_magCal = new_magCal;
				}
				if (magCal == 0b111111) {
					gpio_pin_set_dt(&led, 1);
				}
			}

			if (reconfig) {
				switch (powerstate) {
					case 0:
						set_LN();
						LOG_INF("Switch main imus to low noise");
						last_mag_level = -1;
						break;
					case 1:
						set_LP();
						LOG_INF("Switch main imus to low power");
						reconfigure_mag(main_mag);
						break;
				};
				//reconfigure_imu(main_imu); // Reconfigure if needed
				//reconfigure_mag(main_mag); // Reconfigure if needed
			}
			if (last_mag_level != mag_level && powerstate == 0) {
				switch (mag_level) {
					case 0:
						MODR = MODR_10Hz;
						//LOG_INF("Switch mag to 10Hz");
						break;
					case 1:
						MODR = MODR_20Hz;
						//LOG_INF("Switch mag to 20Hz");
						break;
					case 2:
						MODR = MODR_50Hz;
						//LOG_INF("Switch mag to 50Hz");
						break;
					case 3:
						MODR = MODR_100Hz;
						//LOG_INF("Switch mag to 100Hz");
						break;
					case 4:
						MODR = MODR_200Hz;
						//LOG_INF("Switch mag to 200Hz");
						break;
				};
				reconfigure_mag(main_mag);
			}
			last_mag_level = mag_level;
			mag_level = 0;
#endif

			FusionVector z = {.array = {0, 0, 0}};
			if (packets == 2 && powerstate == 1 && MAG_ENABLED) {
					ahrs.initialising = true;
					ahrs.rampedGain = 10.0f;
					ahrs.accelerometerIgnored = false;
					ahrs.accelerationRecoveryTrigger = 0;
					ahrs.accelerationRecoveryTimeout = 0;
					FusionVector a = {.array = {ax, -az, ay}};
					FusionAhrsUpdate(&ahrs, z, a, z, INTEGRATION_TIME_LP);
					memcpy(q, ahrs.quaternion.array, sizeof(q));
			} else {
				FusionVector g = {.array = {0, 0, 0}};
				FusionVector a = {.array = {ax, -az, ay}};
				FusionVector m = {.array = {my, mz, -mx}};
				for (uint16_t i = 0; i < packets; i++)
				{
					uint16_t index = i * 8; // Packet size 8 bytes
					if ((rawData[index] & 0x80) == 0x80) {
						continue; // Skip empty packets
					}
					// combine into 16 bit values
					float raw0 = (int16_t)((((int16_t)rawData[index + 1]) << 8) | rawData[index + 2]); // gx
					float raw1 = (int16_t)((((int16_t)rawData[index + 3]) << 8) | rawData[index + 4]); // gy
					float raw2 = (int16_t)((((int16_t)rawData[index + 5]) << 8) | rawData[index + 6]); // gz
					if (raw0 < -32766 || raw1 < -32766 || raw2 < -32766) {
						continue; // Skip invalid data
					}
					// transform and convert to float values
					float gx = raw0 * (2000.0f/32768.0f) - gyroBias[0]; //gres
					float gy = raw1 * (2000.0f/32768.0f) - gyroBias[1]; //gres
					float gz = raw2 * (2000.0f/32768.0f) - gyroBias[2]; //gres
					g.axis.x = gx;
					g.axis.y = -gz;
					g.axis.z = gy;
					//FusionVector g = {.array = {gx, -gz, gy}};
					FusionVector g_off = FusionOffsetUpdate2(&offset, g);
#if MAG_ENABLED
					float gyro_speed_square = g.array[0]*g.array[0] + g.array[1]*g.array[1] + g.array[2]*g.array[2];
					// target mag ODR for ~0.25 deg error
					if (gyro_speed_square > 25*25 && mag_level < 3) // >25dps -> 200hz ODR
						mag_level = 4;
					else if (gyro_speed_square > 12*12 && mag_level < 2) // 12-25dps -> 100hz ODR
						mag_level = 3;
					else if (gyro_speed_square > 5*5 && mag_level < 1) // 5-12dps -> 50hz ODR
						mag_level = 2;
					else if (gyro_speed_square > 2*2 && mag_level < 1) // 2-5dps -> 20hz ODR
						mag_level = 1;
					// <2dps -> 10hz ODR
					if (offset.timer < offset.timeout)
						FusionAhrsUpdate(&ahrs, g_off, a, m, INTEGRATION_TIME);
					else
						FusionAhrsUpdate(&ahrs, z, a, m, INTEGRATION_TIME);
#else
					if (offset.timer < offset.timeout)
						FusionAhrsUpdate(&ahrs, g, a, z, INTEGRATION_TIME);
					else
						FusionAhrsUpdate(&ahrs, z, a, z, INTEGRATION_TIME);
#endif
				}

				if (FusionAhrsGetFlags(&ahrs).magneticRecovery) {
					if (gyro_sanity == 2 && vec_epsilon2(gyro_sanity_m.array, m.array, 0.01f)) {
						// For whatever reason the gyro seems unreliable
						// Reset the offset here so the tracker can probably at least turn off
						LOG_INF("Gyro seems unreliable!");
						offset.gyroscopeOffset = g;
						gyro_sanity = 3;
					} else if (gyro_sanity % 2 == 0) {
						LOG_INF("Magnetic recovery");
						gyro_sanity_m = m;
						gyro_sanity = 1;
					}
				} else if (gyro_sanity == 1) {
					LOG_INF("Recovered once");
					gyro_sanity = 2;
				} else if (gyro_sanity == 3) {
					LOG_INF("Reset gyro sanity");
					gyro_sanity = 0;
				}
				
				const FusionVector lin_a = FusionAhrsGetLinearAcceleration(&ahrs); // im going insane
				lin_ax = lin_a.array[0] * CONST_EARTH_GRAVITY; // Also change to m/s for SlimeVR server
				lin_ay = lin_a.array[1] * CONST_EARTH_GRAVITY;
				lin_az = lin_a.array[2] * CONST_EARTH_GRAVITY;
				memcpy(q, ahrs.quaternion.array, sizeof(q));
				memcpy(gOff, offset.gyroscopeOffset.array, sizeof(gOff));
			}

			if (gyro_sanity == 0 ? quat_epsilon_coarse(q, last_q) : quat_epsilon_coarse2(q, last_q)) { // Probably okay to use the constantly updating last_q
				int64_t imu_timeout = CLAMP(last_data_time, 1 * 1000, 15 * 1000); // Ramp timeout from last_data_time
				if (k_uptime_get() - last_data_time > imu_timeout) { // No motion in last 1s - 10s
					LOG_INF("No motion from main imus in %llds", imu_timeout/1000);
					system_off_main = true;
				} else if (powerstate == 0 && k_uptime_get() - last_data_time > 500) { // No motion in last 500ms
					LOG_INF("No motion from main imus in 500ms");
					powerstate = 1;
				}
			} else {
				last_data_time = k_uptime_get();
				powerstate = 0;
			}

			if (!(quat_epsilon(q, last_q))) {
				for (uint8_t i = 0; i < 4; i++) {
					last_q[i] = q[i];
				}
				for (uint16_t i = 0; i < 16; i++) {
					tx_payload.data[i] = 0;
				}
				float q_offset[4];
				q_multiply(q, q3, q_offset);
				tx_buf[0] = TO_FIXED_15(q_offset[1]);
				tx_buf[1] = TO_FIXED_15(q_offset[2]);
				tx_buf[2] = TO_FIXED_15(q_offset[3]);
				tx_buf[3] = TO_FIXED_15(q_offset[0]);
				tx_buf[4] = TO_FIXED_7(lin_ax);
				tx_buf[5] = TO_FIXED_7(lin_ay);
				tx_buf[6] = TO_FIXED_7(lin_az);
				tx_payload.data[0] = 0; //reserved for something idk
				tx_payload.data[1] = tracker_id << 4;
				//tx_payload.data[2] = batt | (charging ? 128 : 0);
				// TODO: Send temperature
				tx_payload.data[2] = batt;
				tx_payload.data[3] = batt_v;
				tx_payload.data[4] = (tx_buf[0] >> 8) & 255;
				tx_payload.data[5] = tx_buf[0] & 255;
				tx_payload.data[6] = (tx_buf[1] >> 8) & 255;
				tx_payload.data[7] = tx_buf[1] & 255;
				tx_payload.data[8] = (tx_buf[2] >> 8) & 255;
				tx_payload.data[9] = tx_buf[2] & 255;
				tx_payload.data[10] = (tx_buf[3] >> 8) & 255;
				tx_payload.data[11] = tx_buf[3] & 255;
				tx_payload.data[12] = (tx_buf[4] >> 8) & 255;
				tx_payload.data[13] = tx_buf[4] & 255;
				tx_payload.data[14] = (tx_buf[5] >> 8) & 255;
				tx_payload.data[15] = tx_buf[5] & 255;
				tx_payload.data[16] = (tx_buf[6] >> 8) & 255;
				tx_payload.data[17] = tx_buf[6] & 255;
				esb_flush_tx();
				main_data = true;
				esb_write_payload(&tx_payload); // Add transmission to queue
				send_data = true;
			}
		} else {
// 5ms delta (???) from entering loop
// skip sleep, surely this wont cause issues :D
/*
			int64_t time_delta = k_uptime_get() - start_time;
			if (time_delta < 11)
			{
				k_msleep(11 - time_delta);
			}
			//k_msleep(11);														 // Wait for start up (1ms for ICM, 10ms for MMC -> 10ms)
*/
			uint8_t ICM42688ID = icm_getChipID(main_imu);						 // Read CHIP_ID register for ICM42688
				LOG_INF("ICM: %u", ICM42688ID);
			uint8_t MMC5983ID = mmc_getChipID(main_mag);						 // Read CHIP_ID register for MMC5983MA
				LOG_INF("MMC: %u", MMC5983ID);
			if ((ICM42688ID == 0x47 || ICM42688ID == 0xDB) && (!MAG_ENABLED || MMC5983ID == 0x30)) // check if all I2C sensors have acknowledged
			{
				LOG_INF("Found main imus");
				i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // i dont wanna wait on icm!!
				i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Reset MMC now to avoid waiting 10ms later
				//icm_reset(main_imu);												 // software reset ICM42688 to default registers
				uint8_t temp;
				i2c_reg_read_byte_dt(&main_imu, ICM42688_INT_STATUS, &temp); // clear reset done int flag
				icm_init(main_imu, Ascale, Gscale, AODR, GODR, aMode, gMode, false); // configure
// 55-66ms delta to wait, get chip ids, and setup icm (50ms spent waiting for accel and gyro to start)
#if MAG_ENABLED
				mmc_SET(main_mag);													 // "deGauss" magnetometer
				mmc_init(main_mag, MODR, MBW, MSET);								 // configure
// 0-1ms delta to setup mmc
#endif
				LOG_INF("Initialized main imus");
				main_ok = true;
				main_running = false;
				k_sleep(K_FOREVER); // Wait for after calibrations have loaded the first time
				main_running = true;
				do {
				if (reset_mode == 1) { // Reset mode main calibration
					LOG_INF("Enter main calibration");
gpio_pin_set_dt(&led, 0); // scuffed led
					// TODO: Add LED flashies
					LOG_INF("Rest the device on a stable surface");
					if (!wait_for_motion(main_imu, false, 20)) { // Wait for accelerometer to settle, timeout 10s
gpio_pin_set_dt(&led, 0); // scuffed led
						break; // Timeout, calibration failed
					}
gpio_pin_set_dt(&led, 1); // scuffed led
					k_msleep(500); // Delay before beginning acquisition
					LOG_INF("Start accel and gyro calibration");
					icm_offsetBias(main_imu, accelBias, gyroBias); // This takes about 3s
					if (!nvs_init) {
						nvs_mount(&fs);
						nvs_init = true;
					}
					nvs_write(&fs, MAIN_ACCEL_BIAS_ID, &accelBias, sizeof(accelBias));
					nvs_write(&fs, MAIN_GYRO_BIAS_ID, &gyroBias, sizeof(gyroBias));
					memcpy(retained.accelBias, accelBias, sizeof(accelBias));
					memcpy(retained.gyroBias, gyroBias, sizeof(gyroBias));
					retained_update();
					LOG_INF("%.5f %.5f %.5f", accelBias[0], accelBias[1], accelBias[2]);
					LOG_INF("%.5f %.5f %.5f", gyroBias[0], gyroBias[1], gyroBias[2]);
					LOG_INF("Finished accel and gyro zero offset calibration");
					// clear fusion gyro offset
					for (uint8_t i = 0; i < 3; i++){
						gOff[i] = 0;
						retained.gOff[i] = gOff[i];
					}
					retained_update();
gpio_pin_set_dt(&led, 0); // scuffed led
					reset_mode = 0; // Clear reset mode
				}
				} while (false);
				// Setup fusion
				LOG_INF("Init fusion");
				FusionOffsetInitialise2(&offset, 1/INTEGRATION_TIME);
				FusionAhrsInitialise(&ahrs);
				// ahrs.initialising = true; // cancel fusion init, maybe only if there is a quat stored? oh well
				const FusionAhrsSettings settings = {
						.convention = FusionConventionNwu,
						.gain = 0.5f,
						.gyroscopeRange = 2000.0f, // also change gyro range in fusion! (.. does it actually work if its set to the limit?)
						.accelerationRejection = 10.0f,
						.magneticRejection = 20.0f,
						.recoveryTriggerPeriod = 5 * 1/INTEGRATION_TIME, // 5 seconds
				};
				FusionAhrsSetSettings(&ahrs, &settings);
				memcpy(ahrs.quaternion.array, q, sizeof(q)); // Load existing quat
				memcpy(offset.gyroscopeOffset.array, gOff, sizeof(gOff)); // Load fusion gyro offset
				LOG_INF("Initialized fusion");
			}
		}
		main_running = false;
		k_sleep(K_FOREVER);
		main_running = true;
	}
}

K_THREAD_DEFINE(main_imu_thread_id, 4096, main_imu_thread, NULL, NULL, NULL, 7, 0, 0);

void wait_for_main_imu_thread(void) {
	while (main_running) {
		k_usleep(1);
	}
}

void wait_for_threads(void) {
	if (threads_running || main_running) {
		while (main_running) {
			k_usleep(1);
		}
	}
}

int main(void)
{
	int32_t reset_reason = NRF_POWER->RESETREAS;
	NRF_POWER->RESETREAS = NRF_POWER->RESETREAS; // Clear RESETREAS
	uint8_t reboot_counter = 0;
	bool booting_from_shutdown = false;

	gpio_pin_configure_dt(&dock, GPIO_INPUT);
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);

	power_check(); // check the battery and dock first before continuing (4ms delta to read from ADC)

//	start_time = k_uptime_get(); // Need to get start time for imu startup delta
	gpio_pin_set_dt(&led, 1); // Boot LED

	bool ram_retention = retained_validate(); // check ram retention

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
		//nvs_read(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
		reboot_counter = retained.reboot_counter;
		if (reboot_counter > 200) reboot_counter = 200; // How did you get here
		booting_from_shutdown = reboot_counter == 0 ? true : false; // 0 means from user shutdown or failed ram validation
		if (reboot_counter == 0) reboot_counter = 100;
		reset_mode = reboot_counter - 100;
		reboot_counter++;
		//nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
		retained.reboot_counter = reboot_counter;
		retained_update();
		LOG_INF("Reset Count: %u", reboot_counter);
		if (booting_from_shutdown) { // 3 flashes
			k_msleep(200);
			for (int j = 0; j < 2; j++) {
				gpio_pin_set_dt(&led, 0);
				k_msleep(200);
				gpio_pin_set_dt(&led, 1);
				k_msleep(200);
			}
		} else
			k_msleep(1000); // Wait before clearing counter and continuing
		reboot_counter = 100;
		//nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
		retained.reboot_counter = reboot_counter;
		retained_update();
	}
// 0ms or 1000ms delta for reboot counter

#if USER_SHUTDOWN_ENABLED
	if (reset_mode == 0 && !booting_from_shutdown) { // Reset mode user shutdown
		reboot_counter = 0;
		//nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
		retained.reboot_counter = reboot_counter;
		retained_update();
		gpio_pin_set_dt(&led, 0);
		k_msleep(250);
		for (int i = 20; i > 0; i--) { // Fade LED pattern
			pwm_set_pulse_dt(&pwm_led, PWM_MSEC(i));
			k_msleep(50);
		}
		bool docked = gpio_pin_get_dt(&dock);
		if (!docked) { // TODO: should the tracker start again if docking state changes?
			// Communicate all imus to shut down
			i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
			i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
			gpio_pin_set_dt(&led, 0); // Turn off LED
			configure_system_off_chgstat();
		} else {
			// Communicate all imus to shut down
			i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
			i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
			gpio_pin_set_dt(&led, 0); // Turn off LED
			configure_system_off_dock(); // usually charging, i would flash LED but that will drain the battery while it is charging..
		}
	}
// How long user shutdown take does not matter really ("0ms")
#endif

	struct flash_pages_info info;
	fs.flash_device = NVS_PARTITION_DEVICE;
	fs.offset = NVS_PARTITION_OFFSET; // Start NVS FS here
	flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	fs.sector_size = info.size; // Sector size equal to page size
	fs.sector_count = 4U; // 4 sectors
// 5-6ms delta to initialize NVS (only done when needed)

	// All contents of NVS was stored in RAM to not need initializing NVS often
	if (!ram_retention) { 
		LOG_INF("Invalidated RAM");
		if (!nvs_init) {
			nvs_mount(&fs);
			nvs_init = true;
		}
		nvs_read(&fs, PAIRED_ID, &retained.paired_addr, sizeof(paired_addr));
		nvs_read(&fs, MAIN_ACCEL_BIAS_ID, &retained.accelBias, sizeof(accelBias));
		nvs_read(&fs, MAIN_GYRO_BIAS_ID, &retained.gyroBias, sizeof(gyroBias));
		nvs_read(&fs, MAIN_MAG_BIAS_ID, &retained.magBAinv, sizeof(magBAinv));
		retained_update();
	} else {
		LOG_INF("Recovered calibration from RAM");
	}

	gpio_pin_set_dt(&led, 0);

// TODO: if reset counter is 0 but reset reason was 1 then perform imu scanning (pressed reset once)
	if (reset_mode == 0) { // Reset mode scan imus
	}
// ?? delta

	if (reset_mode == 3) { // Reset mode pairing reset
		LOG_INF("Enter pairing reset");
		if (!nvs_init) {
			nvs_mount(&fs);
			nvs_init = true;
		}
		nvs_write(&fs, PAIRED_ID, &paired_addr, sizeof(paired_addr)); // Clear paired address
		memcpy(retained.paired_addr, paired_addr, sizeof(paired_addr));
		retained_update();
		reset_mode = 0; // Clear reset mode
	} else {
		// Read paired address from NVS
		//nvs_read(&fs, PAIRED_ID, &paired_addr, sizeof(paired_addr));
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
		while (paired_addr[0] != check) {
			if (paired_addr[0] != 0x00) {
				LOG_INF("Incorrect check code: %02X", paired_addr[0]);
				paired_addr[0] = 0x00; // Packet not for this device
			}
			esb_flush_rx();
			esb_flush_tx();
			esb_write_payload(&tx_payload_pair); // Still fails after a while
			esb_start_tx();
			gpio_pin_set_dt(&led, 1);
			k_msleep(100);
			gpio_pin_set_dt(&led, 0);
			k_msleep(900);
			power_check();
		}
		LOG_INF("Paired");
		if (!nvs_init) {
			nvs_mount(&fs);
			nvs_init = true;
		}
		nvs_write(&fs, PAIRED_ID, &paired_addr, sizeof(paired_addr)); // Write new address and tracker id
		memcpy(retained.paired_addr, paired_addr, sizeof(paired_addr));
		retained_update();
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

	// Read calibration from NVS
	//nvs_read(&fs, MAIN_ACCEL_BIAS_ID, &accelBias, sizeof(accelBias));
	//nvs_read(&fs, MAIN_GYRO_BIAS_ID, &gyroBias, sizeof(gyroBias));
	//nvs_read(&fs, MAIN_MAG_BIAS_ID, &magBAinv, sizeof(magBAinv));
	memcpy(accelBias, retained.accelBias, sizeof(accelBias));
	memcpy(gyroBias, retained.gyroBias, sizeof(gyroBias));
	memcpy(magBAinv, retained.magBAinv, sizeof(magBAinv));
	LOG_INF("Read calibrations");
	LOG_INF("Main accel bias: %.5f %.5f %.5f", accelBias[0], accelBias[1], accelBias[2]);
	LOG_INF("Main gyro bias: %.5f %.5f %.5f", gyroBias[0], gyroBias[1], gyroBias[2]);
	LOG_INF("Main mag matrix:");
	for (int i = 0; i < 3; i++) {
		LOG_INF("%.5f %.5f %.5f %.5f", magBAinv[0][i], magBAinv[1][i], magBAinv[2][i], magBAinv[3][i]);
	}

	//gpio_pin_configure_dt(&chgstat, GPIO_INPUT);
	//gpio_pin_configure_dt(&int0, GPIO_INPUT);
	//gpio_pin_configure_dt(&int1, GPIO_INPUT);

	//pwm_set_pulse_dt(&pwm_led, PWM_MSEC(5)); // 5/20 = 25%
	//gpio_pin_set_dt(&led, 1);

	// Recover quats if present
	//retained_validate();
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
			// Communicate all imus to shut down
			i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
			i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
			// Turn off LED
			gpio_pin_set_dt(&led, 0);
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

		pwm_set_pulse_dt(&pwm_led, 0);

		if (docked) // TODO: keep sending battery state while plugged and docked?
		{ // TODO: move to interrupts? (Then you do not need to do the above)
			LOG_INF("Waiting for system off (Docked)");
			wait_for_threads();
			LOG_INF("Shutdown");
			// Communicate all imus to shut down
			i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
			i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
			// Turn off LED
			gpio_pin_set_dt(&led, 0);
			configure_system_off_dock();
		}

		if (system_off_main) { // System off on extended no movement
			LOG_INF("Waiting for system off (No movement)");
			wait_for_threads();
			LOG_INF("Shutdown");
			// Communicate all imus to shut down
			icm_reset(main_imu);
			i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
			// Turn off LED
			gpio_pin_set_dt(&led, 0);
			configure_system_off_WOM(main_imu);
		}

		reconfig = last_powerstate != powerstate ? true : false;
		last_powerstate = powerstate;
		main_data = false;

		wait_for_threads();
		k_wakeup(main_imu_thread_id);
		threads_running = true;

#if MAG_ENABLED
		// Save magCal while idling
		if (magCal == 0b111111 && last_powerstate == 1) {
			if (!nvs_init) {
				nvs_mount(&fs);
				nvs_init = true;
			}
			k_usleep(1); // yield to imu thread first
			wait_for_threads(); // make sure not to interrupt anything (8ms)
			LOG_INF("Calibrating magnetometer");
			magneto_current_calibration(magBAinv, ata, norm_sum, sample_count); // 25ms
			nvs_write(&fs, MAIN_MAG_BIAS_ID, &magBAinv, sizeof(magBAinv));
			memcpy(retained.magBAinv, magBAinv, sizeof(magBAinv));
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
