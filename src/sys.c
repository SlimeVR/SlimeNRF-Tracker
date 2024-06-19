#include "globals.h"

LOG_MODULE_REGISTER(sys, 4);

void configure_system_off_WOM(const struct i2c_dt_spec imu)
{
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
	// Set system off
	icm_setup_WOM(imu); // enable WOM feature
	sys_poweroff();
}

void configure_system_off_chgstat(void){
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
	// Set system off
	sys_poweroff();
}

void configure_system_off_dock(void){
	// Configure dock interrupt
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL); // Still works
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_HIGH);
	// Store fusion gyro offset
	for (uint8_t i = 0; i < 3; i++){
		retained.gOff[i] = gOff[i];
	}
	retained_update();
	// Set system off
	sys_poweroff();
}

void power_check(void) {
	bool docked = gpio_pin_get_dt(&dock);
	int batt_mV;
	uint32_t batt_pptt = read_batt_mV(&batt_mV);
	if (batt_pptt == 0 && !docked) {
		// Communicate all imus to shut down
		i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
		i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
		gpio_pin_set_dt(&led, 0); // Turn off LED
		configure_system_off_chgstat();
	} else if (docked) {
		// Communicate all imus to shut down
		i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // Don't need to wait for ICM to finish reset
		i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
		gpio_pin_set_dt(&led, 0); // Turn off LED
		configure_system_off_dock(); // usually charging, i would flash LED but that will drain the battery while it is charging..
	}
	LOG_INF("Battery %u%% (%dmV)", batt_pptt/100, batt_mV);
}

#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	sys_reboot(SYS_REBOOT_COLD); // treat like pin reset but without pin reset reason
}
#endif
