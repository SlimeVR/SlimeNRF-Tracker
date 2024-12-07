#include "globals.h"
#include "system.h"
#include "sensor.h"
#include "esb.h"
#include "build_defines.h"

#define USB DT_NODELABEL(usbd)
#if DT_NODE_HAS_STATUS(USB, okay) && CONFIG_USE_SLIMENRF_CONSOLE

#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/console/console.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log_ctrl.h>

#include <ctype.h>

LOG_MODULE_REGISTER(console, LOG_LEVEL_INF);

static void usb_init_thread(void);
K_THREAD_DEFINE(usb_init_thread_id, 256, usb_init_thread, NULL, NULL, NULL, 6, 0, 500); // Wait before enabling USB

static void console_thread(void);
static struct k_thread console_thread_id;
static K_THREAD_STACK_DEFINE(console_thread_id_stack, 512);

#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
#define ADAFRUIT_BOOTLOADER CONFIG_BUILD_OUTPUT_UF2
#define NRF5_BOOTLOADER CONFIG_BOARD_HAS_NRF5_BOOTLOADER

#if NRF5_BOOTLOADER
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
#endif

static const char *meows[] = {
	"Meow",
	"Meow meow",
	"Mrrrp",
	"Mrrf",
	"Mreow",
	"Mrrrow",
	"Mrrr",
	"Purr",
	"meow",
	"meow meow",
	"mrrrp",
	"mrrf",
	"mreow",
	"mrrrow",
	"mrrr",
	"purr",
};

static const char *meow_punctuations[] = {
	".",
	"?",
	"!",
	"-",
	"~",
	""
};

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	switch (status)
	{
	case USB_DC_CONNECTED:
		set_status(SYS_STATUS_USB_CONNECTED, true);
		k_thread_create(&console_thread_id, console_thread_id_stack, K_THREAD_STACK_SIZEOF(console_thread_id_stack), (k_thread_entry_t)console_thread, NULL, NULL, NULL, 6, 0, K_NO_WAIT);
		break;
	case USB_DC_DISCONNECTED:
		set_status(SYS_STATUS_USB_CONNECTED, false);
		k_thread_abort(&console_thread_id);
		break;
	default:
		LOG_DBG("status %u unhandled", status);
		break;
	}
}

static void usb_init_thread(void)
{
	usb_enable(status_cb);
}

static void print_info(void)
{
	printk(CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT "\n");
	printk(FW_STRING);

	printk("Board configuration: " CONFIG_BOARD "\n");
	printk("SOC: " CONFIG_SOC "\n");

	printk("IMU address: 0x%02X, register: 0x%02X\n", retained.imu_addr, retained.imu_reg);
	printk("Magnetometer address: 0x%02X, register: 0x%02X\n", retained.mag_addr, retained.mag_reg);

	printk("IMU: %s\n", sensor_get_sensor_imu_name());
	printk("Magnetometer: %s\n", sensor_get_sensor_mag_name());

	printk("Accelerometer bias: %.5f %.5f %.5f\n", (double)retained.accelBias[0], (double)retained.accelBias[1], (double)retained.accelBias[2]);
	printk("Gyroscope bias: %.5f %.5f %.5f\n", (double)retained.gyroBias[0], (double)retained.gyroBias[1], (double)retained.gyroBias[2]);
	printk("Magnetometer bridge offset: %.5f %.5f %.5f\n", (double)retained.magBias[0], (double)retained.magBias[1], (double)retained.magBias[2]);
	printk("Magnetometer matrix:\n");
	for (int i = 0; i < 3; i++)
		printk("%.5f %.5f %.5f %.5f\n", (double)retained.magBAinv[0][i], (double)retained.magBAinv[1][i], (double)retained.magBAinv[2][i], (double)retained.magBAinv[3][i]);

	printk("Fusion: %s\n", sensor_get_sensor_fusion_name());

	printk("Tracker ID: %u\n", retained.paired_addr[1]);
	printk("Device address: %012llX\n", *(uint64_t *)NRF_FICR->DEVICEADDR & 0xFFFFFFFFFFFF);
	printk("Receiver Address: %012llX\n", (*(uint64_t *)&retained.paired_addr[0] >> 16) & 0xFFFFFFFFFFFF);
}

static void console_thread(void)
{
	console_getline_init();
	while (log_data_pending())
		k_usleep(1);
	k_msleep(100);
	printk("*** " CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT " ***\n");
	printk(FW_STRING);
	printk("info                         Get device information\n");
	printk("reboot                       Soft reset the device\n");
	printk("calibrate                    Calibrate sensor ZRO\n");

	uint8_t command_info[] = "info";
	uint8_t command_reboot[] = "reboot";
	uint8_t command_calibrate[] = "calibrate";

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	printk("6-side                       Calibrate 6-side accelerometer\n");

	uint8_t command_6_side[] = "6-side";
#endif

	printk("pair                         Clear pairing data\n");

	uint8_t command_pair[] = "pair";

#if DFU_EXISTS
	printk("dfu                          Enter DFU bootloader\n");

	uint8_t command_dfu[] = "dfu";
#endif

	printk("meow                         Meow!\n");

	uint8_t command_meow[] = "meow";

	while (1) {
		uint8_t *line = console_getline();
		for (uint8_t *p = line; *p; ++p) {
			*p = tolower(*p);
		}

		if (memcmp(line, command_info, sizeof(command_info)) == 0)
		{
			print_info();
		}
		else if (memcmp(line, command_reboot, sizeof(command_reboot)) == 0)
		{
			sys_reboot(SYS_REBOOT_COLD);
		}
		else if (memcmp(line, command_calibrate, sizeof(command_calibrate)) == 0)
		{
//			reboot_counter_write(101);
			sensor_request_calibration();
			k_msleep(1);
			sys_reboot(SYS_REBOOT_WARM);
		}
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
		else if (memcmp(line, command_6_side, sizeof(command_6_side)) == 0)
		{
			sensor_request_calibration_6_side();
			k_msleep(1);
			sys_reboot(SYS_REBOOT_WARM);
		}
#endif
		else if (memcmp(line, command_pair, sizeof(command_pair)) == 0) 
		{
//			reboot_counter_write(102);
			esb_reset_pair();
			k_msleep(1);
			sys_reboot(SYS_REBOOT_WARM);
		}
#if DFU_EXISTS
		else if (memcmp(line, command_dfu, sizeof(command_dfu)) == 0)
		{
#if ADAFRUIT_BOOTLOADER
			NRF_POWER->GPREGRET = 0x57;
			sys_reboot(SYS_REBOOT_COLD);
#endif
#if NRF5_BOOTLOADER
			gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
#endif
		}
#endif
		else if (memcmp(line, command_meow, sizeof(command_meow)) == 0) 
		{
			uint32_t cycles = k_cycle_get_32();
			cycles %= ARRAY_SIZE(meows) * ARRAY_SIZE(meow_punctuations); // silly number generator
			printk("%s%s\n", meows[cycles % ARRAY_SIZE(meows)], meow_punctuations[cycles / ARRAY_SIZE(meows)]);
		}
		else
		{
			printk("Unknown command\n");
		}
	}
}

#endif