#include "globals.h"
#include "system.h"
#include "sensor.h"
#include "build_defines.h"

#define USB DT_NODELABEL(usbd)
#if DT_NODE_HAS_STATUS(USB, okay) && CONFIG_USE_SLIMENRF_CONSOLE

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/console/console.h>
#include <zephyr/sys/reboot.h>

#include <ctype.h>

LOG_MODULE_REGISTER(console, LOG_LEVEL_INF);

static void usb_init_thread(void);
K_THREAD_DEFINE(usb_init_thread_id, 256, usb_init_thread, NULL, NULL, NULL, 6, 0, 500); // Wait before enabling USB

static void console_thread(void);
static struct k_thread console_thread_id;
static K_THREAD_STACK_DEFINE(console_thread_id_stack, 512);

#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2

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

	printk("Board configuration: ");
	printk(CONFIG_BOARD);
	printk("\n");
	printk("SOC: ");
	printk(CONFIG_SOC);
	printk("\n");

	printk("IMU address: 0x%02X, register: 0x%02X\n", retained.imu_addr, retained.imu_reg);
	printk("Magnetometer address: 0x%02X, register: 0x%02X\n", retained.mag_addr, retained.mag_reg);

	printk("IMU: %s\n", sensor_get_sensor_imu_name());
	printk("Magnetometer: %s\n", sensor_get_sensor_mag_name());

	printk("Accelerometer bias: %.5f %.5f %.5f\n", retained.accelBias[0], retained.accelBias[1], retained.accelBias[2]);
	printk("Gyroscope bias: %.5f %.5f %.5f\n", retained.gyroBias[0], retained.gyroBias[1], retained.gyroBias[2]);
	printk("Magnetometer bridge offset: %.5f %.5f %.5f\n", retained.magBias[0], retained.magBias[1], retained.magBias[2]);
	printk("Magnetometer matrix:\n");
	for (int i = 0; i < 3; i++)
		printk("%.5f %.5f %.5f %.5f\n", retained.magBAinv[0][i], retained.magBAinv[1][i], retained.magBAinv[2][i], retained.magBAinv[3][i]);

	printk("Fusion: %s\n", sensor_get_sensor_fusion_name());

	printk("Tracker ID: %u\n", retained.paired_addr[1]);
	printk("Device address: %012llX\n", *(uint64_t *)NRF_FICR->DEVICEADDR & 0xFFFFFFFFFFFF);
	printk("Receiver Address: %012llX\n", (*(uint64_t *)&retained.paired_addr[0] >> 16) & 0xFFFFFFFFFFFF);
}

static void console_thread(void)
{
	console_getline_init();
	printk("*** " CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT " ***\n");
	printk(FW_STRING);
	printk("info                         Get device information\n");
	printk("reboot                       Soft reset the device\n");
	printk("calibrate                    Calibrate sensor ZRO\n");
	printk("pair                         Clear pairing data\n");

	uint8_t command_info[] = "info";
	uint8_t command_reboot[] = "reboot";
	uint8_t command_calibrate[] = "calibrate";
	uint8_t command_pair[] = "pair";

#if DFU_EXISTS
	printk("dfu                          Enter DFU bootloader\n");

	uint8_t command_dfu[] = "dfu";
#endif

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
			reboot_counter_write(101);
			k_msleep(1);
			sys_reboot(SYS_REBOOT_WARM);
		}
		else if (memcmp(line, command_pair, sizeof(command_pair)) == 0) 
		{
			reboot_counter_write(102);
			k_msleep(1);
			sys_reboot(SYS_REBOOT_WARM);
		}
#if DFU_EXISTS
		else if (memcmp(line, command_dfu, sizeof(command_dfu)) == 0)
		{
			NRF_POWER->GPREGRET = 0x57;
			sys_reboot(SYS_REBOOT_COLD);
		}
#endif
		else
		{
			printk("Unknown command\n");
		}
	}
}

#endif