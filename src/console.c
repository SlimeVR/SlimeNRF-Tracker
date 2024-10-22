#include "globals.h"
#include "system.h"

#define USB DT_NODELABEL(usbd)
#if DT_NODE_HAS_STATUS(USB, okay) && CONFIG_USB_DEVICE_STACK

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/console/console.h>
#include <zephyr/sys/reboot.h>

static bool configured;
static const struct device *hdev;
static ATOMIC_DEFINE(hid_ep_in_busy, 1);

#define HID_EP_BUSY_FLAG 0
#include <ctype.h>

LOG_MODULE_REGISTER(console, LOG_LEVEL_INF);

static void int_in_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	if (!atomic_test_and_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		LOG_WRN("IN endpoint callback without preceding buffer write");
	}
}

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	switch (status) {
	case USB_DC_RESET:
		configured = false;
		break;
	case USB_DC_CONFIGURED:
		if (!configured) {
			int_in_ready_cb(hdev);
			configured = true;
		}
		break;
	case USB_DC_SOF:
		break;
	default:
		LOG_DBG("status %u unhandled", status);
		break;
	}
}

void usb_init_thread(void)
{
	k_msleep(1000);
	usb_disable();
	k_msleep(1000); // Wait before enabling USB // TODO: why does it need to wait so long
	usb_enable(status_cb);
}

K_THREAD_DEFINE(usb_init_thread_id, 256, usb_init_thread, NULL, NULL, NULL, 6, 0, 0);

void console_thread(void)
{
	console_getline_init();
	printk("*** " CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT " ***\n");
	printk("reboot                       Soft reset the device\n");
	printk("calibrate                    Calibrate sensor ZRO\n");
	printk("pair                         Clear pairing data\n");
	printk("dfu                          Enter DFU bootloader\n");

	uint8_t command_reboot[] = "reboot";
	uint8_t command_calibrate[] = "calibrate";
	uint8_t command_pair[] = "pair";
	uint8_t command_dfu[] = "dfu";

	while (1) {
		uint8_t *line = console_getline();
		for (uint8_t *p = line; *p; ++p) {
			*p = tolower(*p);
		}

		if (memcmp(line, command_reboot, sizeof(command_reboot)) == 0)
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
		else if (memcmp(line, command_dfu, sizeof(command_dfu)) == 0)
		{
			NRF_POWER->GPREGRET = 0x57;
			sys_reboot(SYS_REBOOT_COLD);
		}
		else
		{
			printk("Unknown command\n");
		}
	}
}

K_THREAD_DEFINE(console_thread_id, 512, console_thread, NULL, NULL, NULL, 6, 0, 0);

#endif