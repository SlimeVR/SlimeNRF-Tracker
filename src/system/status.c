#include "..\globals.h"

#include <zephyr/kernel.h>

#include "status.h"
#include "led.h"

static int status_state = 0;

LOG_MODULE_REGISTER(status, LOG_LEVEL_INF);

static void status_thread(void);
K_THREAD_DEFINE(status_thread_id, 256, led_thread, NULL, NULL, NULL, 6, 0, 0);

void set_status(enum sys_status status, bool set)
{
	if (set)
	{
		status_state |= status;
		switch (status)
		{
		case SYS_STATUS_SENSOR_ERROR:
			LOG_ERR("Sensor communication error");
			break;
		case SYS_STATUS_CONNECTION_ERROR:
			LOG_WRN("Connection error");
			break;
		case SYS_STATUS_SYSTEM_ERROR:
			LOG_ERR("General error");
		default:
			break;
		}
	}
	else
	{
		status_state &= ~status;
	}
	LOG_INF("Status: %d", status_state);
}

static void status_thread(void)
{
	while (1) // cycle through errors
	{
		if (status_state & SYS_STATUS_SENSOR_ERROR)
		{
			set_led(SYS_LED_PATTERN_ERROR_A, SYS_LED_PRIORITY_STATUS);
			k_msleep(5000);
		}
		if (status_state & SYS_STATUS_CONNECTION_ERROR)
		{
			set_led(SYS_LED_PATTERN_ERROR_B, SYS_LED_PRIORITY_STATUS);
			k_msleep(5000);
		}
		if (status_state & SYS_STATUS_SYSTEM_ERROR)
		{
			set_led(SYS_LED_PATTERN_ERROR_C, SYS_LED_PRIORITY_STATUS);
			k_msleep(5000);
		}
		if (!status_state)
		{
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_STATUS);
			k_msleep(100);
		}
	}
}

bool status_ready(void) // true if no important status errors are active
{
	return (status_state & ~SYS_STATUS_CONNECTION_ERROR) == 0; // connection error is temporary, not critical
}
