#include "globals.h"
#include "util.h"
#include "esb.h"

static uint8_t tracker_id, batt, batt_v;

uint8_t connection_get_id(void)
{
	return tracker_id;
}
void connection_set_id(uint8_t id)
{
	tracker_id = id;
}

void connection_update_battery(bool battery_available, bool plugged, uint32_t battery_pptt, int battery_mV) // format for packet send
{
	if (!battery_available) // No battery, and voltage is <=1500mV
	{
		batt = 0;
		batt_v = 0;
		return;
	}

	battery_pptt /= 100;
	batt = battery_pptt;
	if (batt < 1) // Clamp to 1% (because server sees 0% as "no battery")
		batt = 1;

	if (plugged) // Charging
	{
		batt_v = 255; // server will show a charging indicator
		return;
	}

	battery_mV /= 10;
	battery_mV -= 245;
	if (battery_mV < 0) // Very dead but it is what it is
		batt_v = 0;
	else if (battery_mV > 255)
		batt_v = 255;
	else
		batt_v = battery_mV; // 0-255 -> 2.45-5.00V
}

void connection_write_packet_0(float *q, float *a)
{
	uint8_t data[20] = {0};
	data[0] = 0; // TODO: packet id
	data[1] = tracker_id << 4; // TODO: this is a maximum of 16 trackers, each can have 16 imus. why? This could be changed to 32 trackers/8 imus, or just get rid of the idea of extensions.
	//data[2] = batt | (charging ? 128 : 0);
	// TODO: Send temperature (time to add another packet?)
	data[2] = batt;
	data[3] = batt_v;
	uint16_t *buf = (uint16_t *)&data[4];
	buf[0] = TO_FIXED_15(q[1]);
	buf[1] = TO_FIXED_15(q[2]);
	buf[2] = TO_FIXED_15(q[3]);
	buf[3] = TO_FIXED_15(q[0]);
	buf[4] = TO_FIXED_7(a[0]);
	buf[5] = TO_FIXED_7(a[1]);
	buf[6] = TO_FIXED_7(a[2]);
	esb_write(data);
}
