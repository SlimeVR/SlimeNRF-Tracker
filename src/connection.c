#include "globals.h"
#include "util.h"
#include "esb.h"

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
//	memcpy(data[4], buf, sizeof(buf));
    esb_write(data);
}
