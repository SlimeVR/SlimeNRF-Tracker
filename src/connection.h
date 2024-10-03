#ifndef SLIMENRF_CONNECTION
#define SLIMENRF_CONNECTION

uint8_t connection_get_id(void);
void connection_set_id(uint8_t id);

void connection_update_battery(bool battery_available, bool plugged, uint32_t battery_pptt, int battery_mV);

void connection_write_packet_0(float *q, float *a); // TODO: give packets some names

#endif
