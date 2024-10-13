#ifndef SLIMENRF_CONNECTION
#define SLIMENRF_CONNECTION

uint8_t connection_get_id(void);
void connection_set_id(uint8_t id);

void connection_update_sensor_ids(int imu_id, int mag_id);
void connection_update_sensor_data(float *q, float *a);
void connection_update_sensor_temp(float temp);
void connection_update_battery(bool battery_available, bool plugged, uint32_t battery_pptt, int battery_mV);
void connection_update_status(int status);

void connection_write_packet_0();
void connection_write_packet_1();
void connection_write_packet_2();
void connection_write_packet_3();

#endif
