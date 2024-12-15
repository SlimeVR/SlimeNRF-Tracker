#ifndef SLIMENRF_SYSTEM_POWER
#define SLIMENRF_SYSTEM_POWER

void sys_request_WOM(void);
void sys_request_system_off(void);
void sys_request_system_reboot(void);

bool vin_read(void);

#endif