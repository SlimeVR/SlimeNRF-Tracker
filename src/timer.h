#ifndef SLIMENRF_TIMER
#define SLIMENRF_TIMER

#include <nrfx_timer.h>

void timer_handler(nrf_timer_event_t event_type, void *p_context);
void timer_init(void);

#endif