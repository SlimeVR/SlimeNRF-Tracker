#ifndef SLIMENRF_ESB
#define SLIMENRF_ESB

#include <esb.h>
#include <nrfx_timer.h>

// TODO: timer?
#define LAST_RESET_LIMIT 10
extern uint8_t last_reset;
// TODO: move to esb/timer
extern const nrfx_timer_t m_timer;
extern bool esb_state;
extern bool timer_state;
extern bool send_data;
// TODO: esb/sensor?
extern uint16_t led_clock;
extern uint32_t led_clock_offset;

void event_handler(struct esb_evt const *event);
int clocks_start(void);
int esb_initialize(bool);

void esb_set_addr_discovery(void);
void esb_set_addr_paired(void);

void esb_pair(void);
void esb_reset_pair(void);

void esb_write(uint8_t *data); // TODO: give packets some names

#endif