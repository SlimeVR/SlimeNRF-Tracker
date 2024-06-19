#ifndef SLIMENRF_ESB
#define SLIMENRF_ESB

#include "globals.h"

void event_handler(struct esb_evt const *event);
int clocks_start(void);
int esb_initialize(void);
int esb_initialize_rx(void);

#endif