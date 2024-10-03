#ifndef SLIMENRF_GLOBALS
#define SLIMENRF_GLOBALS

#include <zephyr/logging/log.h>

#include "retained.h"

#define USER_SHUTDOWN_ENABLED true // Allow user to use reset or sw0 to shutdown
#define MAG_ENABLED true // Use magnetometer if it is present
#define IGNORE_RESET true // If sw0 available, don't change any reset behavior
//#define WOM_USE_DCDC true // Use DCDC instead of LDO for WOM if it is more efficient

extern float q3[4]; // correction quat

#endif