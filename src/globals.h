#ifndef SLIMENRF_GLOBALS
#define SLIMENRF_GLOBALS

#include <zephyr/logging/log.h>

#include "retained.h"

#define USER_SHUTDOWN_ENABLED true // Allow user to use reset or sw0 to shutdown
#define MAG_ENABLED true // Use magnetometer if it is present
#define IGNORE_RESET true // If sw0 available, don't change any reset behavior
//#define WOM_USE_DCDC true // Use DCDC instead of LDO for WOM if it is more efficient

#define SENSOR_GYROSCOPE_AXES_ALIGNMENT gx, -gz, gy // gyro alignment
#define SENSOR_ACCELEROMETER_AXES_ALIGNMENT ax, -az, ay // accel alignment
#define SENSOR_MAGNETOMETER_AXES_ALIGNMENT my, mz, -mx // mag alignment
#define SENSOR_QUATERNION_CORRECTION 0.5f, -0.5f, -0.5f, -0.5f // correction quat

#endif