#include "globals.h"
#include "util.h"

#include "../Fusion/Fusion/Fusion.h"
#include "Fusion/FusionOffset2.h"

FusionOffset offset; // could share goff and q with fusionoffset and fusionahrs but init clears the values
FusionAhrs ahrs;

FusionVector gyro_sanity_m;
int gyro_sanity = 0;

LOG_MODULE_REGISTER(fusion, LOG_LEVEL_INF);

void fusion_init(float time)
{
	unsigned int rate = 1.0f / time;
	FusionOffsetInitialise2(&offset, rate);
	FusionAhrsInitialise(&ahrs);
	// ahrs.initialising = true; // cancel fusion init, maybe only if there is a quat stored? oh well
	const FusionAhrsSettings settings = {
			.convention = FusionConventionNwu,
			.gain = 0.5f,
			.gyroscopeRange = 2000.0f, // also change gyro range in fusion! (.. does it actually work if its set to the limit?)
			.accelerationRejection = 10.0f,
			.magneticRejection = 20.0f,
			.recoveryTriggerPeriod = 5 * rate, // 5 seconds
	};
	FusionAhrsSetSettings(&ahrs, &settings);
}

void fusion_load(const void *data)
{
	memcpy(&ahrs, data, sizeof(ahrs));
	memcpy(&offset, (uint8_t *)data + sizeof(ahrs), sizeof(offset));
}

void fusion_save(void *data)
{
	memcpy(data, &ahrs, sizeof(ahrs));
	memcpy((uint8_t *)data + sizeof(ahrs), &offset, sizeof(offset));
}

void fusion_update_accel(float *a, float time)
{
    FusionVector vec_a = {.array = {a[0], a[1], a[2]}};

	ahrs.initialising = true;
	ahrs.rampedGain = 10.0f;
	ahrs.accelerometerIgnored = false;
	ahrs.accelerationRecoveryTrigger = 0;
	ahrs.accelerationRecoveryTimeout = 0;

	FusionAhrsUpdate(&ahrs, FUSION_VECTOR_ZERO, vec_a, FUSION_VECTOR_ZERO, time);
}

void fusion_update(float *g, float *a, float *m, float time)
{
    FusionVector vec_g = {.array = {g[0], g[1], g[2]}};
    FusionVector vec_a = {.array = {a[0], a[1], a[2]}};
    FusionVector vec_m = {.array = {m[0], m[1], m[2]}};

	FusionVector g_off = FusionOffsetUpdate2(&offset, vec_g);
    
	if (offset.timer < offset.timeout) // Don't fuse gyro if not moving
		FusionAhrsUpdate(&ahrs, g_off, vec_a, vec_m, time);
	else
		FusionAhrsUpdate(&ahrs, FUSION_VECTOR_ZERO, vec_a, vec_m, time);
}

void fusion_get_gyro_bias(float *g_off)
{
	memcpy(g_off, offset.gyroscopeOffset.array, sizeof(offset.gyroscopeOffset.array));
}

void fusion_set_gyro_bias(float *g_off)
{
	memcpy(offset.gyroscopeOffset.array, g_off, sizeof(offset.gyroscopeOffset.array));
}

void fusion_update_gyro_sanity(float *g, float *m)
{
	if (FusionAhrsGetFlags(&ahrs).magneticRecovery)
	{
		if (gyro_sanity == 2 && vec_epsilon2(gyro_sanity_m.array, m, 0.01f))
		{
			// For whatever reason the gyro seems unreliable
			// Reset the offset here so the tracker can probably at least turn off
			// TODO: the gyroscope might output garbage, the data should be ignored entirely
			LOG_WRN("Gyroscope may be unreliable");
			fusion_set_gyro_bias(g);
			gyro_sanity = 3;
		}
		else if (gyro_sanity % 2 == 0)
		{
			LOG_WRN("Fusion magnetic recovery active");
            memcpy(gyro_sanity_m.array, m, sizeof(gyro_sanity_m.array));
			gyro_sanity = 1;
		}
	}
	else if (gyro_sanity == 1) 
	{
		LOG_DBG("Fusion recovered once");
		gyro_sanity = 2;
	}
	else if (gyro_sanity == 3)
	{
		LOG_DBG("Gyroscope sanity reset");
		gyro_sanity = 0;
	}
}

int fusion_get_gyro_sanity(void)
{
	return gyro_sanity;
}

void fusion_get_lin_a(float *lin_a)
{
	const FusionVector ahrs_lin_a = FusionAhrsGetLinearAcceleration(&ahrs); // im going insane
	lin_a[0] = ahrs_lin_a.array[0] * CONST_EARTH_GRAVITY; // Also change to m/s for SlimeVR server
	lin_a[1] = ahrs_lin_a.array[1] * CONST_EARTH_GRAVITY;
	lin_a[2] = ahrs_lin_a.array[2] * CONST_EARTH_GRAVITY;
}

void fusion_get_quat(float *q)
{
	memcpy(q, ahrs.quaternion.array, sizeof(ahrs.quaternion.array));
}