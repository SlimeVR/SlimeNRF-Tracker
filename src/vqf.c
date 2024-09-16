#include "globals.h"
#include "util.h"

#include "../vqf-c/src/vqf.h"

#include "vqf.h"

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884f
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0f)
#endif

vqf_params_t params;
vqf_state_t state;
vqf_coeffs_t coeffs;

void vqf_init(float time)
{
	// TODO: store vqf state to retained on sensor write
	init_params(&params);
	params.biasSigmaInit = 1.0f; // naive values from https://github.com/kounocom/SlimeVR-Tracker-ESP/blob/dynamic-sfusion/src/sensors/SensorFusion.h
	params.biasForgettingTime = 30.0f;
	params.biasSigmaMotion = 0.1175f;
	params.biasVerticalForgettingFactor = 0;
	params.biasSigmaRest = 0.007f;
	params.restThGyr = 1.0f; // 400 norm
	params.restThAcc = 0.196f; // 100 norm
	initVqf(&params, &state, &coeffs, time, 0, 0);
}

void vqf_load(const void *data)
{
	memcpy(&state, data, sizeof(state));
}

void vqf_save(void *data)
{
	memcpy(data, &state, sizeof(state));
}

void vqf_update_accel(float *a, float time)
{
	// TODO: time unused?
	// TODO: how to handle change in sample rate
	updateAcc(&params, &state, &coeffs, a);
}

void vqf_update(float *g, float *a, float *m, float time)
{
	float g_rad[3] = {0};
	// g is in deg/s, convert to rad/s
	for (int i = 0; i < 3; i++)
		g_rad[i] = g[i] * DEG_TO_RAD;
	// TODO: time unused?
	// TODO: gyro is a different rate to the others, should they be separated
	updateGyr(&params, &state, &coeffs, g_rad);
	updateAcc(&params, &state, &coeffs, a);
	updateMag(&params, &state, &coeffs, m);
}

void vqf_get_gyro_bias(float *g_off)
{
	getBiasEstimate(&state, &coeffs, g_off);
}

void vqf_set_gyro_bias(float *g_off)
{
	setBiasEstimate(&state, g_off, -1);
}

void vqf_update_gyro_sanity(float *g, float *m)
{
	// TODO: does vqf tell us a "recovery state"
	return;
}

int vqf_get_gyro_sanity(void)
{
	// TODO: does vqf tell us a "recovery state"
	return 0;
}

void vqf_get_lin_a(float *lin_a)
{
	float q[4] = {0};
	vqf_get_quat(q);

	float vec_gravity[3] = {0};
	vec_gravity[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
	vec_gravity[1] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
	vec_gravity[2] = 2.0f * (q[0] * q[0] - 0.5f + q[3] * q[3]);

	float *a = state.lastAccLp;
	for (int i = 0; i < 3; i++)
	{
		lin_a[i] = a[i] -vec_gravity[i];
	}
}

void vqf_get_quat(float *q)
{
	getQuat9D(&state, q);
}

const sensor_fusion_t sensor_fusion_vqf = {
	*vqf_init,
	*vqf_load,
	*vqf_save,

	*vqf_update_accel,
	*vqf_update,

	*vqf_get_gyro_bias,
	*vqf_set_gyro_bias,

	*vqf_update_gyro_sanity,
	*vqf_get_gyro_sanity,

	*vqf_get_lin_a,
	*vqf_get_quat
};
