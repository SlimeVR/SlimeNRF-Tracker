#include "../../globals.h"
#include "../../util.h"

#include "../../../Fusion/Fusion/Fusion.h"

#include "sensorfusion.h"

LOG_MODULE_REGISTER(sensorfusion, LOG_LEVEL_INF);

void sensorfusion_init(float time)
{
}

void sensorfusion_load(const void *data)
{
}

void sensorfusion_save(void *data)
{
}

void sensorfusion_update_accel(float *a, float time)
{
}

void sensorfusion_update(float *g, float *a, float *m, float time)
{
}

void sensorfusion_get_gyro_bias(float *g_off)
{
}

void sensorfusion_set_gyro_bias(float *g_off)
{
}

void sensorfusion_update_gyro_sanity(float *g, float *m)
{
}

int sensorfusion_get_gyro_sanity(void)
{
    return -1;
}

void sensorfusion_get_lin_a(float *lin_a)
{
}

void sensorfusion_get_quat(float *q)
{
}

const sensor_fusion_t sensor_fusion_sensorfusion = {
	*sensorfusion_init,
	*sensorfusion_load,
	*sensorfusion_save,

	*sensorfusion_update_accel,
	*sensorfusion_update,

	*sensorfusion_get_gyro_bias,
	*sensorfusion_set_gyro_bias,

	*sensorfusion_update_gyro_sanity,
	*sensorfusion_get_gyro_sanity,

	*sensorfusion_get_lin_a,
	*sensorfusion_get_quat
};
