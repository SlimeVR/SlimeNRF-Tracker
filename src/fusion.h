#ifndef SLIMENRF_FUSION
#define SLIMENRF_FUSION

void fusion_init(unsigned int rate);
void fusion_load(const void *data);
void fusion_save(void *data);

void fusion_update_accel(float *a, float time);
void fusion_update(float *g, float *a, float *m, float time);

void fusion_get_gyro_bias(float *g_off);
void fusion_set_gyro_bias(float *g_off);

void fusion_update_gyro_sanity(float *g, float *m);
int fusion_get_gyro_sanity(void);

void fusion_get_lin_a(float *lin_a);
void fusion_get_quat(float *q);

#endif