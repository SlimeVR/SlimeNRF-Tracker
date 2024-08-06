#ifndef SLIMENRF_VQF
#define SLIMENRF_VQF

void vqf_init(float time);
void vqf_load(const void *data);
void vqf_save(void *data);

void vqf_update_accel(float *a, float time);
void vqf_update(float *g, float *a, float *m, float time);

void vqf_get_gyro_bias(float *g_off);
void vqf_set_gyro_bias(float *g_off);

void vqf_update_gyro_sanity(float *g, float *m);
int vqf_get_gyro_sanity(void);

void vqf_get_lin_a(float *lin_a);
void vqf_get_quat(float *q);

#endif