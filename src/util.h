#ifndef SLIMENRF_UTILS
#define SLIMENRF_UTILS

#include <math.h>
#include <zephyr/kernel.h>

// Saturate int to 16 bits
// Optimized to a single ARM assembler instruction
#define SATURATE_INT16(x) ((x) > 32767 ? 32767 : ((x) < -32768 ? -32768 : (x)))

#define TO_FIXED_15(x) ((int16_t)SATURATE_INT16((x) * (1 << 15)))
#define TO_FIXED_10(x) ((int16_t)((x) * (1 << 10)))
#define TO_FIXED_7(x) ((int16_t)SATURATE_INT16((x) * (1 << 7)))
#define FIXED_14_TO_DOUBLE(x) (((double)(x)) / (1 << 14))
#define FIXED_10_TO_DOUBLE(x) (((double)(x)) / (1 << 10))
#define FIXED_7_TO_DOUBLE(x) (((double)(x)) / (1 << 7))

#define CONST_EARTH_GRAVITY 9.80665

void q_multiply(float *x, float *y, float *out) {
	out[0] = x[0]*y[0] - x[1]*y[1] - x[2]*y[2] - x[3]*y[3];
	out[1] = x[1]*y[0] + x[0]*y[1] - x[3]*y[2] + x[2]*y[3];
	out[2] = x[2]*y[0] + x[3]*y[1] + x[0]*y[2] - x[1]*y[3];
	out[3] = x[3]*y[0] - x[2]*y[1] + x[1]*y[2] + x[0]*y[3];
}

void q_conj(float *q, float *out) {
	out[0] = q[0];
	out[1] = -q[1];
	out[2] = -q[2];
	out[3] = -q[3];
}

float q_diff_mag(float *x, float *y) {
	float z[4];
	float q[4];
	q_conj(x, z);
	q_multiply(z, y, q);
	return fabsf(2 * acosf(q[0]));
}

float v_diff_mag(float *a, float *b) {
	float x = a[0] - b[0];
	float y = a[1] - b[1];
	float z = a[2] - b[2];
	return sqrtf(x*x + y*y + z*z);
}

bool q_epsilon(float *x, float *y, float eps) {
	float z[4];
	float q[4];
	q_conj(x, z);
	q_multiply(z, y, q);
	return fabsf(2 * acosf(q[0])) < eps;
}

bool v_epsilon(float *a, float *b, float eps) {
	float x = a[0] - b[0];
	float y = a[1] - b[1];
	float z = a[2] - b[2];
	return sqrtf(x*x + y*y + z*z) < eps;
}

// TODO: fix this mess
bool quat_epsilon(float *q, float *q2) {
	return q_epsilon(q, q2,0.0002);
}

bool quat_epsilon_coarse(float *q, float *q2) {
	return q_epsilon(q, q2, 0.001);
}

bool quat_epsilon_coarse2(float *q, float *q2) {
	return q_epsilon(q, q2, 0.01);
}

bool vec_epsilon(float *a, float *a2) {
	return v_epsilon(a, a2, 0.1);
}

bool vec_epsilon2(float *a, float *a2, float eps) {
	return v_epsilon(a, a2, eps);
}

void apply_BAinv(float xyz[3], float BAinv[4][3]) {
	float temp[3];
	for (int i = 0; i < 3; i++)
		temp[i] = xyz[i] - BAinv[0][i];
	xyz[0] = BAinv[1][0] * temp[0] + BAinv[1][1] * temp[1] + BAinv[1][2] * temp[2];
	xyz[1] = BAinv[2][0] * temp[0] + BAinv[2][1] * temp[1] + BAinv[2][2] * temp[2];
	xyz[2] = BAinv[3][0] * temp[0] + BAinv[3][1] * temp[1] + BAinv[3][2] * temp[2];
}

#endif