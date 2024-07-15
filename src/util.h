#ifndef SLIMENRF_UTILS
#define SLIMENRF_UTILS

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

void q_multiply(float *x, float *y, float *out);
void q_conj(float *q, float *out);
float q_diff_mag(float *x, float *y);
float v_diff_mag(float *a, float *b);
bool q_epsilon(float *x, float *y, float eps);
bool v_epsilon(float *a, float *b, float eps);

// TODO: fix this mess
bool quat_epsilon(float *q, float *q2);
bool quat_epsilon_coarse(float *q, float *q2);
bool quat_epsilon_coarse2(float *q, float *q2);
bool vec_epsilon(float *a, float *a2);
bool vec_epsilon2(float *a, float *a2, float eps);
void apply_BAinv(float xyz[3], float BAinv[4][3]);

#endif