#ifndef SLIMENRF_UTILS
#define SLIMENRF_UTILS

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884f
#endif

#ifndef EPS
#define EPS 1e-6
#endif

// Saturate int to 16 bits
// Optimized to a single ARM assembler instruction
#define SATURATE_INT16(x) ((x) > 32767 ? 32767 : ((x) < -32768 ? -32768 : (x)))

#define SATURATE_UINT11(x) ((x) > 2047 ? 2047 : ((x) < 0 ? 0 : (x)))
#define SATURATE_UINT10(x) ((x) > 1023 ? 1023 : ((x) < 0 ? 0 : (x)))

#define TO_FIXED_15(x) ((int16_t)SATURATE_INT16((x) * (1 << 15)))
#define TO_FIXED_11(x) ((int16_t)((x) * (1 << 11)))
#define TO_FIXED_10(x) ((int16_t)((x) * (1 << 10)))
#define TO_FIXED_7(x) ((int16_t)SATURATE_INT16((x) * (1 << 7)))
#define FIXED_15_TO_DOUBLE(x) (((double)(x)) / (1 << 15))
#define FIXED_11_TO_DOUBLE(x) (((double)(x)) / (1 << 11))
#define FIXED_10_TO_DOUBLE(x) (((double)(x)) / (1 << 10))
#define FIXED_7_TO_DOUBLE(x) (((double)(x)) / (1 << 7))

#define CONST_EARTH_GRAVITY 9.80665

void q_normalize(const float *q, float *out);
void q_multiply(const float *x, const float *y, float *out);
void q_conj(const float *q, float *out);
void q_negate(const float *q, float *out);
float q_diff_mag(const float *x, const float *y);
float v_diff_mag(const float *a, const float *b);
bool q_epsilon(const float *x, const float *y, float eps);
bool v_epsilon(const float *a, const float *b, float eps);

// TODO: does this need to be moved?
void apply_BAinv(float xyz[3], float BAinv[4][3]);

void q_fem(const float *q, float *out);
void q_iem(const float *v, float *out);

#endif