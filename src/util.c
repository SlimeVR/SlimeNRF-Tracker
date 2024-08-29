#include <math.h>
#include <zephyr/kernel.h>

void q_normalize(const float *q, float *out)
{
	float mag = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (mag == 0)
		return;
	out[0] = q[0]/mag;
	out[1] = q[1]/mag;
	out[2] = q[2]/mag;
	out[3] = q[3]/mag;
}

void q_multiply(const float *x, const float *y, float *out)
{
	out[0] = x[0]*y[0] - x[1]*y[1] - x[2]*y[2] - x[3]*y[3];
	out[1] = x[1]*y[0] + x[0]*y[1] - x[3]*y[2] + x[2]*y[3];
	out[2] = x[2]*y[0] + x[3]*y[1] + x[0]*y[2] - x[1]*y[3];
	out[3] = x[3]*y[0] - x[2]*y[1] + x[1]*y[2] + x[0]*y[3];
}

void q_conj(const float *q, float *out)
{
	out[0] = q[0];
	out[1] = -q[1];
	out[2] = -q[2];
	out[3] = -q[3];
}

float q_diff_mag(const float *x, const float *y)
{
	float z[4];
	float q[4];
	q_conj(x, z);
	q_multiply(z, y, q);
	if (q[0] > 1)
		return 0;
	return fabsf(2 * acosf(q[0]));
}

float v_diff_mag(const float *a, const float *b)
{
	float x = a[0] - b[0];
	float y = a[1] - b[1];
	float z = a[2] - b[2];
	return sqrtf(x*x + y*y + z*z);
}

bool q_epsilon(const float *x, const float *y, float eps)
{
	float z[4];
	float q[4];
	q_conj(x, z);
	q_multiply(z, y, q);
	if (q[0] > 1)
		return true;
	return fabsf(2 * acosf(q[0])) < eps;
}

bool v_epsilon(const float *a, const float *b, float eps)
{
	float x = a[0] - b[0];
	float y = a[1] - b[1];
	float z = a[2] - b[2];
	return sqrtf(x*x + y*y + z*z) < eps;
}

// TODO: does this need to be moved?
void apply_BAinv(float xyz[3], float BAinv[4][3])
{
	float temp[3];
	for (int i = 0; i < 3; i++)
		temp[i] = xyz[i] - BAinv[0][i];
	xyz[0] = BAinv[1][0] * temp[0] + BAinv[1][1] * temp[1] + BAinv[1][2] * temp[2];
	xyz[1] = BAinv[2][0] * temp[0] + BAinv[2][1] * temp[1] + BAinv[2][2] * temp[2];
	xyz[2] = BAinv[3][0] * temp[0] + BAinv[3][1] * temp[1] + BAinv[3][2] * temp[2];
}
