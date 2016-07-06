#include "matrix.h"

const float zeros[3] = {0, 0, 0};
const float ones[3] = {1, 1, 1};

void int_assign (int res[3], int a[3])
{
	int i;

	for (i = 0; i < 3; i++)
		res[i] = a[i];
}

void float_assign (float res[3], float a[3])
{
	int i;

	for (i = 0; i < 3; i++)
		res[i] = a[i];
}

void matmul (float res[3], float mat[3][3], float vec[3])
{
	int i, j;

	for (i = 0; i < 3; i++) {
		res[i] = 0;
		for (j = 0; j < 3; j++)
			res[i] += mat[i][j] * vec[j];
	}
}

void inverse_jacobian (float res[3][3], float len[3], float theta[3])
{
	const float den_sin = sin(theta[2]-theta[1]);

	res[0][0] = 0;
	res[0][1] = 1;
	res[0][2] = 0;

	res[1][0] = cos(theta[2])/(len[0]*den_sin);
	res[1][1] = 0;
	res[1][2] = sin(theta[2])/(len[0]*den_sin);

	res[2][0] = cos(theta[1])/(len[2]*den_sin);
	res[2][1] = 0;
	res[2][2] = sin(theta[1])/(len[2]*den_sin);
}

void end_position (float res[3], float len[3], float theta[3])
{
	const float pi = M_PI;

	res[0] = len[0]*cos(theta[1]) + len[2]*cos(pi-theta[2]);
	res[1] = theta[0];
	res[2] = len[0]*sin(theta[1]) - len[2]*sin(pi-theta[2]);
}

void cart2cyl (float cyl[3], float cart[3])
{
	float x = cart[0];
	float y = cart[1];
	float z = cart[2];
	
	float s = sqrt(x*x + y*y);
	float phi = atan2(y, x);

	cyl[0] = s;
	cyl[1] = phi;
	cyl[2] = z;
}

void cyl2cart (float cart[3], float cyl[3])
{
	float s = cyl[0];
	float phi = cyl[1];
	float z = cyl[2];

	float x = s * cos(phi);
	float y = s * sin(phi);

	cart[0] = x;
	cart[1] = y;
	cart[2] = z;
}

void sum (float res[3], float a[3], float b[3])
{
	int i;

	for (i = 0; i < 3; i++)
		res[i] = a[i] + b[i];
}

void sub (float res[3], float a[3], float b[3])
{
	int i;

	for (i = 0; i < 3; i++)
		res[i] = a[i] - b[i];
}

void scalarmul (float res[3], float scalar, float a[3])
{
	int i;

	for (i = 0; i < 3; i++)
		res[i] = scalar * a[i];
}

float inner (float a[3], float b[3])
{
	int i;
	float res = 0;

	for (i = 0; i < 3; i++)
		res += a[i]*b[i];

	return res;
}

float norm (float a[3])
{
	return sqrt(inner(a, a));
}

void print_vec (const char* const msg, float a[3], FILE* stream)
{
	fprintf (stream, "%s%.4f %.4f %.4f\n", msg, a[0], a[1], a[2]);
}

void print_mat (const char* const msg, float a[3][3], FILE* stream)
{
	int i;

	fprintf (stream, "%s\n", msg);
	for (i = 0; i < 3; i++)
		fprintf (stream, "    %.4f %.4f %.4f\n", a[i][0], a[i][1], a[i][2]);
}
