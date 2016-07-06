#ifndef _MATRIX_H
#define _MATRIX_H

#include <math.h>
#include <stdio.h>

extern const float zeros[3]; // a vector of zeros
extern const float ones[3]; // a vector of ones

extern void int_assign (int res[3], int a[3]); // MATLAB: res = a;
extern void float_assign (float res[3], float a[3]); // MATLAB: res = a;
extern void matmul (float res[3], float mat[3][3], float vec[3]); // MATLAB: res = mat * vec; 
extern void inverse_jacobian (float res[3][3], float len[3], float theta[3]); // res is the inverse of the Jacobian matrix
extern void end_position (float res[3], float len[3], float theta[3]); // res is the position of the end-effector in cylindrical coordinates

extern void cart2cyl (float cyl[3], float cart[3]); // MATLAB: cyl = cart2pol(cart);
extern void cyl2cart (float cart[3], float cyl[3]); // MATLAB: cart = pol2cart(cyl);

extern void sum (float res[3], float a[3], float b[3]); // MATLAB: res = a + b;
extern void sub (float res[3], float a[3], float b[3]); // MATLAB: res = a - b;
extern void scalarmul (float res[3], float scalar, float a[3]); // MATLAB: res = scalar * a;

extern float inner (float a[3], float b[3]); // returns the inner product of vectors a and b
extern float norm (float a[3]); // returns the 2-norm of vector a

extern void print_vec (const char* const msg, float a[3], FILE* stream); // print vector components
extern void print_mat (const char* const msg, float a[3][3], FILE* stream); // print matrix components

#endif
