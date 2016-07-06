#include "kinematics.h"

float cur_angle[3];
int int_cur_angle[3];
float arm_len[3] = {16, 4, 16};

/*
Returns the relative angle dt = J^(-1) ds (the angle increment)
*/
void relative_angle (float dt[3], float angle[3], float len[3], float ds[3])
{
	float inv[3][3];

	inverse_jacobian (inv, len, angle); // MATLAB: inv = inverse_jacobian (len, angle);
	matmul (dt, inv, ds); // MATLAB: dt = inv * ds;
}

/*
Returns the absolute angle t = t_old + dt
*/
void absolute_angle (float t[3], float angle[3], float len[3], float ds[3])
{
	float dt[3];

	relative_angle (dt, angle, len, ds);
	sum (t, t, dt); // MATLAB: t = t + dt;
}

/*
Executes one step of the movement, but don't update the
angles. Return the distance between the new end-effector
coordinates and the desired end point.
*/
float inverse_kinematics (float t[3], float end_xyz[3], float step_size, float len[3], float angle[3])
{
	float ds[3];
	float end_spz[3];
	float current_pos[3];
	float new_pos_spz[3];
	float new_pos_xyz[3];
	float rel_vec[3];

	end_position (current_pos, len, angle);
	cart2cyl (end_spz, end_xyz);
	sub (ds, end_spz, current_pos);
	scalarmul (ds, step_size/norm(ds), ds);

	absolute_angle (t, angle, len, ds);

	end_position (new_pos_spz, len, t);
	cyl2cart (new_pos_xyz, new_pos_spz);
	sub (rel_vec, end_xyz, new_pos_xyz);

	return norm(rel_vec);
}

/*
Don't let the angles be less than 0 or 180.
*/
int limit (int num)
{
	if (num < 0) return 0;
	if (num > 180) return 180;
	return num;
}

/*
Routine for transforming degrees in radians.
*/
float rad2deg (float rad)
{
	return rad * 180. / M_PI;
}

/*
Routine for transforming radians in degrees.
*/
float deg2rad (float deg)
{
	return deg * M_PI / 180.;
}

/*
Update the integer angles.
*/
void update_int_angle (void)
{
	int i;

	for (i = 0; i < 3; i++)
		int_cur_angle[i] = limit((int) rad2deg(cur_angle[i]));
}

/*
Update the cur_angles, which is used for the calculations.
*/
void update_cur_angle (void)
{
	int i;

	for (i = 0; i < 3; i++)
		cur_angle[i] = deg2rad((float) int_cur_angle[i]);
}
