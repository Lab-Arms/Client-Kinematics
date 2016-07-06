#ifndef _KINEMATICS_H
#	define _KINEMATICS_H

#include "matrix.h"

extern float cur_angle[3]; /* current servo motor angle */ 
extern int int_cur_angle[3]; /* current angles in the arduino */
extern float arm_len[3]; /* the length of each arm */

/*
Returns the relative angle dt = J^(-1) ds (the angle increment)
*/
extern void relative_angle (float dt[3], float angle[3], float len[3], float ds[3]);

/*
Returns the absolute angle t = t_old + dt
*/
extern void absolute_angle (float  t[3], float angle[3], float len[3], float ds[3]);

/*
Executes one step of the movement, but don't update the
angles. Return the distance between the new end-effector
coordinates and the desired end point.
*/
extern float inverse_kinematics (float dt[3], float end_xyz[3], float step_size, float len[3], float angle[3]);


/*
Routine for transforming degrees in radians.
*/
extern float rad2deg (float rad);

/*
Routine for transforming radians in degrees.
*/
extern float deg2rad (float deg);

/*
Update the angles to be sent.
*/
extern void update_int_angle (void);

/*
Update the cur_angles, which is used for the calculations.
*/
extern void update_cur_angle (void);

/*
Don't let the angles be less than 0 or 180.
*/
extern int limit (int num);

#endif
