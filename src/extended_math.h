#ifndef EXTENDED_MATH_H
#define EXTENDED_MATH_H

#include "math.h"
#include "stdint.h"

#define PI 3.14159265358979323846

typedef int16_t vector3[ 3 ];
typedef int32_t vector3_32[ 3 ];
typedef float vector3f[ 3 ];

typedef float matrix3f[ 3 ][ 3 ];

int16_t sqr( int16_t x );
float sqrf( float x );

void matrix3f_mult( matrix3f result, matrix3f a, matrix3f b );
void matrix3f_scale( matrix3f result, matrix3f matrix, float factor );
void matrix3f_sum( matrix3f result, matrix3f a, matrix3f b );

void vector3f_to_rotation_matrix3f( matrix3f result, vector3f vector );
int16_t vector3_length_squared( vector3 vector ); 

void vector3f_scale( vector3f result, vector3f vector, float factor );
void vector3f_sum( vector3f result, vector3f a, vector3f b );

void DCM_to_Euler_angles( float* roll, float* pitch, float* yaw, matrix3f DCM );

#endif
