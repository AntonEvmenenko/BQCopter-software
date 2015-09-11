#ifndef EXTENDED_MATH_H
#define EXTENDED_MATH_H

#include "math.h"
#include "stdint.h"

typedef int16_t vector3[ 3 ];
typedef float vector3f[ 3 ];

typedef float matrix3f[ 3 ][ 3 ];

int16_t sqr( int16_t x );
float sqrf( float x );

void matrix3f_matrix3f_multiplication( matrix3f result, matrix3f a, matrix3f b );
void matrix3f_scalar_multiplication( matrix3f result, matrix3f a, float b );
void matrix3f_matrix3f_sum( matrix3f result, matrix3f a, matrix3f b );
void vector3f_to_rotation_matrix3f( matrix3f result, vector3f vector );
int16_t vector3_length_squared( vector3 vector ); 

void DCM_to_Euler_angles( float* roll, float* pitch, float* yaw, matrix3f DCM );

#endif
