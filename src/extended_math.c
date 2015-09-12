#include "extended_math.h"

int16_t sqr( int16_t x )
{
	return x * x;
}

float sqrf( float x )
{
	return x * x;
}

void vector3f_to_rotation_matrix3f( matrix3f result, vector3f vector )
{
    result[ 0 ][ 0 ] =  0.;
    result[ 0 ][ 1 ] = -vector[ 2 ];
    result[ 0 ][ 2 ] =  vector[ 1 ];
    result[ 1 ][ 0 ] =  vector[ 2 ];
    result[ 1 ][ 1 ] =  0.;
    result[ 1 ][ 2 ] = -vector[ 0 ];
    result[ 2 ][ 0 ] = -vector[ 1 ];
    result[ 2 ][ 1 ] =  vector[ 0 ];
    result[ 2 ][ 2 ] =  0.;
}

int16_t vector3_length_squared( vector3 vector )
{
    return sqr( vector[ 0 ] ) + sqr( vector[ 1 ] ) + sqr( vector[ 2 ] );
}

void DCM_to_Euler_angles( float* roll, float* pitch, float* yaw, matrix3f DCM )
{
    *roll = atan2f( DCM[ 2 ][ 1 ], DCM[ 2 ][ 2 ] );
    *pitch = -asinf( DCM[ 2 ][ 0 ] );
    *yaw = atan2f( DCM[ 1 ][ 0 ], DCM[ 0 ][ 0 ] );
}
