#include "extended_math.h"

int16_t sqr( int16_t x )
{
	return x * x;
}

float sqrf( float x )
{
	return x * x;
}

void matrix3f_mult( matrix3f result, matrix3f a, matrix3f b )
{
    uint8_t i, j, k;
    
    for ( i = 0; i < 3; i++ ) {
        for ( j = 0; j < 3; j++ ) {
            result[ i ][ j ] = 0.;
        }
    }
    
    for ( i = 0; i < 3; i++ ) {
        for ( j = 0; j < 3; j++ ) {
            for ( k = 0; k < 3; k++ ) {
                result[ i ][ j ] += a[ i ][ k ] * b[ k ][ j ];
            }
        }
    }
}

void matrix3f_scale( matrix3f result, matrix3f a, float b )
{
    uint8_t i, j;
    
    for ( i = 0; i < 3; i++ ) {
        for ( j = 0; j < 3; j++ ) {
            result[ i ][ j ] = 0.;
        }
    }
    
    for ( i = 0; i < 3; i++ ) {
        for ( j = 0; j < 3; j++ ) {
            result[ i ][ j ] = a[ i ][ j ] * b;
        }
    }
}

void matrix3f_sum( matrix3f result, matrix3f a, matrix3f b )
{
    uint8_t i, j;
    
    for ( i = 0; i < 3; i++ ) {
        for ( j = 0; j < 3; j++ ) {
            result[ i ][ j ] = a[ i ][ j ] + b[ i ][ j ];
        }
    }
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

void vector3f_scale( vector3f result, vector3f vector, float factor )
{
    uint8_t i;
    
    for ( i = 0; i < 3; i++ ) {
        result[ i ] = vector[ i ] * factor;
    }
}

void vector3f_sum( vector3f result, vector3f a, vector3f b )
{
    uint8_t i;
    
    for ( i = 0; i < 3; i++ ) {
        result[ i ] = a[ i ] + b[ i ];
    }
}

void DCM_to_Euler_angles( float* roll, float* pitch, float* yaw, matrix3f DCM )
{
    *roll = atan2f( DCM[ 2 ][ 1 ], DCM[ 2 ][ 2 ] );
    *pitch = -asinf( DCM[ 2 ][ 0 ] );
    *yaw = atan2f( DCM[ 1 ][ 0 ], DCM[ 0 ][ 0 ] );
}
