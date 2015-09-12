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

#define VECTOR_SUM( a, b, result ) \
    do { \
        uint8_t i; \
        for ( i = 0; i < 3; ++i ) { \
            ( result )[ i ] = ( a )[ i ] + ( b )[ i ]; \
        } \
    } while( 0 )

#define VECTOR_SCALE( vector, factor, result ) \
    do { \
        uint8_t i; \
        for ( i = 0; i < 3; ++i ) { \
            ( result )[ i ] = ( vector )[ i ] * ( factor ); \
        } \
    } while( 0 )

#define MATRIX_SUM( a, b, result ) \
    do { \
        uint8_t i, j; \
        for ( i = 0; i < 3; ++i ) { \
            for ( j = 0; j < 3; ++j ) { \
                ( result )[ i ][ j ] = ( a )[ i ][ j ] + ( b )[ i ][ j ]; \
            } \
        } \
    } while( 0 )

#define MATRIX_SCALE( matrix, factor, result ) \
    do { \
        uint8_t i, j; \
        for ( i = 0; i < 3; ++i ) { \
            for ( j = 0; j < 3; ++j ) { \
                ( result )[ i ][ j ] = ( matrix )[ i ][ j ] * ( factor ); \
            } \
        } \
    } while( 0 )

#define MATRIX_MULT( a, b, result ) \
    do { \
        uint8_t i, j, k; \
        for ( i = 0; i < 3; i++ ) { \
            for ( j = 0; j < 3; j++ ) { \
                for ( k = 0; k < 3; k++ ) { \
                    ( result )[ i ][ j ] += ( a )[ i ][ k ] * ( b )[ k ][ j ]; \
                } \
            } \
        } \
    } while( 0 )

void vector3f_to_rotation_matrix3f( matrix3f result, vector3f vector );
int16_t vector3_length_squared( vector3 vector ); 

void DCM_to_Euler_angles( float* roll, float* pitch, float* yaw, matrix3f DCM );

#endif
