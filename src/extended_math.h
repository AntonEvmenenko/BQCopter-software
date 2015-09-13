#ifndef EXTENDED_MATH_H
#define EXTENDED_MATH_H

#include "math.h"
#include "stdint.h"

#define PI 3.14159265358979323846

typedef int16_t vector3i16[ 3 ];
typedef int32_t vector3i32[ 3 ];
typedef float vector3f[ 3 ];
typedef float matrix3f[ 3 ][ 3 ];

typedef struct 
{
    float q0;
    float q1;
    float q2;
    float q3;    
} quaternionf;

#define EMPTY_VECTOR3 { 0, 0, 0 }
#define EMPTY_MATRIX3 { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } }

#define VECTOR3_SUM( result, a, b ) \
    do { \
        uint8_t i; \
        for ( i = 0; i < 3; ++i ) { \
            ( result )[ i ] = ( a )[ i ] + ( b )[ i ]; \
        } \
    } while( 0 )

#define VECTOR3_SCALE( result, vector, factor ) \
    do { \
        uint8_t i; \
        for ( i = 0; i < 3; ++i ) { \
            ( result )[ i ] = ( vector )[ i ] * ( factor ); \
        } \
    } while( 0 )
    
#define VECTOR3_COPY( dst, src ) \
    do { \
        uint8_t i; \
        for ( i = 0; i < 3; ++i ) { \
            ( dst )[ i ] = ( src )[ i ]; \
        } \
    } while( 0 )

#define MATRIX3_SUM( result, a, b ) \
    do { \
        uint8_t i, j; \
        for ( i = 0; i < 3; ++i ) { \
            for ( j = 0; j < 3; ++j ) { \
                ( result )[ i ][ j ] = ( a )[ i ][ j ] + ( b )[ i ][ j ]; \
            } \
        } \
    } while( 0 )

#define MATRIX3_SCALE( result, matrix, factor ) \
    do { \
        uint8_t i, j; \
        for ( i = 0; i < 3; ++i ) { \
            for ( j = 0; j < 3; ++j ) { \
                ( result )[ i ][ j ] = ( matrix )[ i ][ j ] * ( factor ); \
            } \
        } \
    } while( 0 )

#define MATRIX3_MULT( result, a, b ) \
    do { \
        uint8_t i, j, k; \
        for ( i = 0; i < 3; i++ ) { \
            for ( j = 0; j < 3; j++ ) { \
                ( result )[ i ][ j ] = 0; \
                for ( k = 0; k < 3; k++ ) { \
                    ( result )[ i ][ j ] += ( a )[ i ][ k ] * ( b )[ k ][ j ]; \
                } \
            } \
        } \
    } while( 0 )

int16_t sqr( int16_t x );
float sqrf( float x );
void vector3f_to_rotation_matrix3f( matrix3f result, vector3f vector );
int16_t vector3i16_length_squared( vector3i16 vector ); 
void DCM_to_Euler_angles( float* roll, float* pitch, float* yaw, matrix3f DCM );
float vector3f_dot( vector3f a, vector3f b );
void vector3f_cross( vector3f result, vector3f a, vector3f b );
void matrix3f_normalize( matrix3f matrix );
void quaternionf_to_Euler_angles( float* roll, float* pitch, float* yaw, quaternionf q );

#endif
