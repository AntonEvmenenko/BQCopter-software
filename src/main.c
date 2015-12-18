#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "time.h"
#include "L3G4200D.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "NRF24L.h"
#include "ESC.h"
#include "extended_math.h"
#include "MadgwickAHRS.h"
#include "UART.h"

const float k_gyroscope = 90. * PI / ( 67. * 20. * 180. ); // correction for gyro; for angle in radians
const float kp_rp = 2500, kd_rp = 600, ki_rp = 0.; // PD regulator components for roll and pitch
const float kp_y = 5000, kd_y = 500, ki_y = 0.; // PD regulator components for yaw
const int16_t k_u_throttle = 4, k_u_roll_pitch = 5; // factors for roll and pitch controls
const int16_t k_u_camera = 2; // factors for camera control
const int16_t calibration_iterations_count = 2000; // gyroscope and accelerometer calibration
const int16_t NRF24L_watchdog_initial = 10;

const float max_abs_integral_error = .5;
vector3f integral_error = EMPTY_VECTOR3;

unsigned long previous_timestamp_us = 0, dt_us = 0;
float dt_s = 0.;

int16_t calibration_counter = calibration_iterations_count;
uint8_t calibration = 1;
int16_t NRF24L_watchdog = NRF24L_watchdog_initial;

int16_t u_throttle = 0, u_roll = 0, u_pitch = 0; 
int16_t camera_control_enabled = 0;

vector3i16 gyroscope = EMPTY_VECTOR3, accelerometer = EMPTY_VECTOR3, compass = EMPTY_VECTOR3;
vector3i32 gyroscope_sum = EMPTY_VECTOR3, accelerometer_sum = EMPTY_VECTOR3;
vector3f gyroscope_correction = EMPTY_VECTOR3;
vector3f accelerometer_average = EMPTY_VECTOR3, accelerometer_correction = EMPTY_VECTOR3;
vector3f Euler_angles = EMPTY_VECTOR3, Euler_angles_previous = EMPTY_VECTOR3;


int main(void)
{
    __enable_irq();

    char raddr[ 5 ] = "serv2", taddr[ 5 ] = "serv1";
    NRF24L_init( raddr, taddr, 90, 4 );
    ESC_init( );
    UART_init();
    SysTick_init( );
    I2C_init( I2C1, 200000 );
    setupL3G4200D( 2000 );
    setupADXL345( );
    //setupHMC5883L( );
	
    delay_ms( 500 );
    
    while( 1 )
    {		 
        if ( NRF24L_data_ready( ) ) {
            uint8_t data[ 4 ];
            NRF24L_get_data( data );
            u_throttle = data[ 0 ];
            u_roll = data[ 1 ] - 128;
            u_pitch = data[ 2 ] - 128;
            camera_control_enabled = data[ 3 ];
            NRF24L_watchdog = NRF24L_watchdog_initial;
        }
        
        if ( !( NRF24L_watchdog ? NRF24L_watchdog-- : 0 ) ) {
            u_throttle = 0;
            u_roll = 0;
            u_pitch = 0;
            camera_control_enabled = 0;
        }
        
        getGyroValues( gyroscope, gyroscope + 1, gyroscope + 2 );
        getAccelValues( accelerometer, accelerometer + 1, accelerometer + 2 );
        //getRawCompassValues( compass, compass + 1, compass + 2 );

        if ( calibration ) {
            VECTOR3_SUM( gyroscope_sum, gyroscope_sum, gyroscope );
            VECTOR3_SUM( accelerometer_sum, accelerometer_sum, accelerometer );

            if ( !( calibration_counter ? calibration_counter-- : 0 ) ) {
                calibration = 0;
                VECTOR3_SCALE( gyroscope_correction, gyroscope_sum, -1. / (float)calibration_iterations_count );
                VECTOR3_SCALE( accelerometer_average, accelerometer_sum, 1. / (float)calibration_iterations_count );
                float g_squared = 0;
                VECTOR3_LENGTH_SQUARED( g_squared, accelerometer_average );
                VECTOR3_SCALE( accelerometer_correction, accelerometer_average, -1 );
                accelerometer_correction[ 2 ] += sqrtf( g_squared );
            }
        } else {
            vector3f gyroscope_corrected = EMPTY_VECTOR3, accelerometer_corrected = EMPTY_VECTOR3;
            VECTOR3_SUM( gyroscope_corrected, gyroscope, gyroscope_correction );
            VECTOR3_SCALE( gyroscope_corrected, gyroscope_corrected, k_gyroscope );
            VECTOR3_SUM( accelerometer_corrected, accelerometer, accelerometer_correction );

            unsigned long timestamp_us = get_us_from_start( );
            
            if ( !previous_timestamp_us ) {
                previous_timestamp_us = timestamp_us;
                continue;
            }
            
            dt_us = timestamp_us - previous_timestamp_us;
            previous_timestamp_us = timestamp_us;
            dt_s = ( float )dt_us / 1000000.;
            
            // IMU from http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
            
            sampleFreq = 1. / dt_s;
            MadgwickAHRSupdate( gyroscope_corrected[ 0 ], gyroscope_corrected[ 1 ], gyroscope_corrected[ 2 ],
                                accelerometer_corrected[ 0 ], accelerometer_corrected[ 1 ], accelerometer_corrected[ 2 ],
                                //( float )compass[ 0 ], ( float )compass[ 1 ], ( float )compass[ 2 ] );
                                0., 0., 0. );
            quaternionf q = { q0, q1, q2, q3 };
            
            quaternionf_to_Euler_angles( Euler_angles, Euler_angles + 1, Euler_angles + 2, q );

            if (u_throttle) {
                vector3i16 u = EMPTY_VECTOR3;

                vector3f temp = EMPTY_VECTOR3;
                VECTOR3_SUM(temp, temp, Euler_angles);
                VECTOR3_SCALE(temp, temp, dt_s);
                VECTOR3_SUM(integral_error, integral_error, temp);
                RANGE(integral_error[0], integral_error[0], -max_abs_integral_error, max_abs_integral_error);
                RANGE(integral_error[1], integral_error[1], -max_abs_integral_error, max_abs_integral_error);
                RANGE(integral_error[2], integral_error[2], -max_abs_integral_error, max_abs_integral_error);

                u[ 0 ] = ( int )( Euler_angles[ 0 ] * kp_rp +
                                  ( Euler_angles[ 0 ] - Euler_angles_previous[ 0 ] ) * kd_rp / dt_s +
                                  integral_error[ 0 ] * ki_rp );
                u[ 1 ] = ( int )( Euler_angles[ 1 ] * kp_rp +
                                  ( Euler_angles[ 1 ] - Euler_angles_previous[ 1 ] ) * kd_rp / dt_s +
                                  integral_error[ 1 ] * ki_rp );
                u[ 2 ] = ( int )( Euler_angles[ 2 ] * kp_y +
                                  ( Euler_angles[ 2 ] - Euler_angles_previous[ 2 ] ) * kd_y / dt_s +
                                  integral_error[ 2 ] * ki_y );

                u[ 0 ] += u_roll * k_u_roll_pitch;
                u[ 1 ] += u_pitch * k_u_roll_pitch;

                if (camera_control_enabled) {
                    float sin_cos_pi_4 = sin(PI / 4.);

                    UART_read();

                    int16_t x = (int16_t)((float)(_positionX - _positionY)*sin_cos_pi_4);
                    int16_t y = (int16_t)((float)(_positionX + _positionY)*sin_cos_pi_4);

                    u[ 0 ] += y * k_u_camera;
                    u[ 1 ] += x * k_u_camera;
                }

                VECTOR3_COPY( Euler_angles_previous, Euler_angles );

                RANGE(_M1_POWER, _PWM_MIN_SIGNAL + u_throttle * k_u_throttle - u[ 0 ] - u[ 2 ], _PWM_MIN_SIGNAL, _PWM_MAX_SIGNAL);
                RANGE(_M2_POWER, _PWM_MIN_SIGNAL + u_throttle * k_u_throttle + u[ 0 ] - u[ 2 ], _PWM_MIN_SIGNAL, _PWM_MAX_SIGNAL);
                RANGE(_M3_POWER, _PWM_MIN_SIGNAL + u_throttle * k_u_throttle - u[ 1 ] + u[ 2 ], _PWM_MIN_SIGNAL, _PWM_MAX_SIGNAL);
                RANGE(_M4_POWER, _PWM_MIN_SIGNAL + u_throttle * k_u_throttle + u[ 1 ] + u[ 2 ], _PWM_MIN_SIGNAL, _PWM_MAX_SIGNAL);
            } else {
                _M1_POWER = _PWM_MIN_SIGNAL;
                _M2_POWER = _PWM_MIN_SIGNAL;
                _M3_POWER = _PWM_MIN_SIGNAL;
                _M4_POWER = _PWM_MIN_SIGNAL;
            }
        }
    }
}
