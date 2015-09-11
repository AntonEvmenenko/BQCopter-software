#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

#include "L3G4200D.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "NRF24L.h"
#include "ESC.h"

#include "extended_math.h"

const int PWM_MIN_SIGNAL = 700; // us
const int PWM_MAX_SIGNAL = 2000; // us

const float PI = 3.14159265359;
const float k = 90. * PI / ( 67. * 20. * 1000000. * 180. ); // correction for gyro; for anglein radians
const float kp = 2000, kd = 600; // PD regulator components
const int16_t calibration_iterations_count = 500;
const int16_t bad_values_iterations_count = 1000;

// PWM-like signal for ESC; 700 - 2000 us
volatile int M1_POWER = PWM_MIN_SIGNAL;
volatile int M2_POWER = PWM_MIN_SIGNAL;
volatile int M3_POWER = PWM_MIN_SIGNAL;
volatile int M4_POWER = PWM_MIN_SIGNAL;

unsigned long ms_from_start = 0; // time from start in milliseconds

float angle_x = 0, previous_angle_x = 0, angle_y = 0, previous_angle_y = 0;
unsigned long previous_timestamp_us = 0, dt = 0, previous_dt = 0;
vector3 gyroscope = { 0., 0., 0. };
vector3 accelerometer = { 0., 0., 0. };
vector3 compass = { 0., 0., 0. };
int16_t calibration_counter = calibration_iterations_count;
uint8_t calibration = 1;
uint16_t bad_values_counter = bad_values_iterations_count;
int32_t gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
vector3 gyroscope_correction = { 0., 0., 0. };
unsigned long g_squared_sum = 0;
float g_squared_average = 0;

matrix3f DCM = { { 1., 0., 0., },
                 { 0., 1., 0., },
                 { 0., 0., 1., } };

vector3f Euler_angles = { 0., 0., 0. };

void SysTick_init( void )
{
    SysTick_Config( SystemCoreClock / 1000 ); // 1ms per interrupt

    // set systick interrupt priority
    //NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); // 4 bits for preemp priority 0 bit for sub priority
    NVIC_SetPriority( SysTick_IRQn, 0 ); // i want to make sure systick has highest priority amount all other interrupts

    ms_from_start = 0;
}

unsigned long get_ms_from_start( void ) {
    return ms_from_start;
}

unsigned long get_us_from_start( void ) {
    return ms_from_start * 1000 + 1000 - SysTick->VAL / 72;
}

void SysTick_Handler( void ) {
    ++ms_from_start;
}

void TIM3_IRQHandler( )
{
    TIM_ClearITPendingBit( TIM3, TIM_IT_Update );
    TIM3->CCR3 = M1_POWER;
    TIM3->CCR4 = M2_POWER;
}

void TIM2_IRQHandler( )
{
    TIM_ClearITPendingBit( TIM2, TIM_IT_Update );
    TIM2->CCR3 = M3_POWER;
    TIM2->CCR4 = M4_POWER;
}

int range( int x, int min, int max ) {
    return x > max ? max : ( x < min ? min : x );
}

int main(void)
{
    __enable_irq();

    ESC_init();
    SysTick_init( );
    I2C_init( I2C1, 200000 );
    setupL3G4200D( 2000 );
    setupADXL345( );
    setupHMC5883L( );
	
    while( 1 )
    {		 
        getGyroValues( gyroscope, gyroscope + 1, gyroscope + 2 );
        getAccelValues( accelerometer, accelerometer + 1, accelerometer + 2 );
        getRawCompassValues( compass, compass + 1, compass + 2 );

        if ( ( bad_values_counter ? bad_values_counter-- : 0 ) > 0 ) {
            continue;
        }

        if ( calibration ) {
            gyro_x_sum += gyroscope[ 0 ];
            gyro_y_sum += gyroscope[ 1 ];
            gyro_z_sum += gyroscope[ 2 ];

            g_squared_sum += vector3_length_squared( accelerometer );

            if ( !( calibration_counter ? calibration_counter-- : 0 ) ) {
                calibration = 0;

                gyroscope_correction[ 0 ] = gyro_x_sum / (float)calibration_iterations_count;
                gyroscope_correction[ 1 ] = gyro_y_sum / (float)calibration_iterations_count;
                gyroscope_correction[ 2 ] = gyro_z_sum / (float)calibration_iterations_count;

                g_squared_average = g_squared_sum / (float)calibration_iterations_count;
            }
        } else {
            vector3f gyroscope_corrected = { gyroscope[ 0 ] - gyroscope_correction[ 0 ],
                                             gyroscope[ 1 ] - gyroscope_correction[ 1 ],
                                             gyroscope[ 2 ] - gyroscope_correction[ 2 ] };
            
            unsigned long timestamp_us = get_us_from_start( );
            dt = timestamp_us - previous_timestamp_us;
            previous_timestamp_us = timestamp_us;

            if ( dt > 100000 ) {
                dt = previous_dt;
            }	

            /*
            angle_x += ( k * dt * gyroscope_corrected[ 0 ] );
            angle_y += ( k * dt * gyroscope_corrected[ 1 ] );

            int u_x = (int)(angle_x * kp + ( angle_x - previous_angle_x ) * kd * 1000000. / ( float )dt );
            int u_y = (int)(angle_y * kp + ( angle_y - previous_angle_y ) * kd * 1000000. / ( float )dt );

            previous_angle_x = angle_x;
            previous_angle_y = angle_y;

            M1_POWER = range( PWM_MIN_SIGNAL - u_x, PWM_MIN_SIGNAL, PWM_MAX_SIGNAL );
            M2_POWER = range( PWM_MIN_SIGNAL + u_x, PWM_MIN_SIGNAL, PWM_MAX_SIGNAL );
            M3_POWER = range( PWM_MIN_SIGNAL - u_y, PWM_MIN_SIGNAL, PWM_MAX_SIGNAL );
            M4_POWER = range( PWM_MIN_SIGNAL + u_y, PWM_MIN_SIGNAL, PWM_MAX_SIGNAL );
            */
            
            matrix3f rotation_matrix, delta_angle, delta_DCM;
            vector3f_to_rotation_matrix3f( rotation_matrix, gyroscope_corrected );
            matrix3f_scalar_multiplication( delta_angle, rotation_matrix, 1000000. / ( float )dt );
            matrix3f_matrix3f_multiplication( delta_DCM, DCM, delta_angle );
            matrix3f_matrix3f_sum( DCM, DCM, delta_DCM );
            
            // TODO : normalizaton
            
            DCM_to_Euler_angles( Euler_angles, Euler_angles + 1, Euler_angles + 2, DCM );
        }
    }
}
