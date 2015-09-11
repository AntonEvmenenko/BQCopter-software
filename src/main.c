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


const float k = 90. * PI / ( 67. * 20. * 180. ); // correction for gyro; for anglein radians
const float kp = 2000, kd = 600; // PD regulator components
const int16_t calibration_iterations_count = 500;
const int16_t bad_values_iterations_count = 1000;

// PWM-like signal for ESC; 700 - 2000 us
volatile int M1_POWER = PWM_MIN_SIGNAL;
volatile int M2_POWER = PWM_MIN_SIGNAL;
volatile int M3_POWER = PWM_MIN_SIGNAL;
volatile int M4_POWER = PWM_MIN_SIGNAL;

unsigned long ms_from_start = 0; // time from start in milliseconds

unsigned long previous_timestamp_us = 0, dt_us = 0, previous_dt_us = 0;
float dt_s = 0.;
vector3 gyroscope = { 0., 0., 0. };
vector3 accelerometer = { 0., 0., 0. };
vector3 compass = { 0., 0., 0. };
int16_t calibration_counter = calibration_iterations_count;
uint8_t calibration = 1;
uint16_t bad_values_counter = bad_values_iterations_count;
vector3_32 gyroscope_sum = { 0, 0, 0 };
vector3f gyroscope_correction = { 0., 0., 0. };
unsigned long g_squared_sum = 0;
float g_squared_average = 0;

matrix3f DCM = { { 1., 0., 0., },
                 { 0., 1., 0., },
                 { 0., 0., 1., } };

vector3f Euler_angles = { 0., 0., 0. }, Euler_angles_previous = { 0., 0., 0. };

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
            gyroscope_sum[ 0 ] += gyroscope[ 0 ];
            gyroscope_sum[ 1 ] += gyroscope[ 1 ];
            gyroscope_sum[ 2 ] += gyroscope[ 2 ];

            g_squared_sum += vector3_length_squared( accelerometer );

            if ( !( calibration_counter ? calibration_counter-- : 0 ) ) {
                calibration = 0;

                gyroscope_correction[ 0 ] = -gyroscope_sum[ 0 ] / (float)calibration_iterations_count;
                gyroscope_correction[ 1 ] = -gyroscope_sum[ 1 ] / (float)calibration_iterations_count;
                gyroscope_correction[ 2 ] = -gyroscope_sum[ 2 ] / (float)calibration_iterations_count;

                g_squared_average = g_squared_sum / (float)calibration_iterations_count;
            }
        } else {
            vector3f gyroscope_f = { gyroscope[ 0 ], gyroscope[ 1 ], gyroscope[ 2 ] }, gyroscope_corrected;
            vector3f_sum( gyroscope_corrected, gyroscope_f, gyroscope_correction );
            vector3f_scale( gyroscope_corrected, gyroscope_corrected, k );

            unsigned long timestamp_us = get_us_from_start( );
            dt_us = timestamp_us - previous_timestamp_us;
            previous_timestamp_us = timestamp_us;

            if ( dt_us > 100000 ) {
                dt_us = previous_dt_us;
            }	

            dt_s = ( float )dt_us / 1000000.;

            matrix3f rotation_matrix, delta_angle, delta_DCM;
            vector3f_to_rotation_matrix3f( rotation_matrix, gyroscope_corrected );
            matrix3f_scale( delta_angle, rotation_matrix, dt_s );
            matrix3f_mult( delta_DCM, DCM, delta_angle );
            matrix3f_sum( DCM, DCM, delta_DCM );
            
            // TODO : normalizaton
            
            DCM_to_Euler_angles( Euler_angles, Euler_angles + 1, Euler_angles + 2, DCM );
						
            int u_x = (int)(Euler_angles[ 0 ] * kp + ( Euler_angles[ 0 ] - Euler_angles_previous[ 0 ] ) * kd / dt_s );
            int u_y = (int)(Euler_angles[ 1 ] * kp + ( Euler_angles[ 1 ] - Euler_angles_previous[ 1 ] ) * kd / dt_s );
						
            Euler_angles_previous[ 0 ] = Euler_angles[ 0 ];
            Euler_angles_previous[ 1 ] = Euler_angles[ 1 ];
            Euler_angles_previous[ 2 ] = Euler_angles[ 2 ];
            
            M1_POWER = range( PWM_MIN_SIGNAL - u_x, PWM_MIN_SIGNAL, PWM_MAX_SIGNAL );
            M2_POWER = range( PWM_MIN_SIGNAL + u_x, PWM_MIN_SIGNAL, PWM_MAX_SIGNAL );
            M3_POWER = range( PWM_MIN_SIGNAL - u_y, PWM_MIN_SIGNAL, PWM_MAX_SIGNAL );
            M4_POWER = range( PWM_MIN_SIGNAL + u_y, PWM_MIN_SIGNAL, PWM_MAX_SIGNAL );
        }
    }
}
