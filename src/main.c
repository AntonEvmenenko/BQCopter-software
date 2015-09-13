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
#include "MadgwickAHRS.h"

const int PWM_MIN_SIGNAL = 700; // us
const int PWM_MAX_SIGNAL = 2000; // us

const float k = 90. * PI / ( 67. * 20. * 180. ); // correction for gyro; for angle in radians
const float kp_rp = 2500, kd_rp = 600; // PD regulator components for roll and pitch
const float kp_y = 1000, kd_y = 400; // PD regulator components for yaw
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

int16_t calibration_counter = calibration_iterations_count;
uint8_t calibration = 1;
uint16_t bad_values_counter = bad_values_iterations_count;

vector3i16 gyroscope = EMPTY_VECTOR3, accelerometer = EMPTY_VECTOR3, compass = EMPTY_VECTOR3;
vector3i32 gyroscope_sum = EMPTY_VECTOR3;
vector3f gyroscope_correction = EMPTY_VECTOR3, Euler_angles = EMPTY_VECTOR3, Euler_angles_previous = EMPTY_VECTOR3;

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
            VECTOR3_SUM( gyroscope_sum, gyroscope_sum, gyroscope );

            if ( !( calibration_counter ? calibration_counter-- : 0 ) ) {
                calibration = 0;
                VECTOR3_SCALE( gyroscope_correction, gyroscope_sum, -1. / (float)calibration_iterations_count );
            }
        } else {
            vector3f gyroscope_f = EMPTY_VECTOR3, gyroscope_corrected = EMPTY_VECTOR3;
            VECTOR3_COPY( gyroscope_f, gyroscope );
            VECTOR3_SUM( gyroscope_corrected, gyroscope_f, gyroscope_correction );
            VECTOR3_SCALE( gyroscope_corrected, gyroscope_corrected, k );

            unsigned long timestamp_us = get_us_from_start( );
            dt_us = timestamp_us - previous_timestamp_us;
            previous_timestamp_us = timestamp_us;

            if ( dt_us > 100000 ) {
                dt_us = previous_dt_us;
            }	

            dt_s = ( float )dt_us / 1000000.;
            
            sampleFreq = 1. / dt_s;
            MadgwickAHRSupdate( gyroscope_corrected[ 0 ], gyroscope_corrected[ 1 ], gyroscope_corrected[ 2 ],
                                ( float )accelerometer[ 0 ], ( float )accelerometer[ 1 ], ( float )accelerometer[ 2 ],
                                ( float )compass[ 0 ], ( float )compass[ 1 ], ( float )compass[ 2 ] );
            quaternionf q = { q0, q1, q2, q3 };
            
            quaternionf_to_Euler_angles( Euler_angles, Euler_angles + 1, Euler_angles + 2, q );

            vector3i16 u = EMPTY_VECTOR3;
            u[ 0 ] = (int)(Euler_angles[ 0 ] * kp_rp + ( Euler_angles[ 0 ] - Euler_angles_previous[ 0 ] ) * kd_rp / dt_s );
            u[ 1 ] = (int)(Euler_angles[ 1 ] * kp_rp + ( Euler_angles[ 1 ] - Euler_angles_previous[ 1 ] ) * kd_rp / dt_s );
            u[ 2 ] = 0;//(int)(Euler_angles[ 2 ] * kp_y + ( Euler_angles[ 2 ] - Euler_angles_previous[ 2 ] ) * kd_y / dt_s );

            VECTOR3_COPY( Euler_angles_previous, Euler_angles );

            M1_POWER = range( PWM_MIN_SIGNAL - u[ 0 ] - u[ 2 ], PWM_MIN_SIGNAL, PWM_MAX_SIGNAL );
            M2_POWER = range( PWM_MIN_SIGNAL + u[ 0 ] - u[ 2 ], PWM_MIN_SIGNAL, PWM_MAX_SIGNAL );
            M3_POWER = range( PWM_MIN_SIGNAL - u[ 1 ] + u[ 2 ], PWM_MIN_SIGNAL, PWM_MAX_SIGNAL );
            M4_POWER = range( PWM_MIN_SIGNAL + u[ 1 ] + u[ 2 ], PWM_MIN_SIGNAL, PWM_MAX_SIGNAL );
        }
    }
}
