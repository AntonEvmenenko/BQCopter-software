#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

#include "L3G4200D.h"
#include "ADXL345.h"
#include "NRF24L.h"
#include "ESC.h"

const int PWM_MIN_SIGNAL = 700; // us
const int PWM_MAX_SIGNAL = 2000; // us

// PWM-like signal for ESC; 700 - 2000 us
volatile int M1_POWER = PWM_MIN_SIGNAL;
volatile int M2_POWER = PWM_MIN_SIGNAL;
volatile int M3_POWER = PWM_MIN_SIGNAL;
volatile int M4_POWER = PWM_MIN_SIGNAL;

unsigned long ms_from_start = 0; // time from start in milliseconds

float angle_x = 0, previous_angle_x = 0;
unsigned long previous_timestamp_us = 0, dt = 0, previous_dt = 0;
const float PI = 3.14159265359;
float k = 90. * PI / ( 67. * 20. * 1000000. * 180. ); // correction for gyro; for anglein radians

float kp = 1200, kd = 600; // PD regulator components

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

float abs( float x ) {
	return x > 0 ? x : -x;
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
	
	int16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;
	int16_t calibration_iterations_count = 500;
	int16_t calibration_counter = calibration_iterations_count;
	uint8_t calibration = 1;
	uint16_t bad_values_counter = 1000;
	int32_t gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
	float gyro_x_correction = 0, gyro_y_correction = 0, gyro_z_correction = 0;
	
	int a = 0;
	
	while(1)
	{		
		getGyroValues( &gyro_x, &gyro_y, &gyro_z );
		
		if ( bad_values_counter > 0 ) {
			--bad_values_counter;
			continue;
		}
				
		if ( calibration ) {
			gyro_x_sum += gyro_x;
			gyro_y_sum += gyro_y;
			gyro_z_sum += gyro_z;
			
			if ( calibration_counter == 0 ) {
				calibration = 0;
				
				gyro_x_correction = gyro_x_sum / (float)calibration_iterations_count;
				gyro_y_correction = gyro_y_sum / (float)calibration_iterations_count;
				gyro_z_correction = gyro_z_sum / (float)calibration_iterations_count;
			}
			
			--calibration_counter;
		} else {
			unsigned long timestamp_us = get_us_from_start( );
			dt = timestamp_us - previous_timestamp_us;
			previous_timestamp_us = timestamp_us;
			
			if ( dt > 100000 ) {
				dt = previous_dt;
			}	
			
			previous_angle_x = angle_x;
			angle_x += ( k * dt * ( gyro_x - gyro_x_correction ) );
			
			int u = (int)(angle_x * kp + ( angle_x - previous_angle_x ) * kd / dt );
			
			M1_POWER = range( PWM_MIN_SIGNAL - u, PWM_MIN_SIGNAL, PWM_MAX_SIGNAL );
			M2_POWER = range( PWM_MIN_SIGNAL + u, PWM_MIN_SIGNAL, PWM_MAX_SIGNAL );
		}
	}
}
