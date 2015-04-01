#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#include "L3G4200D.h"
#include "ADXL345.h"
#include "NRF24L.h"

void Soft_Delay(volatile uint32_t number)
{
	while(number--);
} 

void led_init() 
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef  gpio; 
	gpio.GPIO_Mode = GPIO_Mode_Out_PP; 
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	gpio.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &gpio);
}

int main(void)
{
	
	SystemInit();
	led_init();
	//I2C_init(I2C1, 400000);
	//setupL3G4200D(2000);
	//setupADXL345();
	char raddr[5] = "serv2";
	char taddr[5] = "serv1";
	NRF24L_init(raddr, taddr, 90, 1);
	
	
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;
	uint8_t a = 0;
	
	while(1)
	{
		//GPIO_SetBits(GPIOC, GPIO_Pin_13);
		//Soft_Delay(0x00000FF);
		/*GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		Soft_Delay(0x000FFFFF);
		*/
		//getGyroValues(&x, &y, &z);
		//getAccelValues(&x, &y, &z);
		
		if (NRF24L_data_ready()) 
		{
			NRF24L_get_data(&a);
		}
	}
}
