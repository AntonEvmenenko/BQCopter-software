#include "SPI.h"

void SPI_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; 
	// SPI mode 0
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; 
	SPI_Init(SPI1, &SPI_InitStructure); 
	SPI_Cmd(SPI1, ENABLE); 
	
	SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
	
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
}

void SPI_transfer(uint8_t data)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, data);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  SPI_I2S_ReceiveData(SPI1);
}
