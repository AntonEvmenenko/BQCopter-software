#ifndef SPI_H
#define SPI_H

#include "stm32f10x_spi.h"

void SPI_init(void);
void SPI_transfer(uint8_t data);

#endif
