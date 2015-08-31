#ifndef NRF24L_H
#define NRF24L_H

#include "SPI.h"

extern const uint8_t RX_DR;

void NRF24L_init(char* RADDR, char* TADDR, uint8_t channel, uint8_t _payload);
int NRF24L_data_ready(void);
void NRF24L_get_data(uint8_t* data);
void NRF24L_send(uint8_t* value);
void set_CE(uint8_t value);
void set_CSN(uint8_t value);
void NRF24L_process_spi_interrupt(void);
uint8_t* NRF24L_get_data_interrupt(void);

#endif
