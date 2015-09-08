#include "NRF24L.h"

#include <stdlib.h>

uint8_t PTX;
uint8_t payload;

uint8_t state = 0;
uint8_t currentByte = 0;
uint8_t* data = 0;

const uint8_t W_REGISTER = 0x20;
const uint8_t REGISTER_MASK = 0x1F;
const uint8_t RX_ADDR_P1 = 0x0B;
const uint8_t mirf_ADDR_LEN = 0x05;
const uint8_t RF_CH = 0x05;
const uint8_t RX_PW_P0 = 0x11;
const uint8_t RX_PW_P1 = 0x12;
const uint8_t CONFIG = 0x00;
const uint8_t EN_CRC = 3;
const uint8_t CRCO = 2;
const uint8_t mirf_CONFIG = 0x08;
const uint8_t PWR_UP = 1;
const uint8_t PRIM_RX = 0;
const uint8_t STATUS = 0x07;
const uint8_t TX_DS = 5;
const uint8_t MAX_RT = 4;
const uint8_t FLUSH_RX = 0xE2;
const uint8_t R_RX_PAYLOAD = 0x61;
const uint8_t RX_DR = 6;
const uint8_t R_REGISTER = 0x00;
const uint8_t FIFO_STATUS = 0x17;
const uint8_t RX_EMPTY = 0;
const uint8_t RX_ADDR_P0 = 0x0A;
const uint8_t TX_ADDR = 0x10;
const uint8_t FLUSH_TX = 0xE1;
const uint8_t W_TX_PAYLOAD = 0xA0;

void set_CE(uint8_t value)
{
	if (value == 1)
		GPIO_SetBits(GPIOA, GPIO_Pin_3);
	else
		GPIO_ResetBits(GPIOA, GPIO_Pin_3);
}

void set_CSN(uint8_t value)
{
	if (value == 1)
		GPIO_SetBits(GPIOA, GPIO_Pin_2);
	else
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);
}

void NRF24L_transmit_sync(uint8_t* dataout, uint8_t len)
{
	uint8_t i;
	for(i = 0; i < len; i++) 
		SPI_transfer(dataout[i]);
}

void NRF24L_write_register(uint8_t reg, uint8_t * value, uint8_t len) 
{
	set_CSN(0);
	SPI_transfer(W_REGISTER | (REGISTER_MASK & reg));
	NRF24L_transmit_sync(value, len);
	set_CSN(1);
}

void NRF24L_config_register(uint8_t reg, uint8_t value) 
{
	set_CSN(0);
	SPI_transfer(W_REGISTER | (REGISTER_MASK & reg));
	SPI_transfer(value);
	set_CSN(1);
}

void NRF24L_set_RADDR(uint8_t* adr) 
{
	set_CE(0);
	NRF24L_write_register(RX_ADDR_P1, adr, mirf_ADDR_LEN);
	set_CE(1);
}

void NRF24L_set_TADDR(uint8_t* adr)
{
	NRF24L_write_register(RX_ADDR_P0, adr, mirf_ADDR_LEN);
	NRF24L_write_register(TX_ADDR, adr, mirf_ADDR_LEN);
}

void NRF24L_power_up_rx(void){
	PTX = 0;
	set_CE(0);
	NRF24L_config_register(CONFIG, mirf_CONFIG|((1<<PWR_UP)|(1<<PRIM_RX)));
	set_CE(1);
	NRF24L_config_register(STATUS, (1<<TX_DS)|(1<<MAX_RT));
}

void NRF24L_flush_rx(void)
{
	set_CSN(0);
	SPI_transfer(FLUSH_RX);
	set_CSN(1);
}

void NRF24L_config(uint8_t channel, uint8_t payload) 
{
	NRF24L_config_register(RF_CH, channel);
	NRF24L_config_register(RX_PW_P0, payload);
	NRF24L_config_register(RX_PW_P1, payload);

	NRF24L_power_up_rx();
	NRF24L_flush_rx();
}

void NRF24L_init(char* RADDR, char* TADDR, uint8_t channel, uint8_t _payload)
{
	payload = _payload;
	data = (uint8_t*)malloc(payload * sizeof(uint8_t));
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	set_CE(0);
	set_CSN(1);
	
	SPI_init();
	NRF24L_set_RADDR((uint8_t*)RADDR);
	NRF24L_set_TADDR((uint8_t*)TADDR);
	NRF24L_config(channel, payload);
}

void NRF24L_transfer_sync(uint8_t* dataout, uint8_t* datain, uint8_t len){
	uint8_t i;
	for(i = 0;i < len;i++){
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(SPI1, dataout[i]);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		datain[i] = SPI_I2S_ReceiveData(SPI1);
	}
}

void NRF24L_get_data(uint8_t* data) 
{
	set_CSN(0);
	SPI_transfer(R_RX_PAYLOAD);
	NRF24L_transfer_sync(data,data,payload);
	set_CSN(1);
	NRF24L_config_register(STATUS, (1<<RX_DR));
}

void NRF24L_read_register(uint8_t reg, uint8_t * value, uint8_t len)
{
    set_CSN(0);
	SPI_transfer(R_REGISTER | (REGISTER_MASK & reg));
    NRF24L_transfer_sync(value,value,len);
    set_CSN(1);
}

uint8_t NRF24L_get_status(void)
{
	uint8_t rv = 0;
	NRF24L_read_register(STATUS,&rv,1);
	return rv;
}

int NRF24L_rx_fifo_empty(void){
	uint8_t fifoStatus = 0;
	NRF24L_read_register(FIFO_STATUS, &fifoStatus, 1);
	return (fifoStatus & (1 << RX_EMPTY));
}

int NRF24L_data_ready(void) 
{
	uint8_t status = NRF24L_get_status(); 
	if ( status & (1 << RX_DR) ) return 1;
	return !NRF24L_rx_fifo_empty();
}

void NRF24L_power_up_tx(void)
{
	PTX = 1;
	NRF24L_config_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) );
}

void NRF24L_send(uint8_t* value) 
{
	uint8_t status = NRF24L_get_status();

	while (PTX) 
	{
		status = NRF24L_get_status();
		if((status & ((1 << TX_DS)  | (1 << MAX_RT))))
		{
			PTX = 0;
			break;
		}
	}                  
	
	set_CE(0);
	NRF24L_power_up_tx();
	set_CSN(0);
	SPI_transfer(FLUSH_TX);
	set_CSN(1);
	set_CSN(0);  
	SPI_transfer(W_TX_PAYLOAD);
	NRF24L_transmit_sync(value,payload);
	set_CSN(1);
	set_CE(1);
}

void NRF24L_process_spi_interrupt(void)
{
	if (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==SET) {
		uint8_t byte = SPI1->DR;
		if (state == 0) {
			SPI1->DR = 0;
			++state;
		} else if (state == 1) { 
			set_CSN(1);
			if (byte & (1 << RX_DR)) {
				set_CSN(0);
				SPI1->DR = R_RX_PAYLOAD;
				state = 4;
			} else {
				set_CSN(0);
				SPI1->DR = (R_REGISTER | (REGISTER_MASK & FIFO_STATUS));
				++state;
			}
		} else if (state == 2) {
			SPI1->DR = 0;
			++state;
		} else if (state == 3) {
			set_CSN(1);
			if (!(byte & (1 << RX_EMPTY))) {
				set_CSN(0);
				SPI1->DR = R_RX_PAYLOAD;
				++state;
			} else {
				state = 0;
			}
		} else if (state == 4) {
			SPI1->DR = 0;
			++state;
		} else if (state == 5) {
			data[currentByte++] = byte;
			if (currentByte < payload) {
				SPI1->DR = 0;
			} else {
				set_CSN(1);
				set_CSN(0);
				SPI1->DR = (W_REGISTER | (REGISTER_MASK & STATUS));
				++state;
			}
		} else if (state == 6) {
			SPI1->DR = 1 << RX_DR;
			++state;
		} else if (state == 7) {
			set_CSN(1);
			currentByte = 0;
		}
	}
}

uint8_t* NRF24L_get_data_interrupt(void)
{
	uint8_t result;
	if (!state || state == 7) {
		result = (state == 7) ? 1 : 0;
		state = 0;
		set_CSN(0);
		SPI1->DR = (R_REGISTER | (REGISTER_MASK & STATUS));
	}
	return result ? data : 0;
}
