#include "UART.h"

#include "stm32f10x_rcc.h"

const unsigned maxBufferSize = 20;

char _inputUARTBuffer[maxBufferSize];
unsigned _inputUARTBufferSize = 0;

int16_t _positionX = 0;
int16_t _positionY = 0;

void UART_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_Cmd(USART1, ENABLE);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART1_IRQn);
}

char hexStringToByte(char* buffer)
{
    char b1 = buffer[0];
    char b2 = buffer[1];
    return ((((b1 >= '0' && b1 <= '9') ? b1 - '0' : b1 - 'A' + 10) << 4) & 0xF0) |
            (((b2 >= '0' && b2 <= '9') ? b2 - '0' : b2 - 'A' + 10) & 0x0F);
}

int16_t hexStringToInt16(char* buffer)
{
    char b1 = hexStringToByte(buffer);
    char b2 = hexStringToByte(buffer + 2);
    return ((b1 << 8) & 0xFF00) | ((b2) & 0x00FF);
}

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        char buffer = USART_ReceiveData(USART1);

        if (buffer == 'n') {
            _positionX = 0;
            _positionY = 0;
            _inputUARTBufferSize = 0;
        } else if (buffer == '\n') {
            if (_inputUARTBufferSize == 8) {
                _positionX = hexStringToInt16(_inputUARTBuffer);
                _positionY = hexStringToInt16(_inputUARTBuffer + 4);
            }
            _inputUARTBufferSize = 0;
        } else if ((buffer >= '0' && buffer <= '9') || (buffer >= 'A' && buffer <= 'F')) {
            _inputUARTBuffer[_inputUARTBufferSize++] = buffer;
        }
    }
}
