#ifndef STM8_UART_H
#define STM8_UART_H

#include "stm8s003.h"

void uart_init(unsigned int baudrate);
void uart_write(uint8_t data);
uint8_t uart_read();
uint8_t uart_available();

#endif
