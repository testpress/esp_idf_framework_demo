#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>

// UART functions
void uartBegin(uint32_t baud);
void uartWrite(uint8_t data);
uint8_t uartRead();
bool uartAvailable();

#endif // UART_H
