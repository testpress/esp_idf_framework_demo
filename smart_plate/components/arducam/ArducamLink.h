#ifndef ARDUCAM_LINK_H
#define ARDUCAM_LINK_H

#include <stdint.h>
#include "ArducamCamera.h"

// SPI communication functions
void spiBegin(uint8_t cs_pin);
void spiEnd();
uint8_t spiReadByte();
void spiWriteByte(uint8_t data);
void spiWriteBuffer(uint8_t *buffer, uint32_t length);
void spiReadBuffer(uint8_t *buffer, uint32_t length);

// Delay functions
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

// UART functions (if needed)
void uartBegin(uint32_t baud);
void uartWrite(uint8_t data);
uint8_t uartRead();
bool uartAvailable();

#endif // ARDUCAM_LINK_H
