#ifndef __STM32F4_H
#define __STM32F4_H
#include "stm32f4xx.h"

/* This library contains routines for interfacing with the STM32F4xx. */

/* Configures the RS232 serial port using the following settings:
 *   115200 Baud
 *   8 bits + 1 stop bit
 *   No parity bit
 *   No hardware flow control
 */
void init_rs232(USART_TypeDef* usart, uint32_t BaudRate);

#endif /* __STM32F4_H */
