#ifndef __STM32F429_H
#define __STM32F429_H
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"

/* This library contains routines for interfacing with the STM32F429i-DISCO board. */

/* Configures the RS232 serial port using the following settings:
 *   115200 Baud
 *   8 bits + 1 stop bit
 *   No parity bit
 *   No hardware flow control
 */
void init_rs232(USART_TypeDef* usart, uint32_t BaudRate);

#endif /* __STM32F429_H */
