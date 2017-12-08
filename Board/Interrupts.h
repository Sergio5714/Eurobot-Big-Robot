#ifndef STM32F4_INTERRUPTS
#define STM32F4_INTERRUPTS

#include "stm32f4xx.h"

extern uint8_t buf;

// Interrupt handler for receiving data from Raspberry Pi (now it is debug module)
void USART1_IRQHandler(void);




#endif
