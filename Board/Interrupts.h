#ifndef STM32F4_INTERRUPTS
#define STM32F4_INTERRUPTS

#include "stm32f4xx.h"
#include "Board.h"
#include "Robot.h"

extern uint8_t buf;

// Interrupt handler for receiving data from Raspberry Pi (DEBUG_USART_MODULE = USART1)
// This Interrupt is implemented in Communication.c
void USART1_IRQHandler(void);

// Interrupt handler for communication with Dynamixel servo
// This Interrupt is implemented in Dynamixel_control.c
void UART4_IRQHandler (void);

// Interrupt handler for motor control
void TIM6_DAC_IRQHandler(void);

// Interrupt handler for manipulators control
// This interrupt is implemented in Manipulators.c
void TIM5_IRQHandler(void);

// Interrupt handler for I2C errors
void I2C2_ER_IRQHandler(void);


#endif
