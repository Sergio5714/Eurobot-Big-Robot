#include "Interrupts.h"
#include "STM32F4_GPIO.h"
#include "STM32F4_UART.h"
#include "Board.h"

// Interrupt handler for receiving data from Raspberry Pi (DEBUG_USART_MODULE = USART1)
void USART1_IRQHandler ()
{
	if (READ_BIT(DEBUG_USART_MODULE->SR, USART_SR_RXNE))
	{
		CLEAR_BIT(DEBUG_USART_MODULE->SR, USART_SR_RXNE);

		buf = DEBUG_USART_MODULE->DR;
		
		usartPutStr(DEBUG_USART_MODULE, "byte: ");
		usartPutC(DEBUG_USART_MODULE, buf);
		usartPutStr(DEBUG_USART_MODULE, " ");
		usartPutStr(DEBUG_USART_MODULE,"\n");
	
		gpioPinChangeLevel(DEBUG_LED_PORT, DEBUG_LED_GREEN_PIN);
	}
}