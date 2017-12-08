#include "stm32f4xx.h"
#include "STM32F4_GPIO.h"
#include "STM32F4_UART.h"
#include "STM32F4_TIM.h"
#include "STM32F4_RCC.h"
#include "Board.h"
#include "Dynamixel_control.h"

uint8_t buf;
uint8_t i = 0;


//void USART2_IRQHandler ()
//{
//	if (READ_BIT(USART2->SR, USART_SR_RXNE))
//	{
//		CLEAR_BIT(USART2->SR, USART_SR_RXNE);
//	}
//}

////void UART4_IRQHandler ()
////{
////	if (READ_BIT(UART4->SR, USART_SR_RXNE))
////	{
////		CLEAR_BIT(UART4->SR, USART_SR_RXNE);
////		buf = UART4->DR;
////		
////		usartPutStr(USART1, "answer: ");
////		usartPutC(USART1, buf);
////		
////		gpioPinChangeLevel(DEBUG_LED_PORT, DEBUG_LED_BLUE_PIN);
////	}
////}
//void delay()
//{
//	for (j = 0; j < max; j++);
//	j=0;
//}
//void pwmInitialization()
//{
//	float dutyCycle[4] = {0.2, 0, 0, 0};
//	int channels[4] = {1, 0, 0, 0};
//	float duty1 = 0.5;
//	float duty2 = 0.7;
//	
//	timSettings.TIM_Dir = TIM_DIR_UPCOUNTER;
//	timSettings.TIM_Prescaler = (uint16_t)1600; // 10000 Hz
//	timSettings.TIM_Period = 200; // 50 Hz
//	
//	
//	timInitPwm(TIM4, &timSettings, &dutyCycle[0], &channels[0]);
//	timEnable(TIM4);
//	timPwmChangeDutyCycle(TIM4, duty1, TIM_PWM_CHANNEL_1);
//	timPwmChangeDutyCycle(TIM4, duty2, TIM_PWM_CHANNEL_1);
//}
//void timEncoder()
//{
//	// Pins's settings
//	gpioInitPinAf(GPIOA, GPIO_Pin_0, GPIO_AF_TIM5);
//	gpioInitPinAf(GPIOA, GPIO_Pin_1, GPIO_AF_TIM5);
//	
//	// PA0 and PA1 (channel 1 and channel 2)
//	timInitEncoder(TIM5);
//}
int main()
{
	boardInitAll();

	//timerInitialization();
	//timEncoder();
	//timEnable(TIM5);
	while(1)
	{
		if (buf == 0x01)
		{
			for (i = 0x00; i < 0xFF; i++ )
			{
				if(setServoAngle (i, 0x5A) == true)
				{
					usartPutNumberInASCII(USART1, i);
					usartPutC(USART1, ' ');
					usartPutStr(USART1,"Angle OK");
					usartPutStr(USART1,"\n");
				}
			}
			buf = 0x00;
			usartPutStr(USART1,"Finished");
			usartPutStr(USART1,"\n");
		}
		else if (buf == 0x02)
		{
			buf = 0x00;
		}
		else if (buf == 0x03)
		{
			setServoAngle (0XFE, 0x00);
			buf = 0x00;
		}
		else if (buf == 0x04)
		{
			if(setServoAngle (0X01, 0xBB))
			{
			usartPutStr(USART1,"Eblan");				
			}
			buf = 0x00;
		}
		else if (buf == 0x05)
		{
			for (i = 0x00; i < 0xFF; i++ )
			{
				if(pingServo(i) == true)
				{
					usartPutNumberInASCII(USART1, i);
					usartPutC(USART1, ' ');
					usartPutStr(USART1," OK");
					usartPutStr(USART1,"\n");
				}
			}
			buf = 0x00;
		}
	}
}
