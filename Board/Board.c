#include "Board.h"
#include "STM32F4_GPIO.h"
#include "STM32F4_TIM.h"
#include "STM32F4_UART.h"
#include "STM32F4_RCC.h"

// Initialize all necessary peripheral devices
void boardInitAll()
{
	// Structures for initialization
	USART_Init_Typedef dynamixelUsartSettings = usartGetInitStruct();
	USART_Init_Typedef debugUsartSettings = usartGetInitStruct();
	//--------------------------------------------- Clock System -------------------------------------------------// 
	// Initialization of Clock system 
	// System Core clock frequency = 168 MHZ, AHB frequency = 168 MHz, APB1 frequency = 42 MHz, APB2 frequency = 84 MHz)
	SystemClockInitHse168Mhz();
	
	//--------------------------------------------- Usart module for dynamixel servo -----------------------------// 
	// Usart initialization
	dynamixelUsartSettings.USART_Baudrate = DYNAMIXEL_USART_BAUDRATE;
	dynamixelUsartSettings.USART_Half_Duplex_Mode = USART_HALF_DUPLEX_MODE_DISABLE;
	usartInit(DYNAMIXEL_USART_MODULE, &dynamixelUsartSettings);
	// Usart initialization for TX and Rx pin
	gpioInitPin(DYNAMIXEL_USART_TX_PIN_PORT, DYNAMIXEL_USART_TX_PIN_NUMBER, GPIO_MODE_AF, GPIO_OUTPUT_MODE_OD, GPIO_PUPD_NOPULL);
	gpioInitPinAf(DYNAMIXEL_USART_TX_PIN_PORT, DYNAMIXEL_USART_TX_PIN_NUMBER, DYNAMIXEL_USART_PIN_AF);
	gpioInitPin(DYNAMIXEL_USART_RX_PIN_PORT, DYNAMIXEL_USART_RX_PIN_NUMBER, GPIO_MODE_AF, GPIO_OUTPUT_MODE_OD, GPIO_PUPD_NOPULL);
	gpioInitPinAf(DYNAMIXEL_USART_RX_PIN_PORT, DYNAMIXEL_USART_RX_PIN_NUMBER, DYNAMIXEL_USART_PIN_AF);
	
	//--------------------------------------------- Usart module for debug --------------------------------------// 
	debugUsartSettings.USART_Baudrate = DEBUG_USART_BAUDRATE;
	usartInit(DEBUG_USART_MODULE, &debugUsartSettings);
	// Settings for pins
	gpioInitPin(DEBUG_USART_TX_PIN_PORT, DEBUG_USART_TX_PIN_NUMBER, GPIO_MODE_AF, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_UP);
	gpioInitPin(DEBUG_USART_RX_PIN_PORT, DEBUG_USART_RX_PIN_NUMBER, GPIO_MODE_AF, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_UP);
	gpioInitPinAf(DEBUG_USART_TX_PIN_PORT, DEBUG_USART_TX_PIN_NUMBER, DEBUG_USART_PIN_AF);
	gpioInitPinAf(DEBUG_USART_RX_PIN_PORT, DEBUG_USART_RX_PIN_NUMBER, DEBUG_USART_PIN_AF);
	
	//--------------------------------------------- Onboard LEDs for debug --------------------------------------//
	gpioInitPin(DEBUG_LED_PORT, DEBUG_LED_GREEN_PIN, GPIO_MODE_OUT, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	gpioInitPin(DEBUG_LED_PORT, DEBUG_LED_ORANGE_PIN, GPIO_MODE_OUT, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	gpioInitPin(DEBUG_LED_PORT, DEBUG_LED_RED_PIN, GPIO_MODE_OUT, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	gpioInitPin(DEBUG_LED_PORT, DEBUG_LED_BLUE_PIN, GPIO_MODE_OUT, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	
	//--------------------------------------------- Enable octal buffer for dynamixel signal pin ----------------//
	gpioInitPin(GPIOB, GPIO_Pin_12, GPIO_MODE_OUT, GPIO_OUTPUT_MODE_OD, GPIO_PUPD_UP);
	gpioPinSetLevel(GPIOB, GPIO_Pin_12, GPIO_LEVEL_HIGH);
	
	//--------------------------------------------- Enable modules --------------------------------------//
	// Enable USART modules
	usartEnable(DEBUG_USART_MODULE);
	usartEnable(DYNAMIXEL_USART_MODULE);
	
	//--------------------------------------------- Enable interrupts --------------------------------------//
	__enable_irq();
	__NVIC_EnableIRQ(DEBUG_USART_IRQN);
	__NVIC_EnableIRQ(DYNAMIXEL_USART_IRQN);
	
	
}
