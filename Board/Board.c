#include "Board.h"

TIM_PWM_Typedef motorPwmCh[4] = {{MOTOR_PWM_TIM_MODULE, MOTOR_CH1_NUMBER},
                                {MOTOR_PWM_TIM_MODULE, MOTOR_CH2_NUMBER},
								{MOTOR_PWM_TIM_MODULE, MOTOR_CH3_NUMBER},
								{MOTOR_PWM_TIM_MODULE, MOTOR_CH4_NUMBER}};

GPIO_Pin_TypeDef motorDir[4] = {{MOTOR_CH1_DIR_PORT, MOTOR_CH1_DIR_PIN},
                                {MOTOR_CH2_DIR_PORT, MOTOR_CH2_DIR_PIN},
								{MOTOR_CH3_DIR_PORT, MOTOR_CH3_DIR_PIN},
								{MOTOR_CH4_DIR_PORT, MOTOR_CH4_DIR_PIN}};

GPIO_Pin_TypeDef motorEn[4] = {{MOTOR_CH1_EN_PORT, MOTOR_CH1_EN_PIN},
                               {MOTOR_CH2_EN_PORT, MOTOR_CH2_EN_PIN},
							   {MOTOR_CH3_EN_PORT, MOTOR_CH3_EN_PIN},
							   {MOTOR_CH4_EN_PORT, MOTOR_CH4_EN_PIN}};

uint16_t* encCnt[4] = {ENCODER_1_CNT,
                       ENCODER_2_CNT,
					   ENCODER_3_CNT,
					   ENCODER_4_CNT};


// I2C module for rangefinders
extern I2C_Module_With_State_Typedef I2CModule;
					   
// Initialize all necessary peripheral devices
void boardInitAll()
{
	// Structures for initialization of USART modules
	USART_Init_Typedef usartSettings = usartGetInitStruct();
	TIM_Init_Typedef timSettings = timGetInitStruct();
	
	//--------------------------------------------- Clock System -------------------------------------------------// 
	
	// Initialization of Clock system 
	// System Core clock frequency = 168 MHZ, AHB frequency = 168 MHz, APB1 frequency = 42 MHz, APB2 frequency = 84 MHz)
	SystemClockInitHse168Mhz();
	
	//--------------------------------------------- I2C module for rangefinders ----------------------------------//
	// Initialization
	I2CModule.module = I2C_MODULE;
	I2CModule.status = I2C_ACTIVE_MODE;
	I2CInit(&I2CModule);
	
	// Initialization for SDA and SCL pins
	gpioInitPin(I2C_MODULE_SCL_PIN_PORT, I2C_MODULE_SCL_PIN_NUMBER, GPIO_MODE_AF, GPIO_OUTPUT_MODE_OD, GPIO_PUPD_NOPULL);
	gpioInitPinAf(I2C_MODULE_SCL_PIN_PORT, I2C_MODULE_SCL_PIN_NUMBER, I2C_MODULE_PIN_AF);
	gpioInitPin(I2C_MODULE_SDA_PIN_PORT, I2C_MODULE_SDA_PIN_NUMBER, GPIO_MODE_AF, GPIO_OUTPUT_MODE_OD, GPIO_PUPD_NOPULL);
	gpioInitPinAf(I2C_MODULE_SDA_PIN_PORT, I2C_MODULE_SDA_PIN_NUMBER, I2C_MODULE_PIN_AF);
	
	//--------------------------------------------- Usart module for dynamixel servo -----------------------------//
	
	// Usart initialization
	usartSettings.USART_Baudrate = DYNAMIXEL_USART_BAUDRATE;
	usartInit(DYNAMIXEL_USART_MODULE, &usartSettings);
	
	// Usart initialization for TX and Rx pin
	gpioInitPin(DYNAMIXEL_USART_TX_PIN_PORT, DYNAMIXEL_USART_TX_PIN_NUMBER, GPIO_MODE_AF, GPIO_OUTPUT_MODE_OD, GPIO_PUPD_NOPULL);
	gpioInitPinAf(DYNAMIXEL_USART_TX_PIN_PORT, DYNAMIXEL_USART_TX_PIN_NUMBER, DYNAMIXEL_USART_PIN_AF);
	gpioInitPin(DYNAMIXEL_USART_RX_PIN_PORT, DYNAMIXEL_USART_RX_PIN_NUMBER, GPIO_MODE_AF, GPIO_OUTPUT_MODE_OD, GPIO_PUPD_NOPULL);
	gpioInitPinAf(DYNAMIXEL_USART_RX_PIN_PORT, DYNAMIXEL_USART_RX_PIN_NUMBER, DYNAMIXEL_USART_PIN_AF);
	
	//--------------------------------------------- Usart module for communication ------------------------------//
	// Init usart module
	usartSettings.USART_Baudrate = COM_USART_BAUDRATE;
	usartInit(COM_USART_MODULE, &usartSettings);
	
	// Settings for pins
	// Base Init
	gpioInitPin(COM_USART_TX_PIN_PORT, COM_USART_TX_PIN_NUMBER, GPIO_MODE_AF, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_UP);
	gpioInitPin(COM_USART_RX_PIN_PORT, COM_USART_RX_PIN_NUMBER, GPIO_MODE_AF, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_UP);
	
	// Set alternatife function
	gpioInitPinAf(COM_USART_TX_PIN_PORT, COM_USART_TX_PIN_NUMBER, COM_USART_PIN_AF);
	gpioInitPinAf(COM_USART_RX_PIN_PORT, COM_USART_RX_PIN_NUMBER, COM_USART_PIN_AF);
	
	//--------------------------------------------- Initialization of timers for encoders -----------------------//
	// Settings for pins
	// Base Init
	gpioInitPin(ENCODER_1_CHA_PORT, ENCODER_1_CHA_PIN, GPIO_MODE_AF, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	gpioInitPin(ENCODER_1_CHB_PORT, ENCODER_1_CHB_PIN, GPIO_MODE_AF, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	gpioInitPin(ENCODER_2_CHA_PORT, ENCODER_2_CHA_PIN, GPIO_MODE_AF, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	gpioInitPin(ENCODER_2_CHB_PORT, ENCODER_2_CHB_PIN, GPIO_MODE_AF, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	gpioInitPin(ENCODER_3_CHA_PORT, ENCODER_3_CHA_PIN, GPIO_MODE_AF, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	gpioInitPin(ENCODER_3_CHB_PORT, ENCODER_3_CHB_PIN, GPIO_MODE_AF, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	gpioInitPin(ENCODER_4_CHA_PORT, ENCODER_4_CHA_PIN, GPIO_MODE_AF, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	gpioInitPin(ENCODER_4_CHB_PORT, ENCODER_4_CHB_PIN, GPIO_MODE_AF, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	
	// Set alternatice function
	gpioInitPinAf(ENCODER_1_CHA_PORT, ENCODER_1_CHA_PIN, ENCODER_1_PIN_AF);
	gpioInitPinAf(ENCODER_1_CHB_PORT, ENCODER_1_CHB_PIN, ENCODER_1_PIN_AF);
	gpioInitPinAf(ENCODER_2_CHA_PORT, ENCODER_2_CHA_PIN, ENCODER_2_PIN_AF);
	gpioInitPinAf(ENCODER_2_CHB_PORT, ENCODER_2_CHB_PIN, ENCODER_2_PIN_AF);
	gpioInitPinAf(ENCODER_3_CHA_PORT, ENCODER_3_CHA_PIN, ENCODER_3_PIN_AF);
	gpioInitPinAf(ENCODER_3_CHB_PORT, ENCODER_3_CHB_PIN, ENCODER_3_PIN_AF);
	gpioInitPinAf(ENCODER_4_CHA_PORT, ENCODER_4_CHA_PIN, ENCODER_4_PIN_AF);
	gpioInitPinAf(ENCODER_4_CHB_PORT, ENCODER_4_CHB_PIN, ENCODER_4_PIN_AF);
	
	// Init timer modules
	timInitEncoder(ENCODER_1_TIM_MODULE);
	timInitEncoder(ENCODER_2_TIM_MODULE);
	timInitEncoder(ENCODER_3_TIM_MODULE);
	timInitEncoder(ENCODER_4_TIM_MODULE);
	
	//--------------------------------------------- Initialization of PWM channels for motor control ------------//
	
	// Settings for pins
	// Base initialization of PWM pins
	gpioInitPin(MOTOR_CH_PWM_PORT, MOTOR_CH1_PWM_PIN,GPIO_MODE_AF, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	gpioInitPin(MOTOR_CH_PWM_PORT, MOTOR_CH2_PWM_PIN,GPIO_MODE_AF, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	gpioInitPin(MOTOR_CH_PWM_PORT, MOTOR_CH3_PWM_PIN,GPIO_MODE_AF, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	gpioInitPin(MOTOR_CH_PWM_PORT, MOTOR_CH4_PWM_PIN,GPIO_MODE_AF, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	
	// Set alternative function for PWM pins
	gpioInitPinAf(MOTOR_CH_PWM_PORT, MOTOR_CH1_PWM_PIN, MOTOR_PWM_PIN_AF);
	gpioInitPinAf(MOTOR_CH_PWM_PORT, MOTOR_CH2_PWM_PIN, MOTOR_PWM_PIN_AF);
	gpioInitPinAf(MOTOR_CH_PWM_PORT, MOTOR_CH3_PWM_PIN, MOTOR_PWM_PIN_AF);
	gpioInitPinAf(MOTOR_CH_PWM_PORT, MOTOR_CH4_PWM_PIN, MOTOR_PWM_PIN_AF);
	
	// Base initialization of DIR pins
	gpioInitPin(MOTOR_CH1_DIR_PORT, MOTOR_CH1_DIR_PIN, GPIO_MODE_OUT, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	gpioInitPin(MOTOR_CH2_DIR_PORT, MOTOR_CH2_DIR_PIN, GPIO_MODE_OUT, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	gpioInitPin(MOTOR_CH3_DIR_PORT, MOTOR_CH3_DIR_PIN, GPIO_MODE_OUT, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	gpioInitPin(MOTOR_CH4_DIR_PORT, MOTOR_CH4_DIR_PIN, GPIO_MODE_OUT, GPIO_OUTPUT_MODE_PP, GPIO_PUPD_NOPULL);
	
	// All directions CW
	gpioPinSetLevel(MOTOR_CH1_DIR_PORT, MOTOR_CH1_DIR_PIN, GPIO_LEVEL_HIGH);
	gpioPinSetLevel(MOTOR_CH2_DIR_PORT, MOTOR_CH2_DIR_PIN, GPIO_LEVEL_HIGH);
	gpioPinSetLevel(MOTOR_CH3_DIR_PORT, MOTOR_CH3_DIR_PIN, GPIO_LEVEL_HIGH);
	gpioPinSetLevel(MOTOR_CH4_DIR_PORT, MOTOR_CH4_DIR_PIN, GPIO_LEVEL_HIGH);
	
	// Initialize timer module for PWM
	timSettings.TIM_Period = MOTOR_PWM_TIM_ARR;
	timSettings.TIM_Prescaler = MOTOR_PWM_TIM_PSC;
	timInitBase(MOTOR_PWM_TIM_MODULE, &timSettings);
	timInitPwm(MOTOR_PWM_TIM_MODULE, &timSettings, (float[4]){0.1, 0.1, 0.1, 0.1}, (uint8_t[4]){0x01, 0x01, 0x01, 0x01});
	
	//--------------------------------------------- Motor control timer initialization ---------------------------//
	timSettings.TIM_Period = MOTOR_CONTROL_TIM_ARR;
	timSettings.TIM_Prescaler = MOTOR_CONTROL_TIM_PSC;
	timInitBase(MOTOR_CONTROL_TIM_MODULE, &timSettings);
	
	// Update interrupt enable
	timInterruptEnable(MOTOR_CONTROL_TIM_MODULE, TIM_DIER_UIE);
	
	//--------------------------------------------- Manipulators control timer initialization --------------------//
	timSettings.TIM_Period = SERVO_CHECKER_TIM_ARR;
	timSettings.TIM_Prescaler = SERVO_CHECKER_TIM_PSC;
	timInitBase(SERVO_CHECKER_TIM_MODULE, &timSettings);
	
	// Set one pulse mode
	timSetOnePulseMode(SERVO_CHECKER_TIM_MODULE);
	
	// Update interrupt enable
	timInterruptEnable(SERVO_CHECKER_TIM_MODULE, TIM_DIER_UIE);
	
	//--------------------------------------------- Local time timer initialization ------------------------------//
	timSettings.TIM_Period = LOCAL_TIME_TIM_ARR;
	timSettings.TIM_Prescaler = LOCAL_TIME_TIM_PSC;
	timInitBase(LOCAL_TIME_TIM_MODULE, &timSettings);
	
	
	// Update interrupt enable
	timInterruptEnable(LOCAL_TIME_TIM_MODULE, TIM_DIER_UIE);
	
	//--------------------------------------------- External interrupts ------------------------------------------//
	gpioInitPin(EXTI_STARTUP_PORT, EXTI_STARTUP_PIN, GPIO_MODE_IN, GPIO_OUTPUT_MODE_OD, GPIO_PUPD_NOPULL);
	extiInit(EXTI_STARTUP_SOURCE_PORT, EXTI_STARTUP_PIN, EXTI_INTERRUPT_MODE_FALLING);
	
	//--------------------------------------------- Enable microchip for dynamixel signal pin --------------------//

	gpioInitPin(DYNAMIXEL_SIGNAL_EN_PORT, DYNAMIXEL_SIGNAL_EN_PIN, GPIO_MODE_OUT, GPIO_OUTPUT_MODE_OD, GPIO_PUPD_UP);
	gpioPinSetLevel(DYNAMIXEL_SIGNAL_EN_PORT, DYNAMIXEL_SIGNAL_EN_PIN, GPIO_LEVEL_HIGH);
	
	//--------------------------------------------- Enable modules -----------------------------------------------//
	
	// Enable USART modules
	usartEnable(COM_USART_MODULE);
	usartEnable(DYNAMIXEL_USART_MODULE);
	
	// Enable I2C module
	I2CEnable(I2C_MODULE);
	
	// Enable Encoders
	timEnable(ENCODER_1_TIM_MODULE);
	timEnable(ENCODER_2_TIM_MODULE);
	timEnable(ENCODER_3_TIM_MODULE);
	timEnable(ENCODER_4_TIM_MODULE);
	
	// Enable timers
	// Enable PWM for Motors
	timEnable(MOTOR_PWM_TIM_MODULE);
	// Enable Motor control timer
	timEnable(MOTOR_CONTROL_TIM_MODULE);
	// Enable Manipulators control timer
	timEnable(SERVO_CHECKER_TIM_MODULE);
	// Enable Local time timer
	timEnable(LOCAL_TIME_TIM_MODULE);
	
	//--------------------------------------------- Enable interrupts -------------------------------------------//
	// Enable
	__NVIC_EnableIRQ(COM_USART_IRQN);
	__NVIC_EnableIRQ(LOCAL_TIME_IRQN);
	__NVIC_EnableIRQ(DYNAMIXEL_USART_IRQN);
	
	__NVIC_EnableIRQ(I2C_MODULE_ERROR_IRQN);
	
	__NVIC_EnableIRQ(MOTOR_CONTROL_IRQN);
	__NVIC_EnableIRQ(SERVO_CHECKER_IRQN);
	
	__NVIC_EnableIRQ(EXTI_STARTUP_IRQ);
	
	// Priority
	__NVIC_SetPriority(COM_USART_IRQN, 0X00);
	__NVIC_SetPriority(LOCAL_TIME_IRQN, 0X01);
	
	__NVIC_SetPriority(DYNAMIXEL_USART_IRQN, 0X02);
	__NVIC_SetPriority(EXTI_STARTUP_IRQ, 0X02);
	
	__NVIC_SetPriority(I2C_MODULE_ERROR_IRQN, 0X03);
	
	__NVIC_SetPriority(MOTOR_CONTROL_IRQN, 0X05);
	__NVIC_SetPriority(SERVO_CHECKER_IRQN, 0X04);
	
	// Global enable
	__enable_irq();
	return;
}
