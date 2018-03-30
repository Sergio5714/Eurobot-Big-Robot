#include "stm32f4xx.h"
#include "Board.h"
#include "Communication.h"
#include "Manipulators.h"
#include "Collision_avoidance.h"


extern RobotStatus Robot;
// I2C module for rangefinders
extern I2C_Module_With_State_Typedef I2CModule;

uint32_t numberOfReceivedPackages;
uint32_t numberOfChecksumErrors;
uint32_t numberOfSmallLengthErrors;

uint8_t range;

uint8_t state;

int main()
{		

	state = 0;
   	boardInitAll();
	initManipulators();
	Robot.forwardKinCalcStatusFlag = 0x01;
	
	//initAllRangefinders();
	//rangeFinderInitContiniousInterruptLevelLowMode(RANGEFINDER_DEFAULT_ADDR, 100);
	//rangeFinderStartContiniousMeasurements(RANGEFINDER_DEFAULT_ADDR);
	
	while (1)
	{
		switch(getPackage())
		{
			case SMALL_LENGTH:
				numberOfSmallLengthErrors++;
				break;
			case WRONG_CHECKSUM:
				numberOfChecksumErrors++;
				break;
			case SUCCESFULL_PACKAGE:
				numberOfReceivedPackages++;
			default:
				break;
		};
		checkCommandAndExecute();
		if (checkTimeout(I2CModule.timeOfLastI2CResetMillis, 1000))
		{
			rangeFinderCheckInterruptStatusOfSensor(RANGEFINDER_DEFAULT_ADDR, &state, RANGEFINDER_INTERRUPT_LEVEL_LOW);
			if (state)
			{
				rangeFinderReadMeasuredRange(RANGEFINDER_DEFAULT_ADDR, &range);
			}
		}
//  	if (state)
//		{
//			expanderReadInterrupt();
//			state = 0;
//		}
		//I2CCheckBus(&I2CModule);
		
		//I2CSearch(&I2CModule);
	}
}
