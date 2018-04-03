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

uint32_t startTime;
uint32_t readDuration;
uint32_t reinitDuration;

int main()
{		
   	boardInitAll();
	initManipulators();
	Robot.forwardKinCalcStatusFlag = 0x01;
	initRangeFindersGlobally();
	
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
		
		// get local time
		startTime = getLocalTime();
		
		readRangesGlobally();
		
		// calculate duration of read function
		readDuration = getLocalTime() - startTime;
		
		
		checkRangeFindersReinitFlags();
		// calculate duration of reinitialization
		reinitDuration = getLocalTime() - startTime - readDuration;
	}
}
