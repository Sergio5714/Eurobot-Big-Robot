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

uint32_t prevMeasTime;
uint32_t timeDuration;


int main()
{	
    boardInitAll();
	initManipulators();
	Robot.forwardKinCalcStatusFlag = 0x01;
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
	}
}
