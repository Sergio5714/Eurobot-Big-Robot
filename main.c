#include "stm32f4xx.h"
#include "Board.h"
#include "Communication.h"
#include "Manipulators.h"

extern RobotStatus Robot;
uint32_t numberOfReceivedPackages;
uint32_t numberOfChecksumErrors;
uint32_t numberOfSmallLengthErrors;

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
