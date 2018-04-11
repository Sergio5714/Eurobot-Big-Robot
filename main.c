#include "stm32f4xx.h"
#include "Board.h"
#include "Communication.h"
#include "Manipulators.h"
#include "Collision_avoidance.h"


extern RobotStatus Robot;
// I2C module for rangefinders
extern I2C_Module_With_State_Typedef I2CModule;

// Communication errors
uint32_t numberOfReceivedPackages;
uint32_t numberOfChecksumErrors;
uint32_t numberOfSmallLengthErrors;

int main()
{	
	// Init everything 
    boardInitAll();
	// Upload angle values to manipulators
	initManipulators();
	// Turn on Forward kinematics calculations and Collision avoidance 
	Robot.forwardKinCalcStatusFlag = 0x01;
	Robot.collisionAvoidanceStatusFlag = 0x01;
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
