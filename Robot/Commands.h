#ifndef ROBOT_COMMANDS
#define ROBOT_COMMANDS

#include "Communication.h"
#include "Robot.h"

extern float wheelsSpeed[ROBOT_NUMBER_OF_MOTORS];
extern float robotTargetSpeedCs1[3];
extern float robotSpeedSc1[3];
extern float robotCoordSc1[3];

enum
{
	ECHO                   = 0x01,
	
	
	SET_PWM                = 0x03,
	SET_DIR_BIT            = 0x04,
	CLEAR_DIR_BIT          = 0x05,
	
	SET_ALL_MOTOR_SPEEDS   = 0x06,
	READ_ALL_WHEELS_SPEEDS = 0x07,
	
	SET_SPEED_ROBOT_CS1    = 0x08,
	READ_SPEED_ROBOT_CS1   = 0x09,
	
	READ_COORD_ROBOT_CS1   = 0x0A,
	SET_ANGLE_DNMX         = 0x0B,
	GET_ANGLE_DNMX         = 0x0C,
	
	TURN_FORW_KIN_ON_OFF   = 0x0D,
	
	GET_STATUS             = 0xA0,
};

void checkCommandAndExecute(void);
#endif
