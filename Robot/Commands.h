#ifndef ROBOT_COMMANDS
#define ROBOT_COMMANDS

#include "Communication.h"

enum
{
	ECHO          = 0x01,
	SET_COORD     = 0x02,
	SET_PWM       = 0x03,
	SET_DIR_BIT   = 0x04,
	CLEAR_DIR_BIT = 0x05,
	ROTATE_DNMX   = 0x05
};

void checkCommandAndExecute(void);
#endif
