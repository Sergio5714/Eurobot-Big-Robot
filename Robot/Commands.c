#include "Commands.h"
extern Command_Struct inputCommand;
extern RobotStatus Robot;

void checkCommandAndExecute()
{
	if (inputCommand.status == 0x00)
	{
		// No command
		return;
	}
	switch (inputCommand.command)
	{
		case ECHO:
		{
			if (inputCommand.numberOfreceivedParams != 0x04)
				break;
			if (!((inputCommand.params[0] == 'E') && (inputCommand.params[1] == 'C') 
				&& (inputCommand.params[2] == 'H') && (inputCommand.params[3] == 'O')))
				break;
			uint8_t* answer = (uint8_t*)&"ECHO";
			sendAnswer(inputCommand.command, answer, 0x04);
			break;
		}	
		case SET_PWM:
		{
			if (inputCommand.numberOfreceivedParams != 0x05)
				break;
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			
			// Get motor Number and shift it
			uint8_t  motorNumber = inputCommand.params[0];
			motorNumber = motorNumber - 1;
			
			// Set pwm
			timPwmChangeDutyCycle(motorPwmCh[motorNumber].timModule,motorPwmCh[motorNumber].channel, *(__packed float*)(inputCommand.params +1));
			break;
		}
		case SET_DIR_BIT:
		{
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			
			// Get motor Number and shift it
			uint8_t  motorNumber = inputCommand.params[0];
			motorNumber = motorNumber - 1;
			
			gpioPinSetLevel(motorDir[motorNumber].port, motorDir[motorNumber].number, GPIO_LEVEL_HIGH);
			break;
		}
		case CLEAR_DIR_BIT:
		{
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			
			// Get motor Number and shift it
			uint8_t  motorNumber = inputCommand.params[0];
			motorNumber = motorNumber - 1;
			
			gpioPinSetLevel(motorDir[motorNumber].port, motorDir[motorNumber].number, GPIO_LEVEL_LOW);
			break;
		}
		case SET_ALL_MOTOR_SPEEDS:
		{
			// Check if there are at least one motor
			if (inputCommand.numberOfreceivedParams < 0x05)
				break;
			uint8_t numberOfMotors = inputCommand.params[0];
			if (inputCommand.numberOfreceivedParams != (0x01 + 0x04*numberOfMotors))
				break;
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			uint8_t motorNumber;
			for (motorNumber = 0x01; motorNumber <= numberOfMotors; motorNumber++)
			{
				// Set speed for all motors
				// Use __packed to avoid problem with data alignment
				setMotorSpeed(motorNumber, *(__packed float*)(inputCommand.params + 0x01 + 0x04*(motorNumber - 0x01)));
			}
			
			break;
		}
		case READ_ALL_WHEELS_SPEEDS:
		{
			// Check if number of wheels was received
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			uint8_t numberOfMotors = inputCommand.params[0];
			// Check if number of wheels exceeded real number or not
			if (numberOfMotors > ROBOT_NUMBER_OF_MOTORS)
				break;
			// Send Answer
			sendAnswer(inputCommand.command, (__packed uint8_t*)wheelsSpeed, 0x04*numberOfMotors);
			break;
		}
		case SET_SPEED_ROBOT_CS1:
		{
			// Check if 12 bytes (= 3 float) was received
			if (inputCommand.numberOfreceivedParams != 0x0C)
				break;
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			// Copy target speeds
			robotTargetSpeedCs1[0] = *(__packed float*)(inputCommand.params);
			robotTargetSpeedCs1[1] = *(__packed float*)(inputCommand.params + 0x04);
			robotTargetSpeedCs1[2] = *(__packed float*)(inputCommand.params + 0x08);
			break;
		}
		case READ_SPEED_ROBOT_CS1:
		{
			// Check if there is no parameters
			if (inputCommand.numberOfreceivedParams != 0x00)
				break;
			// Send Answer of 4 float (12 bytes)
			sendAnswer(inputCommand.command, (__packed uint8_t*)robotSpeedSc1, 0x0C);
			break;
		}
		case READ_COORD_ROBOT_CS1:
		{
			// Check if there is no parameters
			if (inputCommand.numberOfreceivedParams != 0x00)
				break;
			// Send Answer of 4 float (12 bytes)
			sendAnswer(inputCommand.command, (__packed uint8_t*)robotCoordSc1, 0x0C);
			uint8_t i = 0x01;
			for (i = 0x00; i < 0x03; i++)
			{
				robotCoordSc1[i] = 0;
			}
			break;
		}
		case SET_ANGLE_DNMX:
		{
			if (inputCommand.numberOfreceivedParams != 0x03)
				break;
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			
			// Set angle for particular servo motor
			setServoAngle(inputCommand.params[0], *(uint16_t*)(inputCommand.params + 1));
			break;
		}
		case GET_ANGLE_DNMX:
		{
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			float answerFloat;
	
			// Get angle from particular servo motor
			getServoAngle(inputCommand.params[0], &answerFloat);
			// send answer
			sendAnswer(inputCommand.command, (uint8_t*)&answerFloat, 0x04);
			break;
		}
		case TURN_FORW_KIN_ON_OFF:
		{
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			if (inputCommand.params[0] > 0x00)
			{
				Robot.forwardKinCalcStatusFlag = 0x01;
			}
			else
			{
				Robot.forwardKinCalcStatusFlag = 0x00;
			}
			break;
		}
		case GET_STATUS:
		{
			// Check if coordinates (x, y and angle) is received. All together 12 bytes
			if (inputCommand.numberOfreceivedParams != 0x0C)
				break;
			uint8_t buf [13];
			uint8_t i;
			buf[0] = Robot.movingStatusFlag;
			__packed uint8_t* bufPtr = (__packed uint8_t*)robotCoordSc1;
			for (i = 0x00; i < 0x0C; i++)
			{
				buf[i + 0x01] = *bufPtr;
				bufPtr++;
			}
			sendAnswer(inputCommand.command, (uint8_t*)&buf, 0x0D);
			break;
		}
	}
	// Command is already executed
	inputCommand.status = 0x00;
	return;
}
