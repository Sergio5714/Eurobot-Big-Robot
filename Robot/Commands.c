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
		case GET_ALL_WHEELS_SPEEDS:
		{
			// Check if number of wheels was received
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			uint8_t numberOfMotors = inputCommand.params[0];
			// Check if number of wheels exceeded real number or not
			if (numberOfMotors > ROBOT_NUMBER_OF_MOTORS)
				break;
			// Save current coord to send data correctly (process can be interrupted and values can be changed)
			float speedBuf[numberOfMotors];
			uint8_t i;
			for (i = 0x00; i < numberOfMotors; i++ )
			{
				speedBuf[i] = wheelsSpeed[i];
			}
			// Send Answer
			sendAnswer(inputCommand.command, (__packed uint8_t*)speedBuf, 0x04*numberOfMotors);
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
		case GET_SPEED_ROBOT_CS1:
		{
			// Check if there is no parameters
			if (inputCommand.numberOfreceivedParams != 0x00)
				break;
			// Save current coord to send data correctly (process can be interrupted and values can be changed)
			float speedBuf[3];
			uint8_t i;
			for (i = 0x00; i < 0x03; i++ )
			{
				speedBuf[i] = robotSpeedCs1[i];
			}
			// Send Answer of 4 float (12 bytes)
			sendAnswer(inputCommand.command, (__packed uint8_t*)speedBuf, 0x0C);
			break;
		}
		case GET_COORD_ROBOT_CS1:
		{
			// Check if there is no parameters
			if (inputCommand.numberOfreceivedParams != 0x00)
				break;
			// Save current coord to send data correctly (process can be interrupted and values can be changed)
			float coordBuf[3];
			uint8_t i;
			for (i = 0x00; i < 0x03; i++ )
			{
				coordBuf[i] = robotCoordCs1[i];
			}
			// Send Answer of 4 float (12 bytes)
			sendAnswer(inputCommand.command, (__packed uint8_t*)coordBuf, 0x0C);
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
		case SET_COORD_ROBOT_CS_GLOBAL:
		{
			// Check if 12 bytes (= 3 float) was received
			if (inputCommand.numberOfreceivedParams != 0x0C)
				break;
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			// Copy target speeds
			robotCoordCsGlobal[0] = *(__packed float*)(inputCommand.params);
			robotCoordCsGlobal[1] = *(__packed float*)(inputCommand.params + 0x04);
			robotCoordCsGlobal[2] = *(__packed float*)(inputCommand.params + 0x08);
			break;
		}
		case GET_COORD_ROBOT_CS_GLOBAL:
		{
			// Check if there is no parameters
			if (inputCommand.numberOfreceivedParams != 0x00)
				break;
			// Save current coord to send data correctly (process can be interrupted and values can be changed)
			float coordBuf[3];
			uint8_t i;
			for (i = 0x00; i < 0x03; i++ )
			{
				coordBuf[i] = robotCoordCsGlobal[i];
			}
			// Send Answer of 4 float (12 bytes)
			sendAnswer(inputCommand.command, (__packed uint8_t*)coordBuf, 0x0C);
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
			__packed uint8_t* bufPtr = (__packed uint8_t*)robotCoordCs1;
			for (i = 0x00; i < 0x0C; i++)
			{
				buf[i + 0x01] = *bufPtr;
				bufPtr++;
			}
			sendAnswer(inputCommand.command, (uint8_t*)&buf, 0x0D);
			break;
		}
		case TAKE_CUBE:
		{
			// Check if manipulator's id is received
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			uint8_t number = inputCommand.params[0];
			setManipHighLevelCommand(TAKE_CUBE_COMMAND, number, &cubeManipulators[number]);
			// Send answer
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			break;
		}
		case UNLOAD_TOWER:
		{
			// Check if manipulator's id is received
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			uint8_t number = inputCommand.params[0];
			setManipHighLevelCommand(UNLOAD_TOWER_COMMAND, number, &cubeManipulators[number]);
			// Send answer
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			break;
		}
		case ODOMETRY_MOVEMENT:
		{
			// Check if 6 float numbers or 24 bytes are received
			if (inputCommand.numberOfreceivedParams != 0x18)
				break;
			uint8_t i;
			float distance[3];
			float speed[3];
			float acceleration[3];
			for (i = 0x00; i < 0x03; i++)
			{
				distance[i] = *(__packed float*)(inputCommand.params + 0x04*i);
				speed[i] = *(__packed float*)(inputCommand.params + 0x04*i + 0x0C);
				acceleration[i] = accelerationMax[i];
			}
			startMovementRobotCs1(&distance[0], &speed[0], &acceleration[0]);
			// Send answer
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			break;
		}
	}
	// Command is already executed
	inputCommand.status = 0x00;
	return;
}
