#include "Robot.h"

// Struct for current robot status 
RobotStatus Robot;

// Values, calculated by using encoder's data (input)
// Speeds (rps) and coordinates (rotations) of particular motor
float wheelsSpeed[ROBOT_NUMBER_OF_MOTORS];
float wheelsCoord[ROBOT_NUMBER_OF_MOTORS];

// Speeds and coordinates of in robot coordinate system
float robotSpeedSc1[3];
float robotCoordSc1[3];

// Value's that are calculated by STM or Raspberry pi and used for output
// Target speeds and coordinates of robot in robot's coordinate system
float robotTargetSpeedCs1[3];
float robotTargetMotorSpeedCs1[ROBOT_NUMBER_OF_MOTORS];

// Set speed of particular motor
void setMotorSpeed(uint8_t motorNumber, float speed)
{
	if (motorNumber > ROBOT_NUMBER_OF_MOTORS)
	{
		return;
	}
	float duty;
	
	// Shift of motor numeration
	motorNumber = motorNumber - 0x01;
	
	// Set direction
	if (speed >= 0)
	{
		gpioPinSetLevel(motorDir[motorNumber].port, motorDir[motorNumber].number, GPIO_LEVEL_HIGH);
	}
	else
	{
		gpioPinSetLevel(motorDir[motorNumber].port, motorDir[motorNumber].number, GPIO_LEVEL_LOW);
		speed = - speed;
	}
	
	if (speed >= MAX_ROT_SPEED)
	{
		speed = MAX_ROT_SPEED;
	}
	else if (speed < MIN_ROT_SPEED)
	{
		speed = MIN_ROT_SPEED;
	}
	// Calculate duty cycle
	duty = speed*(float)ESCON_CALIBR_COEF_A + (float)ESCON_CALIBR_COEF_B;
	
	// Change PWM
	timPwmChangeDutyCycle(motorPwmCh[motorNumber].timModule, motorPwmCh[motorNumber].channel, duty);
	return;
}

// Calculate forward kinematics (from desired speeds in robot coordinate system to motor's speed)
void calcForwardKin(void)
{
	float vLine[ROBOT_NUMBER_OF_MOTORS], vRot[ROBOT_NUMBER_OF_MOTORS], vSum[ROBOT_NUMBER_OF_MOTORS];
	float maxMotorSpeed;
	float correctionCoef;
	
	// Note : matrix of forward kinematics already takes into account 2 cylindrical transmissions (wheels 2 and 4)
	// Calculate linear speed
	matrixMultiplyM2M(&MLineSpeed[0][0], ROBOT_NUMBER_OF_MOTORS, 3, &robotTargetSpeedCs1[0], 3, 1, &vLine[0]);
	
	// Calculate rotational speed
	matrixMultiplyM2M(&MRotSpeed[0][0], ROBOT_NUMBER_OF_MOTORS, 3, &robotTargetSpeedCs1[0], 3, 1, &vRot[0]);
	
	// Sum speeds
	matrixPlusMinus(&vLine[0], &vRot[0], ROBOT_NUMBER_OF_MOTORS, 1, 1, &vSum[0]);
	
	// Find max value all motor's speeds
	MaxValue(&vSum[0], ROBOT_NUMBER_OF_MOTORS, &maxMotorSpeed);
	
	// Check if we exceed maximum available motor speed
	if (fabs(maxMotorSpeed) > MAX_ROT_SPEED)
	{
		correctionCoef = fabs(MAX_ROT_SPEED / maxMotorSpeed);
	} 
	else 
	{
		correctionCoef = 1.0;
	}
	matrixMultiplyS2M(&vSum[0], ROBOT_NUMBER_OF_MOTORS, 1, correctionCoef, &robotTargetMotorSpeedCs1[0]);
	return;
}
// Calculate inverse kinematics (from encoder's wheels' speeds to speeds in robot's coordinate system)
void calcInverseKin(void)
{
	// Multiply inverse kinematics matrix by motor's speed
	matrixMultiplyM2M(&InverseKinematics[0][0], 3, ROBOT_NUMBER_OF_MOTORS, &wheelsSpeed[0], ROBOT_NUMBER_OF_MOTORS, 1, &robotSpeedSc1[0]);
}

// Set speeds for all motors
void setMotorSpeeds(void)
{
	uint8_t i;
	for(i = 0; i < ROBOT_NUMBER_OF_MOTORS; i++)
	{
	setMotorSpeed(i + 0x01, robotTargetMotorSpeedCs1[i]);
	}
	return;
}

// Maximum value of array
void MaxValue(float *a,uint8_t rows,float *b)
{
	uint8_t i;
	*b = fabs(*a);
	for (i = 0x01; (i <= rows - 0x01); i++)
	{
		if ((*b) < fabs(*(a+i)))
		*b = fabs(*(a + i));
	}
	return;
}

// Read data from encoders, calculate wheels coordinates and speeds in robot's coordinate system
void readEnc(void)
{
	uint8_t i;
	int16_t encTicksBuf[ROBOT_NUMBER_OF_MOTORS];
	
	// Get data from encoders
	for (i = 0x00; i < ROBOT_NUMBER_OF_MOTORS; i++)
	{
		encTicksBuf[i] = *encCnt[i] - ENCODER_CNT_INITIAL_VALUE;
		*encCnt[i] = ENCODER_CNT_INITIAL_VALUE;
	}
	
	// Calculate coordinates and speeds
	for (i = 0x00; i < ROBOT_NUMBER_OF_MOTORS; i++)
	{
		// Actually now it is only motor's speed, but not wheel speed
		wheelsSpeed[i] = encTicksBuf[i] * TICKS_TO_SPEED_COEF_SHORT;
		// Now one motor (1st and 3rd) is especial (20.12.2017)
		if ((i == 0) || (i == 2))
		{
			wheelsSpeed[i] = encTicksBuf[i] * TICKS_TO_SPEED_COEF_LONG;	
		}
	}
	
	// Inverse motors 2 and 4 because of cylindrical transmission (Positive direction is CCW)
	wheelsSpeed[1] = - wheelsSpeed[1];
	wheelsSpeed[3] = - wheelsSpeed[3];
	
	// Calculate distance that wheel passed
	for (i = 0x00; i < ROBOT_NUMBER_OF_MOTORS; i++)
	{
		wheelsCoord[i] += wheelsSpeed[i] * MOTOR_CONTROL_PERIOD;
	}
	
	// Calculation of inverse kinematics (wheelsSpeed -> robotSpeedSc1)
	calcInverseKin();
	for (i = 0x00; i < 0x03; i++)
	{
		robotCoordSc1[i] += robotSpeedSc1[i] * MOTOR_CONTROL_PERIOD;
	}
	return;
}

// Check status
void updateRobotStatus(void)
{
	uint8_t i;
	for ( i = 0x00; i < ROBOT_NUMBER_OF_MOTORS; i++ )
	{
		if (wheelsSpeed[i] > EPS_OF_ROT_SPEED)
		{
			Robot.movingStatusFlag = 0x01;
			return;
		}
	}
	Robot.movingStatusFlag = 0x01;
	return;
}
