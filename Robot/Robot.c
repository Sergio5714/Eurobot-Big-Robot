#include "Robot.h"

// Struct for current robot status 
RobotStatus Robot;

// Values, calculated by using encoder's data
// Speeds (rad/s) and coordinates (rad) of particular wheel
float wheelsSpeed[ROBOT_NUMBER_OF_MOTORS];
float wheelsCoord[ROBOT_NUMBER_OF_MOTORS];

// Instantaneous speeds and coordinates of robot in global coordinate system
float robotSpeedCsGlobal[3];
float robotCoordCsGlobal[3];

// Instantaneous speeds and coordinates of robot in robot coordinate system
float robotSpeedCs1[3];
float robotCoordCs1[3];

// Value's that are calculated by STM or Raspberry pi and used for output
// Target speeds and coordinates of robot in robot's coordinate system
float robotTargetSpeedCs1[3];
float robotTargetMotorSpeedCs1[ROBOT_NUMBER_OF_MOTORS];

//--------------------------------------------- Functions for acquiring odometry and navigation-----------------//

// Calculate robot speed and coordinates in global coordinate system by using speeds in robot's coordinate system
void calcGlobSpeedAndCoord(void)
{
	float alphaRobot = robotCoordCsGlobal[2];
	// Calculation of matrix that transforms speed from robot coordinate system to global one
	float transfMatrix [2][2] = { {cos(alphaRobot), -sin(alphaRobot)},
	                              {sin(alphaRobot), cos(alphaRobot)}};
	// Transformation
	// Linear speeds
	matrixMultiplyM2M(&transfMatrix[0][0], 2, 2, &robotSpeedCs1[0], 2, 1, &robotSpeedCsGlobal[0]);
	// Rotational speed
	robotSpeedCsGlobal[2] = robotSpeedCs1[2];
								  
	// Coordinates' increment calculation
	robotCoordCsGlobal[0] += robotSpeedCsGlobal[0] * MOTOR_CONTROL_PERIOD;
	robotCoordCsGlobal[1] += robotSpeedCsGlobal[1] * MOTOR_CONTROL_PERIOD;
	robotCoordCsGlobal[2] += robotSpeedCsGlobal[2] * MOTOR_CONTROL_PERIOD;
	// Normalize angle of rotation
	normalizeAngle(&robotCoordCsGlobal[2]);
	return;
}

// Read data from encoders, calculate coordinates and speeds of wheel
void readEnc(void)
{
	// Counter
	uint8_t i;
	// Buffer for encoders' ticks
	int16_t encTicksBuf[ROBOT_NUMBER_OF_MOTORS];
	
	#ifdef ENCODER_IMITATION
	// Imitation of movement
	for (i = 0x00; i < ROBOT_NUMBER_OF_MOTORS; i++)
	{
		// Actually now it is only motor's speed, but not wheel speed
		wheelsSpeed[i] = robotTargetMotorSpeedCs1[i];
	}
	#else
	// Get data from encoders
	for (i = 0x00; i < ROBOT_NUMBER_OF_MOTORS; i++)
	{
		encTicksBuf[i] = *encCnt[i] - ENCODER_CNT_INITIAL_VALUE;
		*encCnt[i] = ENCODER_CNT_INITIAL_VALUE;
	}
	
	// Calculate speeds
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
	#endif
	
	// Inverse motors 2 and 4 because of cylindrical transmission (Positive direction is CCW)
	wheelsSpeed[1] = - wheelsSpeed[1];
	wheelsSpeed[3] = - wheelsSpeed[3];
	
	// Calculate distance that wheel passed
	for (i = 0x00; i < ROBOT_NUMBER_OF_MOTORS; i++)
	{
		wheelsCoord[i] += wheelsSpeed[i] * MOTOR_CONTROL_PERIOD;
	}
	
	// Calculate inverse kinematics (wheelsSpeed -> robotSpeedCs1)
	calcInverseKin();
	// Calculate instanteneous coordinates in robot cpprdinate system
	for (i = 0x00; i < 0x03; i++)
	{
		robotCoordCs1[i] += robotSpeedCs1[i] * MOTOR_CONTROL_PERIOD;
	}
	normalizeAngle(&robotCoordCs1[2]);
	return;
}

// Calculate forward kinematics (from speeds in robot's coordinate system to motors' speed)
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
	maxValue(&vSum[0], ROBOT_NUMBER_OF_MOTORS, &maxMotorSpeed);
	
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
	// Multiply inverse kinematics matrix by wheels' speeds
	matrixMultiplyM2M(&InverseKinematics[0][0], 3, ROBOT_NUMBER_OF_MOTORS, &wheelsSpeed[0], ROBOT_NUMBER_OF_MOTORS, 1, &robotSpeedCs1[0]);
}

//--------------------------------------------- Functions for motor control ------------------------------------//

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

// Set speeds for all motors by using target motors' speed array
void setMotorSpeeds(void)
{
	uint8_t i;
	for(i = 0; i < ROBOT_NUMBER_OF_MOTORS; i++)
	{
	setMotorSpeed(i + 0x01, robotTargetMotorSpeedCs1[i]);
	}
	return;
}


//--------------------------------------------- Other functions ------------------------------------------------//

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
	Robot.movingStatusFlag = 0x00;
	return;
}

// Maximum value of array
void maxValue(float *a,uint8_t rows,float *b)
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

// Normalize input angle to range [0, 2*pi)
void normalizeAngle(float* angle)
{
	if (*angle >= 2*PI_NUMBER)
	{
		*angle = *angle - 2 * PI_NUMBER;
	}
	else if (*angle < 0)
	{
		*angle = *angle + 2 * PI_NUMBER; 
	}
	return;
}
