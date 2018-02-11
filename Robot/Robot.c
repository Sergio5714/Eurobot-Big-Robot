#include "Robot.h"

// Struct for current robot status 
RobotStatus Robot;

// Struct for odometry movement
OdometryMovementStruct OdometryMovement;

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

// Accuracy of movement and acceleration for odometry movement
float accuracyOfMovement[3] = {MOVEMENT_XY_ACCURACY, MOVEMENT_XY_ACCURACY, MOVEMENT_ANGULAR_ACCURACY};
float acceleration[3] = {ODOMETRY_MOVEMENT_ACCELERATION_XY, ODOMETRY_MOVEMENT_ACCELERATION_XY, ODOMETRY_MOVEMENT_ACCELERATION_W};

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

//--------------------------------------------- Functions for movement -----------------------------------------//
void startMovementRobotCs1(float* distance, float* speed)
{
	uint8_t i;
	for (i = 0x00; i < 0x03; i++)
	{
		// 1 Check if distance is non equal to 0
		if ((distance[i] != 0.0f) && (speed[i] > 0.0f))
		{
			if (distance[i] > 0)
			{
				OdometryMovement.stableSpeed[i] = speed[i];
				OdometryMovement.acceleration[i] = acceleration[i];
				OdometryMovement.direction[i] = 1;
			}
			else
			{
				OdometryMovement.stableSpeed[i] = -speed[i];
				OdometryMovement.acceleration[i] = -acceleration[i];
				OdometryMovement.direction[i] = -1;
			}
			
			// 2 Set current coordinate in own system to 0
			robotCoordCs1[i] = 0.0f;
			
			// 3 Copy distance to be passed
			OdometryMovement.robotTargetDistanceCs1[i] = distance[i];
			
			// 4 Optimize direction of robots rotation and angle
			if (i == 0x02)
			{
				if (fabs(distance[i]) > 2 * PI_NUMBER)
				{
					return;
				}
				
				// Normalize angle
				normalizeAngle(&distance[i]);
				
				if (distance[i] > PI_NUMBER)
				{
					OdometryMovement.robotTargetDistanceCs1[i] = distance[i] -  2 * PI_NUMBER;
					OdometryMovement.acceleration[i] = -acceleration[i];
					OdometryMovement.stableSpeed[i] = -speed[i];
					OdometryMovement.direction[i] = -1;
				}
				else
				{
					OdometryMovement.robotTargetDistanceCs1[i] = distance[i];
					OdometryMovement.acceleration[i] = acceleration[i];
					OdometryMovement.stableSpeed[i] = speed[i];
					OdometryMovement.direction[i] = 1;
				}
			}
			
			// 5 Calculate speed increment after all direction are defined
			OdometryMovement.speedIncrement[i] = OdometryMovement.acceleration[i] * MOTOR_CONTROL_PERIOD;
			
			// 6 Calculation of essential parameters of the trajectory
			calculateTrajectParameters(i);
			
			OdometryMovement.odometryMovementStatusFlag[i] = ODOMETRY_MOVEMENT_ACCELERATION;
		}
	}
	
	Robot.odometryMovingStatusFlag = 0x01;
	return;
}
void calculateTrajectParameters(uint8_t i)
{
	float deltaCoord = OdometryMovement.direction[i] * OdometryMovement.stableSpeed[i] * OdometryMovement.stableSpeed[i] / (0x02 * acceleration[i]);
	// Point of trajectory to stop acceleration
	OdometryMovement.stopAccCoordinate[i] = deltaCoord;
	// Point of trajectory to start decceleration
	OdometryMovement.startDeccCoordinate[i] = OdometryMovement.robotTargetDistanceCs1[i] - deltaCoord;
	
	// If distance is too short, then correct stable speed
	if (fabs(OdometryMovement.startDeccCoordinate[i]) < fabs(OdometryMovement.stopAccCoordinate[i]))
	{
		OdometryMovement.stableSpeed[i] = OdometryMovement.direction[i] * sqrt(OdometryMovement.acceleration[i] * OdometryMovement.robotTargetDistanceCs1[i]);
		OdometryMovement.stopAccCoordinate[i] = OdometryMovement.robotTargetDistanceCs1[i] / 2;
		OdometryMovement.startDeccCoordinate[i] = OdometryMovement.stopAccCoordinate[i];
	}
	return;

}
void speedRecalculation(void)
{
	uint8_t i;
	for (i = 0x00; i < 0x03; i++)
	{
		switch(OdometryMovement.odometryMovementStatusFlag[i])
		{
			case ODOMETRY_MOVEMENT_NO_MOVEMENT:
				break;
			
			case ODOMETRY_MOVEMENT_ACCELERATION:
				robotTargetSpeedCs1[i] = robotTargetSpeedCs1[i] + OdometryMovement.acceleration[i] * MOTOR_CONTROL_PERIOD;
				// Constrain increase in speed if coordinate is still less than stopAccCoord[i]
				if (fabs(robotTargetSpeedCs1[i]) > fabs(OdometryMovement.stableSpeed[i]))
				{
					robotTargetSpeedCs1[i] = OdometryMovement.stableSpeed[i];
				}					
				break;
				
			case ODOMETRY_MOVEMENT_STABLE_SPEED:
				
				// Keep constant speed
				robotTargetSpeedCs1[i] = OdometryMovement.stableSpeed[i];
				break;
			
			case ODOMETRY_MOVEMENT_DECCELERATION:
				// Check if speed is less than spee increment
				if (fabs(robotTargetSpeedCs1[i]) < fabs(OdometryMovement.speedIncrement[i]))
				{
					robotTargetSpeedCs1[i] = 0.0f;
				}
				else
				{
					robotTargetSpeedCs1[i] = robotTargetSpeedCs1[i] - OdometryMovement.speedIncrement[i];
				}	
				break;
		}
	}
	return;
}
// Check if we reached target position is reached or not
void checkIfPositionIsReached(void)
{
	uint8_t i;
	float robotCoordBuf[3];
	
	// Copy coordinates from robotCoordCs1[i] to robotCoordBuf[i]
	for (i = 0x00; i < 0x03; i++)
	{
		robotCoordBuf[i] = robotCoordCs1[i];
	}
	// Change representation of angle if it is needed
	if (robotCoordBuf[2] > PI_NUMBER)
	{
		robotCoordBuf[2] = robotCoordBuf[2] - 2 * PI_NUMBER;
	}
	
	for (i = 0x00; i < 0x03; i++)
	{
		if (OdometryMovement.odometryMovementStatusFlag[i] != ODOMETRY_MOVEMENT_NO_MOVEMENT)
		{
			// Check if we reached final position or not
			if (fabs(OdometryMovement.robotTargetDistanceCs1[i] - robotCoordBuf[i]) < accuracyOfMovement[i])
			{
				// Stop motors
				robotTargetSpeedCs1[i] = 0.0;
				OdometryMovement.odometryMovementStatusFlag[i] = ODOMETRY_MOVEMENT_NO_MOVEMENT;
				return;
			}
			else
			{
				Robot.odometryMovingStatusFlag = 0x01;
			}
			// Check if mode should be changed
			if ((fabs(robotCoordBuf[i]) < fabs(OdometryMovement.stopAccCoordinate[i])) && (fabs(robotSpeedCs1[i]) < fabs(OdometryMovement.stableSpeed[i])))
			{
				OdometryMovement.odometryMovementStatusFlag[i] = ODOMETRY_MOVEMENT_ACCELERATION;
				return;
			}
			else if (fabs(robotCoordBuf[i]) < fabs(OdometryMovement.startDeccCoordinate[i]))
			{
				OdometryMovement.odometryMovementStatusFlag[i] = ODOMETRY_MOVEMENT_STABLE_SPEED;
				return;
			}
			else
			{
				OdometryMovement.odometryMovementStatusFlag[i] = ODOMETRY_MOVEMENT_DECCELERATION;
			}
		}
	}
	return;
}

void initOdometryMovement(void)
{
	OdometryMovement.acceleration[0] = ODOMETRY_MOVEMENT_ACCELERATION_XY;
	OdometryMovement.acceleration[1] = ODOMETRY_MOVEMENT_ACCELERATION_XY;
	OdometryMovement.acceleration[2] = ODOMETRY_MOVEMENT_ACCELERATION_W;
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
