#ifndef EUROBOT_ROBOT
#define EUROBOT_ROBOT

#include "stm32f407xx.h"
#include "Board.h"
#include "matrix.h"
#include "stdlib.h"
#include "math.h"

// Encoder imitation mode
//#define ENCODER_IMITATION

#define PI_NUMBER                      3.14159265358f

// Common robot parameters
#define ROBOT_NUMBER_OF_MOTORS         0x04

// For SetMotorSpeed function to convert speed in rad/s to duty cycle (PWM)
// PWM = A*speed + B
#define ESCON_CALIBR_COEF_A            0.0293824f
#define ESCON_CALIBR_COEF_B            0.1f


// Minimum and maximum speed of motor/wheel in rad/s 
// (it corresponds to maximum duty cycle and minimum duty cycle)
#define MAX_ROT_SPEED                  27.22713f
#define MIN_ROT_SPEED                  0.0f
#define EPS_OF_ROT_SPEED               0.004f

// Parameters of motors
// Gear ratios 
#define MAXON_MOTOR_SHORT_GR           26.0f
#define MAXON_MOTOR_LONG_GR            21.0f

// Encoder's ticks per one rotation of initial shaft
#define MAXON_MOTOR_ENC_TICKS          4096

// Total number of ticks per one rotation
#define MAXON_MOTOR_LONG_TOTAL_TICKS   MAXON_MOTOR_LONG_GR * MAXON_MOTOR_ENC_TICKS
#define MAXON_MOTOR_SHORT_TOTAL_TICKS  MAXON_MOTOR_SHORT_GR * MAXON_MOTOR_ENC_TICKS

// Ticks to speed (rad/s) coefficient 
#define TICKS_TO_SPEED_COEF_LONG       2*PI_NUMBER / (MAXON_MOTOR_LONG_TOTAL_TICKS * MOTOR_CONTROL_PERIOD)
#define TICKS_TO_SPEED_COEF_SHORT      2*PI_NUMBER / (MAXON_MOTOR_SHORT_TOTAL_TICKS * MOTOR_CONTROL_PERIOD)

typedef struct 
{
	uint8_t movingStatusFlag;
	uint8_t forwardKinCalcStatusFlag;
}RobotStatus;

// For setMotorSpeed function
extern TIM_PWM_Typedef motorPwmCh[4];
extern GPIO_Pin_TypeDef motorDir[4];
extern GPIO_Pin_TypeDef motorEn[4];
extern uint16_t* encCnt[4];

// For kinematics calculation
extern float MLineSpeed[ROBOT_NUMBER_OF_MOTORS][3];
extern float MRotSpeed[ROBOT_NUMBER_OF_MOTORS][3];
extern float InverseKinematics[3][ROBOT_NUMBER_OF_MOTORS];

//--------------------------------------------- FUNCTIONS ------------------------------------------------------//

//--------------------------------------------- Functions for acquiring odometry and navigation-----------------//

// Calculate robot speed and coordinates in global coordinate system by using speeds in robot's coordinate system
void calcGlobSpeedAndCoord(void);

// Read data from encoders, calculate coordinates and speeds of wheels
void readEnc(void);

// Calculate forward kinematics (from speeds in robot's coordinate system to motors' speed)
void calcForwardKin(void);

// Calculate inverse kinematics (from wheels' speeds to speeds in robot's coordinate system)
void calcInverseKin(void);

//--------------------------------------------- Functions for motor control ------------------------------------//

// Set speed of particular wheel motorNumber = 1, 2, 3, 4 ...
void setMotorSpeed(uint8_t motorNumber, float speed);

// Set speeds for all motors (target motor speeds)
void setMotorSpeeds(void);

//--------------------------------------------- Other functions ------------------------------------------------//

// Check status
void updateRobotStatus(void);

// Maximum value of array
void maxValue(float *a, uint8_t rows,float *b);

// Normalize input angle to range [0, 2*pi)
void normalizeAngle(float* angle);

#endif
