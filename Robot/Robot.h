#ifndef EUROBOT_ROBOT
#define EUROBOT_ROBOT

#include "stm32f407xx.h"
#include "Board.h"
#include "matrix.h"
#include "stdlib.h"
#include "math.h"

#define PI_NUMBER                      3.14159265358

// Common robot parameters
#define ROBOT_NUMBER_OF_MOTORS         0x04

// For SetMotorSpeed function to convert speed in rad/s to duty cycle (PWM)
// PWM = A*speed + B
#define ESCON_CALIBR_COEF_A            0.0293824
#define ESCON_CALIBR_COEF_B            0.1


// Minimum and maximum speed of motor/wheel in rad/s 
// (it corresponds to maximum duty cycle of 0.9 and minimum duty cycle of 0.1)
#define MAX_ROT_SPEED                  27.22713
#define MIN_ROT_SPEED                  0.0
#define EPS_OF_ROT_SPEED               0.004

// Parameters of motors
// Gear ratios 
#define MAXON_MOTOR_SHORT_GR           26.0
#define MAXON_MOTOR_LONG_GR            21.0

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
extern float MLineSpeed[4][3];
extern float MRotSpeed[4][3];
extern float InverseKinematics[3][4];

// Set speed of particular wheel motorNumber = 1, 2, 3, 4 ...
void setMotorSpeed(uint8_t motorNumber, float speed);

// Set speeds for all motors
void setMotorSpeeds(void);

// Read data from encoders, calculate wheels coordinates and speeds
void readEnc(void);

// Calculate forward kinematics (from desired speeds to motor's speed)
void calcForwardKin(void);

// Calculate inverse kinematics (from encoder's speed to global speeds in robot's SC)
void calcInverseKin(void);

// Check status
void updateRobotStatus(void);

// Maximum value of array
void MaxValue(float *a, uint8_t rows,float *b);

#endif
