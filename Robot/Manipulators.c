#include "Manipulators.h"

uint8_t servoSlidersId[3] = {MANIP_RIGHT_SERVO_SLIDER_ID,
                            MANIP_LEFT_SERVO_SLIDER_ID,
							MANIP_CENTRAL_SERVO_SLIDER_ID};

uint16_t servoSlidersTopPos[3] = {MANIP_RIGHT_SERVO_SLIDER_TOP_POS,
                                MANIP_LEFT_SERVO_SLIDER_TOP_POS,
                                MANIP_CENTRAL_SERVO_SLIDER_TOP_POS};

uint16_t servoSlidersBotPos[3] = {MANIP_RIGHT_SERVO_SLIDER_TOP_POS,
                                MANIP_LEFT_SERVO_SLIDER_TOP_POS,
                                MANIP_CENTRAL_SERVO_SLIDER_TOP_POS};

uint8_t servoGrippersId[3] = {MANIP_RIGHT_SERVO_GRIPPER_ID,
                            MANIP_LEFT_SERVO_GRIPPER_ID,
							MANIP_CENTRAL_SERVO_GRIPPER_ID};
								
uint16_t servoGrippersOpenedPos[3] = {MANIP_RIGHT_SERVO_GRIPPER_OPENED_POS,
                                   MANIP_LEFT_SERVO_GRIPPER_OPENED_POS,
                                   MANIP_CENTRAL_SERVO_GRIPPER_OPENED_POS};

uint16_t servoGrippersClosedPos[3] = {MANIP_RIGHT_SERVO_GRIPPER_CLOSED_POS,
                                    MANIP_LEFT_SERVO_GRIPPER_CLOSED_POS,
                                    MANIP_CENTRAL_SERVO_GRIPPER_CLOSED_POS};

//--------------------------------------------- Low level actions ----------------------------------------------//
// manipulatorNumber = 0-2 that corresponds to "Right, Left, Central"
void openManipulator(uint8_t manipulatorNumber)
{
	setServoAngle(servoGrippersId[manipulatorNumber],servoGrippersOpenedPos[manipulatorNumber]);
	return;
}

void closeManipulator(uint8_t manipulatorNumber)
{
	setServoAngle(servoGrippersId[manipulatorNumber],servoGrippersClosedPos[manipulatorNumber]);
	return;
}

void liftManipulator(uint8_t manipulatorNumber)
{
	setServoAngle(servoSlidersId[manipulatorNumber],servoSlidersTopPos[manipulatorNumber]);
	return;
}
void lowerManipulator(uint8_t manipulatorNumber)
{
	setServoAngle(servoSlidersId[manipulatorNumber],servoSlidersBotPos[manipulatorNumber]);
	return;
}

//--------------------------------------------- High level actions ----------------------------------------------//
void takeCube(uint8_t manipulatorNumber)
{
	return;
}
void unloadTower(uint8_t manipulatorNumber)
{
	return;
}
