#ifndef MANIPULATORS
#define MANIPULATORS
#include "Robot.h"
#include "Dynamixel_control.h"

//--------------------------------------------- DYNAMIXEL's INFO ------------------------------------------------//
// P.S Left, right and central manipulators when we are in front of robot
// ID of servo motors
#define MANIP_RIGHT_SERVO_SLIDER_ID             0x01
#define MANIP_RIGHT_SERVO_GRIPPER_ID            0x16

#define MANIP_LEFT_SERVO_SLIDER_ID              0x02
#define MANIP_LEFT_SERVO_GRIPPER_ID             0x17

#define MANIP_CENTRAL_SERVO_SLIDER_ID           0x03  // To be mounted (11.01.2018)
#define MANIP_CENTRAL_SERVO_GRIPPER_ID          0x18  // To be mounted (11.01.2018)

// Boundary angles
#define MANIP_RIGHT_SERVO_SLIDER_TOP_POS        0x2D  // 45°
#define MANIP_RIGHT_SERVO_SLIDER_BOT_POS        0x12C // 300°
#define MANIP_RIGHT_SERVO_GRIPPER_OPENED_POS    0x28  // 40°
#define MANIP_RIGHT_SERVO_GRIPPER_CLOSED_POS    0x96  // 150°

#define MANIP_LEFT_SERVO_SLIDER_TOP_POS         0x2D  // TBD!
#define MANIP_LEFT_SERVO_SLIDER_BOT_POS         0x12C // TBD!
#define MANIP_LEFT_SERVO_GRIPPER_OPENED_POS     0x28  // TBD!
#define MANIP_LEFT_SERVO_GRIPPER_CLOSED_POS     0x96  // TBD!

#define MANIP_CENTRAL_SERVO_SLIDER_TOP_POS      0x2D  // TBD!
#define MANIP_CENTRAL_SERVO_SLIDER_BOT_POS      0x12C // TBD!
#define MANIP_CENTRAL_SERVO_GRIPPER_OPENED_POS  0x28  // TBD!
#define MANIP_CENTRAL_SERVO_GRIPPER_CLOSED_POS  0x96  // TBD!

//--------------------------------------------- FUNCTIONS ------------------------------------------------------//

//--------------------------------------------- Low level actions ----------------------------------------------//
// manipulatorNumber = 0-2 that corresponds to "Right, Left, Central"
void openManipulator(uint8_t manipulatorNumber);
void closeManipulator(uint8_t manipulatorNumber);
void liftManipulator(uint8_t manipulatorNumber);
void lowerManipulator(uint8_t manipulatorNumber);

//--------------------------------------------- High level actions ----------------------------------------------//
void takeCube(uint8_t manipulatorNumber);
void unloadTower(uint8_t manipulatorNumber);

#endif
