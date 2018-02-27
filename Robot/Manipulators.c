#include "Manipulators.h"

Cube_Manipulator_Typedef cubeManipulators[NUMBER_OF_MANIPULATORS];
Servo_Checker_Typedef servoChecker[NUMBER_OF_MANIPULATORS];

// Task sequence for taking a cube
Manipulator_Subtasks_Typedef takeCubetaskSeq[] = {SUBTASK_OPEN_MANIPULATOR, SUBTASK_LOWER_MANIPULATOR,
                                                  SUBTASK_CLOSE_MANIPULATOR, SUBTASK_LIFT_MANIPULATOR,
                                                  SUBTASK_TERMINATOR};
// Task sequence for taking last cube
Manipulator_Subtasks_Typedef takeLastCubetaskSeq[] = {SUBTASK_OPEN_MANIPULATOR, SUBTASK_LOWER_MANIPULATOR,
                                                      SUBTASK_CLOSE_MANIPULATOR, SUBTASK_LIFT_MANIPULATOR_INTERM,
                                                      SUBTASK_TERMINATOR};
// Task sequence for unloading tower
Manipulator_Subtasks_Typedef unloadTowertaskSeq[] = {SUBTASK_LOWER_MANIPULATOR, SUBTASK_OPEN_MANIPULATOR, 
                                                     SUBTASK_LIFT_MANIPULATOR, SUBTASK_TERMINATOR};

uint32_t ticks;
uint32_t deltaticks;
// Timer interrupt handler for servoChecker
void TIM5_IRQHandler(void)
{

	if (SERVO_CHECKER_TIM_MODULE->SR & TIM_SR_UIF)
	{	
		timClearStatusRegisterFlag(SERVO_CHECKER_TIM_MODULE, TIM_SR_UIF);
		ticks = SERVO_CHECKER_TIM_MODULE->CNT;
		// Check manipulators
		execManipTasks(0x00, &cubeManipulators[0]);
		execManipTasks(0x01, &cubeManipulators[1]);
		execManipTasks(0x02, &cubeManipulators[2]);
		checkPosServo(&servoChecker[0]);
		checkPosServo(&servoChecker[1]);
		checkPosServo(&servoChecker[2]);
		deltaticks = SERVO_CHECKER_TIM_MODULE->CNT - ticks;
	}
	return;
}

//--------------------------------------------- FUNCTIONS ------------------------------------------------------//
void initManipulators(void)
{
	cubeManipulators[0].gripper.id = MANIP_LEFT_SERVO_GRIPPER_ID;
	cubeManipulators[0].gripper.closedAngle = MANIP_LEFT_SERVO_GRIPPER_CLOSED_POS;
	cubeManipulators[0].gripper.openedAngle = MANIP_LEFT_SERVO_GRIPPER_OPENED_POS;
	cubeManipulators[0].slider.id = MANIP_LEFT_SERVO_SLIDER_ID;
	cubeManipulators[0].slider.botPos = MANIP_LEFT_SERVO_SLIDER_BOT_POS;
	cubeManipulators[0].slider.topPos = MANIP_LEFT_SERVO_SLIDER_TOP_POS;
	cubeManipulators[0].slider.intermPos = MANIP_LEFT_SERVO_SLIDER_INTERM_POS;
	cubeManipulators[0].tasksSequencePtr = takeCubetaskSeq;
	
	cubeManipulators[1].gripper.id = MANIP_CENTRAL_SERVO_GRIPPER_ID;
	cubeManipulators[1].gripper.closedAngle = MANIP_CENTRAL_SERVO_GRIPPER_CLOSED_POS;
	cubeManipulators[1].gripper.openedAngle = MANIP_CENTRAL_SERVO_GRIPPER_OPENED_POS;
	cubeManipulators[1].slider.id = MANIP_CENTRAL_SERVO_SLIDER_ID;
	cubeManipulators[1].slider.botPos = MANIP_CENTRAL_SERVO_SLIDER_BOT_POS;
	cubeManipulators[1].slider.topPos = MANIP_CENTRAL_SERVO_SLIDER_TOP_POS;
	cubeManipulators[1].slider.intermPos = MANIP_CENTRAL_SERVO_SLIDER_INTERM_POS;
	cubeManipulators[1].tasksSequencePtr = takeCubetaskSeq;
	
	cubeManipulators[2].gripper.id = MANIP_RIGHT_SERVO_GRIPPER_ID;
	cubeManipulators[2].gripper.closedAngle = MANIP_RIGHT_SERVO_GRIPPER_CLOSED_POS;
	cubeManipulators[2].gripper.openedAngle = MANIP_RIGHT_SERVO_GRIPPER_OPENED_POS;
	cubeManipulators[2].slider.id = MANIP_RIGHT_SERVO_SLIDER_ID;
	cubeManipulators[2].slider.botPos = MANIP_RIGHT_SERVO_SLIDER_BOT_POS;
	cubeManipulators[2].slider.topPos = MANIP_RIGHT_SERVO_SLIDER_TOP_POS;
	cubeManipulators[2].slider.intermPos = MANIP_RIGHT_SERVO_SLIDER_INTERM_POS;
	cubeManipulators[2].tasksSequencePtr = takeCubetaskSeq;
	return;
}

ErrorStatus setServoAngleWithRetries(const uint8_t servoId, const uint16_t servoAngle)
{
	uint8_t i;
	for (i = 0x00; i < TASKS_EXECUTOR_MAX_SEND_RETRIES; i++)
	{
		if (setServoAngle(servoId, servoAngle))
		{
			return SUCCESS;
		}
	}
	return ERROR;
}

ErrorStatus getServoAngleWithRetries(const uint8_t servoId, float* servoAngle)
{
	uint8_t i;
	for (i = 0x00; i < TASKS_EXECUTOR_MAX_SEND_RETRIES; i++)
	{
		if (getServoAngle(servoId, servoAngle))
		{
			return SUCCESS;
		}
	}
	return ERROR;
}

ErrorStatus getServoLoadWithRetries(const uint8_t servoId, uint16_t* servoLoad)
{
	uint8_t i;
	for (i = 0x00; i < TASKS_EXECUTOR_MAX_SEND_RETRIES; i++)
	{
		if (getCurrentLoad(servoId, servoLoad))
		{
			return SUCCESS;
		}
	}
	return ERROR;
}
//--------------------------------------------- servoChecker functions ---------------------------------------//
void checkPosServo(Servo_Checker_Typedef* servoChecker)
{
	if (servoChecker->statusFlag == SERVO_CHECKER_ACTIVE_MODE)
	{	
		// buffer for current angle and torgue
		float angle;
//		uint16_t torgue;
		
		// Increment number of time periods
		servoChecker->numberOfTimerPeriods++;

		// Read current angle of the servo
		if (!getServoAngleWithRetries(servoChecker->servoId, &angle))
		{
			servoChecker->statusFlag = SERVO_CHECKER_ERROR_MAXIMUM_RETRIES_EXCEEDED;
			return;
		}
		
//		// Read current load of the servo
//		if (!getServoLoadWithRetries(servoChecker->servoId, &torgue))
//		{
//			servoChecker->statusFlag = SERVO_CHECKER_ERROR_MAXIMUM_RETRIES_EXCEEDED;
//			return;
//		}

//		// Check if current load is abnormal
//		if (torgue >= SERVO_CHECKER_MAX_LOAD)
//		{
//			servoChecker->statusFlag = SERVO_CHECKER_ERROR_MAXIMUM_LOAD_EXCEEDED;
//			// Return servo to previous position
//			setServoAngleWithRetries(servoChecker->servoId, servoChecker->previousPos);
//			return;
//		}
		
		// Check if servo reached target position
		if (fabs(servoChecker->targetPos - angle) < SERVO_CHECKER_ANGLE_EPS)
		{
			servoChecker->statusFlag = SERVO_CHECKER_SUCCESFUL_CONFIRMATION;
			return;
		}
		else if (servoChecker->numberOfTimerPeriods >= SERVO_CHECKER_MAX_TIMEOUT)
		{
			servoChecker->statusFlag = SERVO_CHECKER_ERROR_WRONG_POSITION;
			return;
		}
	}
	return;
}

void resetChecker(Servo_Checker_Typedef* servoChecker)
{
	servoChecker->numberOfTimerPeriods = 0x00;
	servoChecker->servoId = 0x00;
	servoChecker->targetPos = 0.0;
	servoChecker->statusFlag = SERVO_CHECKER_WAITING_MODE;
	return;
}
//--------------------------------------------- Tasks executor's functions -------------------------------------//
void execManipTasks(uint8_t number, Cube_Manipulator_Typedef* manipulator)
{	
	// Check if task executor is active
	if (manipulator->subtasksExecutorStatusFlag == TASKS_EXECUTOR_ACTIVE_MODE)
	{
		Checker_Status_Typedef checkerStatusFlag = servoChecker[number].statusFlag;
		switch (checkerStatusFlag)
		{
			case SERVO_CHECKER_WAITING_MODE:
				// First command
				execManipSubtasks(number, manipulator);
				// Extend timer
				timEnable(SERVO_CHECKER_TIM_MODULE);
				break;
			case SERVO_CHECKER_ACTIVE_MODE:
				// Wait for servo checker
				// Extend timer
				timEnable(SERVO_CHECKER_TIM_MODULE);
				break;
			case SERVO_CHECKER_SUCCESFUL_CONFIRMATION:
				// Check if all subtasks were executed
				manipulator->tasksSequencePtr++;
				if(manipulator->tasksSequencePtr[0] == SUBTASK_TERMINATOR)
				{
					manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_SUCCESFUL_EXECUTION;
					// Reset servo checker
					resetChecker(&servoChecker[number]);
				}
				else
				{
					// Next subtask
					execManipSubtasks(number, manipulator);
				}
				break;
			case SERVO_CHECKER_ERROR_MAXIMUM_RETRIES_EXCEEDED:
				manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_MAX_RETRIES_EXCEEDED;
				// Reset servo checker
				resetChecker(&servoChecker[number]);
				break;
			case SERVO_CHECKER_ERROR_WRONG_POSITION:
				manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_WRONG_POSITION;
				// Reset servo checker
				resetChecker(&servoChecker[number]);
				break;
			case SERVO_CHECKER_ERROR_MAXIMUM_LOAD_EXCEEDED:
				manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_MAX_LOAD_EXCEEDED;
				// Reset servo checker
				resetChecker(&servoChecker[number]);
				break;
		}
	}
	return;
}
void execManipSubtasks(uint8_t number, Cube_Manipulator_Typedef* manipulator)
{
	// Extract current subtask
	Manipulator_Subtasks_Typedef subtask = *(manipulator->tasksSequencePtr);
	
	// Buffers for servo id, target position and current position
	uint16_t servoTargetPos;
	float servoCurrentPos;
	uint8_t servoId;
	
	// Unload values that corresponds to partivular subtask
	switch (subtask)
	{
		case SUBTASK_OPEN_MANIPULATOR:
			servoId  = manipulator->gripper.id;
			servoTargetPos = manipulator->gripper.openedAngle;
			break;
		case SUBTASK_CLOSE_MANIPULATOR:
			servoId  = manipulator->gripper.id;
			servoTargetPos = manipulator->gripper.closedAngle;
			break;
		case SUBTASK_LOWER_MANIPULATOR:
			servoId  = manipulator->slider.id;
			servoTargetPos = manipulator->slider.botPos;
			break;
		case SUBTASK_LIFT_MANIPULATOR:
			servoId  = manipulator->slider.id;
			servoTargetPos = manipulator->slider.topPos;
			break;
		case SUBTASK_LIFT_MANIPULATOR_INTERM:
			servoId  = manipulator->slider.id;
			servoTargetPos = manipulator->slider.intermPos;
			break;
		case SUBTASK_TERMINATOR:
			manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_TERMINATOR_REACHED;
			return;
	}
	// Read current angle of the servo
	if (!getServoAngleWithRetries(servoId, &servoCurrentPos))
	{
		manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_MAX_RETRIES_EXCEEDED;
		// Reset servo checker
		resetChecker(&servoChecker[number]);
		return;
	}
	// Send set angle command to servo
	if (!setServoAngleWithRetries(servoId, servoTargetPos))
	{
		manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_MAX_RETRIES_EXCEEDED;
		// Reset servo checker
		resetChecker(&servoChecker[number]);
		return;
	}
	// Reset servo checker
	resetChecker(&servoChecker[number]);
	// Load angle and id into servoChecker and turn it on
	servoChecker[number].servoId = servoId;
	servoChecker[number].targetPos = servoTargetPos;
	servoChecker[number].previousPos = servoCurrentPos;
	servoChecker[number].statusFlag = SERVO_CHECKER_ACTIVE_MODE;
	// Extend timer
	timEnable(SERVO_CHECKER_TIM_MODULE);
	return;
}

//--------------------------------------------- High-level command assignment ----------------------------------//
void setManipHighLevelCommand(Manipulator_Command_Typedef command, uint8_t number, Cube_Manipulator_Typedef* manipulator)
{
	if (number > NUMBER_OF_MANIPULATORS)
	{
		// Wrong manipulator's id
		return;
	}
	//Check if manipulator is still working
	if (manipulator->subtasksExecutorStatusFlag == TASKS_EXECUTOR_ACTIVE_MODE)
	{
		return;
	}
	// Set Command
	switch(command)
	{
		case TAKE_CUBE_COMMAND:
			manipulator->tasksSequencePtr = &takeCubetaskSeq[0];
			break;
		case TAKE_LAST_CUBE_COMMAND:
			manipulator->tasksSequencePtr = &takeLastCubetaskSeq[0];
			break;
		case UNLOAD_TOWER_COMMAND:
			manipulator->tasksSequencePtr = &unloadTowertaskSeq[0];
			break;
	}
	// Set flag for task sequence execution
	manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ACTIVE_MODE;
	// Turn timer on
	timEnable(SERVO_CHECKER_TIM_MODULE);
	return;
}
