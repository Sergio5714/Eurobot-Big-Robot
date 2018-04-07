#ifndef COLLISION_AVOIDANCE
#define COLLISION_AVOIDANCE

#include "STM32F4_I2C.h"
#include "VL6180x.h"
#include "Interrupts.h"
//--------------------------------------------- Macros for proximity sensor ------------------------------------//
#define RANGE_FINDER_RESET_DELAY_MS                0x01
#define RANGE_FINDER_INITIAL_ADDR_TO_SETUP         RANGEFINDER_DEFAULT_ADDR + 0x01
#define RANGE_FINDER_NUMBER_OF_LAST_COL_AV_SENSOR  0x09 // Number of last sensor for collision avoidance

#define NUMBER_OF_RANGE_FINDERS                    0x10
#define NUMBER_OF_RANGE_FINDERS_FOR_CALIBR         0x06
//--------------------------------------------- Macros for expander --------------------------------------------//
#define EXPANDER_INTERRUPT_I2C_ADDRESS             0x20
#define EXPANDER_RESET_I2C_ADDRESS                 0x21

#define EXPANDER_REG_DIR_A                         0x00
#define EXPANDER_REG_DIR_B                         0x10
#define EXPANDER_REG_VALUE_A                       0x0A
#define EXPANDER_REG_VALUE_B                       0x1A
#define EXPANDER_REG_INTERRUPT_ON_A                0x02
#define EXPANDER_REG_INTERRUPT_ON_B                0x12
#define EXPANDER_REG_COMPARE_VALUE_FOR_INTER_A     0x03
#define EXPANDER_REG_COMPARE_VALUE_FOR_INTER_B     0x13
#define EXPANDER_REG_INTERRUPT_CONTROL_A           0x04
#define EXPANDER_REG_INTERRUPT_CONTROL_B           0x14
#define EXPANDER_CONFIG_REG_DEFAULT                0x0A
#define EXPANDER_CONFIG_REG                        0x05
//INTERRUPT CAPTURED VALUE FOR PORT REGISTER
#define EXPANDER_REG_INT_CAP_VAL_A                 0x08
#define EXPANDER_REG_INT_CAP_VAL_B                 0x18 

// Typedef for rangefinder's errors
typedef enum
{
	RANGE_FINDER_NO_ERROR,
	RANGE_FINDER_ERROR_WHILE_INIT,
	RANGE_FINDER_ERROR_WHILE_START_MEASUREMENTS,
	RANGE_FINDER_ERROR_WHILE_OPERATION,
} Range_Finder_Errors_Typedef;

// Typedef for expander's errors
typedef enum
{
	EXPANDER_NO_ERROR,
	EXPANDER_ERROR,
} Expander_Errors_Typedef;

typedef enum
{
	RANGE_FINDER_NO_NEED_TO_REINIT,
	RANGE_FINDER_REINIT_IS_NECESSARY,
} Range_Finders_Reinit_flag_Typedef;

// Typedef for common struc for all rangefinders that contain the whole information
typedef struct
{
	uint8_t rangeValues[NUMBER_OF_RANGE_FINDERS];
	uint8_t dataForCalibration[NUMBER_OF_RANGE_FINDERS_FOR_CALIBR];
	Range_Finder_Errors_Typedef errorFlags[NUMBER_OF_RANGE_FINDERS];
	Range_Finders_Reinit_flag_Typedef reinitFlags[NUMBER_OF_RANGE_FINDERS];
	Expander_Errors_Typedef outputExpander;
	Expander_Errors_Typedef interruptExpander;
	uint16_t outputVoltageOfExpander;
	uint8_t globalReinitFlag;
} Range_Finders_Struct_Typedef;

//--------------------------------------------- High level functions -------------------------------------------//

// Global initialization
ErrorStatus initRangeFindersGlobally(void);

// Read all values of rangefinders
ErrorStatus readRangesGlobally(void);

// Check if rangefinders should be reinitialized or not
void checkRangeFindersReinitFlags(void);

// Make postprocessing of data from rangefinders for calibration
void postprocessDataForCalibration(void);

// Reset particular rangefinder
ErrorStatus resetRangeFinder(uint8_t numberOfSensor);

// Init particular rangefinder
ErrorStatus initRangeFinder(uint8_t numberOfSensor);

//--------------------------------------------- Middle level functions -----------------------------------------//

// Initialize expander in output mode
ErrorStatus initExpanderOutputMode(uint8_t expanderAddr);

// Initialize expander in input (interrupt) mode
ErrorStatus initExpanderInterruptMode(uint8_t expanderAddr);

// Read interrupt of expander
ErrorStatus expanderReadInterrupt(uint8_t expanderAddr, uint16_t* interruptStatus);

// Set voltage on expander's pins
ErrorStatus setExpanderVoltage(uint16_t voltage, uint8_t expanderAddr);

//--------------------------------------------- Low level functions to access registers ------------------------//

// Write an 8-bit value to expander's register
ErrorStatus expanderWriteReg(uint8_t reg, uint8_t value, uint8_t addr);

// Read a 8-bit value from  register
ErrorStatus expanderReadReg(uint8_t reg, uint8_t* value, uint8_t addr);

#endif
