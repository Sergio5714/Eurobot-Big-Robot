#include "Collision_avoidance.h"
extern I2C_Module_With_State_Typedef I2CModule;

Range_Finders_Struct_Typedef rangeFinders;
//--------------------------------------------- High level functions -------------------------------------------//
// Global initialization
ErrorStatus initRangeFindersGlobally(void)
{
	if (initExpanderOutputMode(EXPANDER_RESET_I2C_ADDRESS) != SUCCESS)
	{
		// Clear low level error flag
		I2CModule.status = I2C_ACTIVE_MODE;
		return ERROR;
	}
	if (initExpanderInterruptMode(EXPANDER_INTERRUPT_I2C_ADDRESS) != SUCCESS)
	{
		// Clear low level error flag
		I2CModule.status = I2C_ACTIVE_MODE;
		return ERROR;
	}
	// Initialize rangefinders
	uint8_t i;
	for(i = 0x00; i < NUMBER_OF_RANGE_FINDERS; i++)
	{
		if (initRangeFinder(i) != SUCCESS)
		{
			// Some error arised
			rangeFinders.errorFlags[i] = RANGE_FINDER_ERROR_WHILE_INIT;
			// Error was recoreded into high level flag, so clear low level
			I2CModule.status = I2C_ACTIVE_MODE;
		}
	}
	for(i = 0x00; i < NUMBER_OF_RANGE_FINDERS; i++)
	{
		if (rangeFinderStartContiniousMeasurements(RANGE_FINDER_INITIAL_ADDR_TO_SETUP + i) != SUCCESS)
		{  
			// Some error arised
			rangeFinders.errorFlags[i] = RANGE_FINDER_ERROR_WHILE_START_MEASUREMENTS;
			// Error was recoreded into high level flag, so clear low level
			I2CModule.status = I2C_ACTIVE_MODE;
		}
	}

	return SUCCESS;
}

// Read all values of rangefinders
ErrorStatus readRangesGlobally(void)
{
	uint8_t i;
	uint8_t interruptStatus[NUMBER_OF_RANGE_FINDERS];
	
	// Check timeout for last I2C bus restart
	if (checkTimeout(I2CModule.timeOfLastI2CResetMillis, I2C_TIMEOUT_AFTER_RESET))
	{
		for(i = 0x00; i < NUMBER_OF_RANGE_FINDERS; i++)
		{
			// Get status of interrupt of particular rangefinder
			if (rangeFinderCheckInterruptStatusOfSensor(RANGE_FINDER_INITIAL_ADDR_TO_SETUP + i, &interruptStatus[i], RANGEFINDER_INTERRUPT_LEVEL_LOW) != SUCCESS)
			{
				rangeFinders.errorFlags[i] = RANGE_FINDER_ERROR_WHILE_OPERATION;
				rangeFinders.reinitFlags[i] = RANGE_FINDER_REINIT_IS_NECESSARY;
				// Error was recoreded into high level flag, so clear low level
				I2CModule.status = I2C_ACTIVE_MODE;
			}
			else
			{
				// Status was received. No errors occured
				rangeFinders.errorFlags[i] = RANGE_FINDER_NO_ERROR;
			}
			// If measurement is ready
			if (interruptStatus[i])
			{
				// Read measurements
				if(rangeFinderReadMeasuredRange(RANGE_FINDER_INITIAL_ADDR_TO_SETUP +i, &rangeFinders.rangeValues[i]) != SUCCESS)
				{
					rangeFinders.errorFlags[i] = RANGE_FINDER_ERROR_WHILE_OPERATION;
					rangeFinders.reinitFlags[i] = RANGE_FINDER_REINIT_IS_NECESSARY;
					// Error was recoreded into high level flag, so clear low level
					I2CModule.status = I2C_ACTIVE_MODE;
				}
				else
				{
					// Measurement was received. No errors occured
					rangeFinders.errorFlags[i] = RANGE_FINDER_NO_ERROR;
				}
			}
		}
	}
	return SUCCESS;
}

// Check if rangefinders should be reinitialized or not
void checkRangeFindersReinitFlags(void)
{
	uint8_t i;
	// Check a situation when even expander should be reinitialized
	if (rangeFinders.gloabalReinitFlag)
	{
		initRangeFindersGlobally();
		// No need for global reinit
		rangeFinders.gloabalReinitFlag = 0x00;
		for(i = 0x00; i < NUMBER_OF_RANGE_FINDERS; i++)
		{
			rangeFinders.reinitFlags[i] = RANGE_FINDER_NO_NEED_TO_REINIT;
		}
		return;
	}
	else
	{
		uint8_t sum = 0x00;
		// Check if all rangefinders should be reinitialized at the same time
		for(i = 0x00; i < NUMBER_OF_RANGE_FINDERS; i++)
		{
			sum += rangeFinders.reinitFlags[i];
		}
		// If all rangefinders should be reinitialized then there is an error with bus
		if (sum == NUMBER_OF_RANGE_FINDERS)
		{
			// Reset I2C bus
			I2CReset(&I2CModule);
			rangeFinders.gloabalReinitFlag = 0x01;
			return;
		}
		
		// Initialize "silent" rangefinders
		for(i = 0x00; i < NUMBER_OF_RANGE_FINDERS; i++)
		{
			if (rangeFinders.reinitFlags[i])
			{
				if (initRangeFinder(i) != SUCCESS)
				{
					// Clear low level error flag
					I2CModule.status = I2C_ACTIVE_MODE;
					return;
				}
				if(rangeFinderStartContiniousMeasurements(RANGE_FINDER_INITIAL_ADDR_TO_SETUP + i) != SUCCESS)
				{
					// Clear low level error flag
					I2CModule.status = I2C_ACTIVE_MODE;
				}
				// Successful initialization, clear reinit flag
				rangeFinders.reinitFlags[i] = RANGE_FINDER_NO_NEED_TO_REINIT;
			}
		}
		
	}
	return;
}

// Reset particular rangefinder
ErrorStatus resetRangeFinder(uint8_t numberOfSensor)
{

	rangeFinders.outputVoltageOfExpander &= ~(1 << numberOfSensor);
	if (setExpanderVoltage(rangeFinders.outputVoltageOfExpander, EXPANDER_RESET_I2C_ADDRESS) != SUCCESS)
	{
		return ERROR;
	}
	delayMs(RANGE_FINDER_RESET_DELAY_MS);
	rangeFinders.outputVoltageOfExpander |= (1 << numberOfSensor);
	if (setExpanderVoltage(rangeFinders.outputVoltageOfExpander, EXPANDER_RESET_I2C_ADDRESS) != SUCCESS)
	{
		return ERROR;
	}
	delayMs(RANGE_FINDER_RESET_DELAY_MS);
	return SUCCESS;
}
// Init particular rangefinder
ErrorStatus initRangeFinder(uint8_t numberOfSensor)
{
	// Reset sensor
	if (resetRangeFinder(numberOfSensor) != SUCCESS)
	{
		return ERROR;
	}
	// Initialization
	if(rangeFinderInitContiniousInterruptLevelLowMode(RANGEFINDER_DEFAULT_ADDR, LEVEL_LOW_RANGE_INTERRUPT_VALUE)  != SUCCESS)
	{
		return ERROR;
	}
	// Change address
	uint8_t newAddr = RANGE_FINDER_INITIAL_ADDR_TO_SETUP + numberOfSensor;
	if(rangeFinderChangeAddress(RANGEFINDER_DEFAULT_ADDR, newAddr)  != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}
//--------------------------------------------- Middle level functions -----------------------------------------//
// Initialize expander in output mode
ErrorStatus initExpanderOutputMode(uint8_t expanderAddr)
{
	// Setup config register for right mapping of ports
	if (expanderWriteReg(EXPANDER_CONFIG_REG_DEFAULT, 0xA2, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Init direction of A bank (0 - output, 1 - input) A - output
	if (expanderWriteReg(EXPANDER_REG_DIR_A, 0x00, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Init direction of B bank (0 - output, 1 - input) B - output
	if (expanderWriteReg(EXPANDER_REG_DIR_B, 0x00, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Output value of A bank (initial voltages - low)
	if (expanderWriteReg(EXPANDER_REG_VALUE_A, 0x00, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Output value of B bank (initial voltages - low)
	if (expanderWriteReg(EXPANDER_REG_VALUE_B, 0x00, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

// Initialize expander in input (interrupt) mode
ErrorStatus initExpanderInterruptMode(uint8_t expanderAddr)
{
	// Setup config register for right mapping of ports
	if (expanderWriteReg(EXPANDER_CONFIG_REG_DEFAULT, 0xA2, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Init direction of A bank (0 - output, 1 - input) A - input
	if (expanderWriteReg(EXPANDER_REG_DIR_A, 0xFF, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Init direction of B bank (0 - output, 1 - input) B - input
	if (expanderWriteReg(EXPANDER_REG_DIR_B, 0xFF, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Set value to compare for A bank
	if (expanderWriteReg(EXPANDER_REG_COMPARE_VALUE_FOR_INTER_A, 0x00, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Set value to compare for B bank
	if (expanderWriteReg(EXPANDER_REG_COMPARE_VALUE_FOR_INTER_B, 0x00, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Init interrupt mode that describes how the associated pin value 
	// is compared for the interrupt-on-change feature (it is compared against REG_COMPARE_VALUE_FOR_INTER_A)
	if (expanderWriteReg(EXPANDER_REG_INTERRUPT_CONTROL_A, 0xff, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Init interrupt mode that describes how the associated pin value 
	// is compared for the interrupt-on-change feature (it is compared against REG_COMPARE_VALUE_FOR_INTER_B)
	if (expanderWriteReg(EXPANDER_REG_INTERRUPT_CONTROL_B, 0xff, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Enable interrupt on change (all A pins)
	if (expanderWriteReg(EXPANDER_REG_INTERRUPT_ON_A, 0xFF, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Enable interrupt on change (all B pins)
	if (expanderWriteReg(EXPANDER_REG_INTERRUPT_ON_B, 0xFF, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	uint16_t buf;
	// Read interrupt 
	if (expanderReadInterrupt(expanderAddr, &buf) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

// Read interrupt of expander
ErrorStatus expanderReadInterrupt(uint8_t expanderAddr, uint16_t* interruptStatus)
{
	uint8_t interruptStatusA;
	uint8_t interruptStatusB;
	
	// Interrupt captured value for port A
	if (expanderReadReg(EXPANDER_REG_INT_CAP_VAL_A, &interruptStatusA, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Interrupt captured value for port B
	if (expanderReadReg(EXPANDER_REG_INT_CAP_VAL_B, &interruptStatusB, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	*interruptStatus = (interruptStatusB << 8) + interruptStatusA;
	return SUCCESS;
}

// Set voltage on expander's pins
ErrorStatus setExpanderVoltage(uint16_t voltage, uint8_t expanderAddr)
{
	uint8_t voltageA = (uint8_t)(voltage & 0xFF);
	uint8_t voltageB = (uint8_t)(voltage >> 8);
	// Output value of A bank
	if (expanderWriteReg(EXPANDER_REG_VALUE_A, voltageA, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Output value of B bank
	if (expanderWriteReg(EXPANDER_REG_VALUE_B, voltageB, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

//--------------------------------------------- Low level functions to access registers ------------------------//

// Write an 8-bit value to expander's register
ErrorStatus expanderWriteReg(uint8_t reg, uint8_t value, uint8_t addr)
{
	uint8_t buf[2];
	buf[0] = reg;
	buf[1] = value;
	
	if (I2CSendBytes(&I2CModule, buf, 0x02, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

// Read a 8-bit value from  register
ErrorStatus expanderReadReg(uint8_t reg, uint8_t* value, uint8_t addr)
{
	if (I2CSendBytes(&I2CModule, &reg, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (I2CReadBytes(&I2CModule, value, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}
