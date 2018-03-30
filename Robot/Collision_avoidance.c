#include "Collision_avoidance.h"
extern I2C_Module_With_State_Typedef I2CModule;

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
	// Init logic how 
	if (expanderWriteReg(EXPANDER_REG_INTERRUPT_CONTROL_A, 0xff, expanderAddr) != SUCCESS)
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
	if (expanderWriteReg(EXPANDER_REG_INTERRUPT_ON_A, 0xFF, expanderAddr) != SUCCESS)
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
	if (expanderReadReg(EXPANDER_REG_INT_CAP_VAL_A, &interruptStatusA, EXPANDER_I2C_ADDRESS) != SUCCESS)
	{
		return ERROR;
	}
	// Interrupt captured value for port B
	if (expanderReadReg(EXPANDER_REG_INT_CAP_VAL_B, &interruptStatusB, EXPANDER_I2C_ADDRESS) != SUCCESS)
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
