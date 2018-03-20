 #include "VL6180x.h"
 #include "STM32F4_I2C.h"
 #include "math.h"
 
extern I2C_Module_With_State_Typedef I2CModule;
extern uint32_t timeMilliseconds;

//--------------------------------------------- High level functions -------------------------------------------//
// Initialization
ErrorStatus rangeFinderInitContiniousInterruptMode(uint8_t addr)
{
  // Store part-to-part range offset so it can be adjusted if scaling is changed
  //  ptp_offset = readReg(SYSRANGE__PART_TO_PART_RANGE_OFFSET);
 	uint8_t buf;
	if (rangeFinderReadReg(SYSTEM_FRESH_OUT_OF_RESET, &buf, addr) != SUCCESS)
	{
	  return ERROR;
	}
	if (buf == 0x01)
	{
		// Mandatory private registers
		if (initMandatoryRegs(addr) != SUCCESS)
		{
			return ERROR;
		}

		// Recommended settings
		
//		// Set averaging sample period (tradeoff between lower noise and execution time)
//		rangeFinderWriteReg(0x10A, 0x30, addr);

		// The light and dark gain
		if (rangeFinderWriteReg(0x03F, 0x46, addr) != SUCCESS)
		{
			return ERROR;
		}
		
		// Sets the # of range measurements after which auto calibtration of system is performed
		if (rangeFinderWriteReg(0x031, 0xFF, addr) != SUCCESS)
		{
			return ERROR;
		}
		
		// Performs  a single temperature calibration of  the ranging sensor
		rangeFinderWriteReg(0x02E, 0x01, addr);
		
		// Optional
		// Sets default ranging inter measurement period to 20 ms
		if (rangeFinderWriteReg(SYSRANGE_INTERMEASUREMENT_PERIOD, 0x01, addr) != SUCCESS)
		{
			return ERROR;
		}
		
		// Sets max converge time (13 ms)
		if (rangeFinderWriteReg(SYSRANGE_MAX_CONVERGENCE_TIME, 0x0D, addr) != SUCCESS)
		{
			return ERROR;
		}

		// Setup GPIO1
		if (rangeFinderWriteReg(SYSTEM_MODE_GPIO1, 0x30, addr) != SUCCESS)
		{
			return ERROR;
		}
		
		// Indicate that data will be updated
		if (rangeFinderWriteReg(SYSTEM_GROUPED_PARAMETER_HOLD, 0x01, addr) != SUCCESS)
		{
			return ERROR;
		}
		
//		// Configure interrupt on 'Level Low (value < thresh_low)' and 'New sample ready' for range
//		if (rangeFinderWriteReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x01, addr) != SUCCESS)
//		{
//			return ERROR;
//		}
		
		// Configure interrupt on Out Of Window (value < thresh_low OR value > thresh_high)
		if (rangeFinderWriteReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x03, addr) != SUCCESS)
		{
			return ERROR;
		}
		
		// Configure level low value  for range (50 mm)
		if (rangeFinderWriteReg(SYSRANGE_THRESH_LOW, LEVEL_LOW_RANGE_INTERRUPT_VALUE, addr) != SUCCESS)
		{
			return ERROR;
		}
		
		// Configure level high value  for range (100 mm)
		if (rangeFinderWriteReg(SYSRANGE_THRESH_HIGH, LEVEL_HIGH_RANGE_INTERRUPT_VALUE, addr) != SUCCESS)
		{
			return ERROR;
		}
		
		// Indicate that data was updated
		if (rangeFinderWriteReg(SYSTEM_GROUPED_PARAMETER_HOLD, 0x00, addr) != SUCCESS)
		{
			return ERROR;
		}
		
		// Max SNR 0.5 % 
		if (rangeFinderWriteReg(SYSRANGE_MAX_AMBIENT_LEVEL_MULT, 0x08, addr) != SUCCESS)
		{
			return ERROR;
		}
		
		// Finish setup
		if (rangeFinderWriteReg(SYSTEM_FRESH_OUT_OF_RESET, 0x00, addr) != SUCCESS)
		{
			return ERROR;
		}
		return SUCCESS;
  }
  else
  {
    // Sensor has already been initialized
  }
	return SUCCESS;
}

// Return Status
ErrorStatus getStatusOfSensor(uint8_t addr, uint8_t* value)
{
	if(rangeFinderReadReg(RESULT_RANGE_STATUS, value, addr)!= SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

// Single shot measurements
ErrorStatus singleShotMeasurements(uint8_t addr, uint8_t* value)
{
	uint32_t startTime = timeMilliseconds;
	//uint8_t status;

	// Start a range measurements
	if(rangeFinderWriteReg(SYSRANGE_START, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	uint8_t buf = 0x00;
	// Wait for range measurement to be completed
	while(!(buf & BIT2_MASK))
	{
		if(rangeFinderReadReg(RESULT_INTERRUPT_STATUS_GPIO, &buf, addr)!= SUCCESS || ((timeMilliseconds - startTime) > MEASUREMENT_TIMEOUT))
		{
			return ERROR;
		}
	}
	// Read value of range
	if(readMeasuredRange(addr,value) != SUCCESS)
	{
		return ERROR;
	};
	return SUCCESS;
}



// Start continious measurements
ErrorStatus startContiniousMeasurements(uint8_t addr)
{
	// Start a continious range measurements
	if(rangeFinderWriteReg(SYSRANGE_START, 0x03, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

// Read value after measurement
ErrorStatus readMeasuredRange(uint8_t addr, uint8_t* value)
{
	uint8_t status;
	uint8_t buf;
	// Read status register
	if (getStatusOfSensor(addr, &status) != SUCCESS)
	{
		return ERROR;
	}
	// Extracting bits from 4 to 7
	status = (status & 0xF0) >> 4;
	
	// Check is there was an error or not
//	if(status)
//	{
//		return ERROR;
//	}
	
	if(rangeFinderReadReg(RESULT_INTERRUPT_STATUS_GPIO, &buf, addr)!= SUCCESS)
	{
		return ERROR;
	}
	if (!(buf & 0x03))
	{
		if(rangeFinderWriteReg(SYSTEM_INTERRUPT_CLEAR, 0x07, addr) != SUCCESS)
		{
			return ERROR;
		}
		return ERROR;
	}
	// Read value of range
 	if(rangeFinderReadReg(RESULT_RANGE_VAL, value, addr) != SUCCESS)
	{
		return ERROR;
	}
	if(rangeFinderWriteReg(SYSTEM_INTERRUPT_CLEAR, 0x07, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

// Read raw value (without crosstalk calibration)
ErrorStatus readMeasuredRangeRaw(uint8_t addr, uint8_t* value)
{
	uint8_t status;
	// Read status register
	if (getStatusOfSensor(addr, &status) != SUCCESS)
	{
		return ERROR;
	}
	// Extracting bits from 4 to 7
	status = (status & 0xF0) >> 4;
//	// Check is there was an error or not
//	if(status)
//	{
//		return ERROR;
//	}
	// Read value of range
	if(rangeFinderReadReg(RESULT_RANGE_RAW, value, addr) != SUCCESS)
	{
		return ERROR;
	}
	if(rangeFinderWriteReg(SYSTEM_INTERRUPT_CLEAR, 0x07, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

// Initialize mandatory registers
ErrorStatus initMandatoryRegs(uint8_t addr)
{
	// Mandatory private registers
	if (rangeFinderWriteReg(0x207, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x208, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x096, 0x00, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x097, 0xFD, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0E3, 0x00, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0E4, 0x04, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0E5, 0x02, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0E6, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0E7, 0x03, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0F5, 0x02, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0D9, 0x05, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0DB, 0xCE, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0DC, 0x03, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0DD, 0xF8, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x09F, 0x00, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0A3, 0x3C, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0B7, 0x00, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0BB, 0x3C, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0B2, 0x09, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0CA, 0x09, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x198, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x1B0, 0x17, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x1AD, 0x00, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0FF, 0x05, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x100, 0x05, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x199, 0x05, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x1A6, 0x1B, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x1AC, 0x3E, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x030, 0x00, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

//--------------------------------------------- Low level functions to access registers ------------------------//

 // Write an 8-bit value to register
ErrorStatus rangeFinderWriteReg(uint16_t reg, uint8_t value, uint8_t addr)
{
	uint8_t buf[3];
	buf[0] = (reg >> 8) & MASK_8_BITS;
	buf[1] = reg & MASK_8_BITS;
	buf[2] = value;
	
	if (I2CSendBytes(&I2CModule, buf, 0x03, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}
 
 // Write a 16-bit value to register
ErrorStatus rangeFinderWriteReg16(uint16_t reg, uint16_t value, uint8_t addr)
{
	uint8_t buf[4];
	buf[0] = (reg >> 8) & MASK_8_BITS;
	buf[1] = reg & MASK_8_BITS;
	buf[2] = (value >> 8) & MASK_8_BITS;
	buf[3] = value & MASK_8_BITS;
	
	if (I2CSendBytes(&I2CModule, buf, 0x04, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}
 
 // Write a 32-bit value to register
ErrorStatus rangeFinderWriteReg32(uint16_t reg, uint32_t value, uint8_t addr)
{
	uint8_t buf[6];
	buf[0] = (reg >> 8) & MASK_8_BITS;
	buf[1] = reg & MASK_8_BITS;
	buf[2] = (value >> 24) & MASK_8_BITS;
	buf[3] = (value >> 16) & MASK_8_BITS;
	buf[4] = (value >> 8) & MASK_8_BITS;
	buf[5] = value & MASK_8_BITS;
	
	if (I2CSendBytes(&I2CModule, buf, 0x06, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}
 
 // Read a 8-bit value from  register
ErrorStatus rangeFinderReadReg(uint16_t reg, uint8_t* value, uint8_t addr)
{
	uint8_t bufReg [2];
	bufReg[0] = (reg >> 8) & MASK_8_BITS;
	bufReg[1] = reg & MASK_8_BITS;
	if (I2CSendBytes(&I2CModule, bufReg, 0x00, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (I2CSendBytes(&I2CModule, bufReg, 0x02, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (I2CReadBytes(&I2CModule, value, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}
 
 // Read a 16-bit value from  register
ErrorStatus rangeFinderReadReg16(uint16_t reg, uint16_t* value, uint8_t addr)
{
	uint8_t bufReg [2];
	bufReg[0] = (reg >> 8) & MASK_8_BITS;
	bufReg[1] = reg & MASK_8_BITS;
	
	uint8_t bufValue [2];
	
	if (I2CSendBytes(&I2CModule, bufReg, 0x02, addr) != SUCCESS)
	{
		return ERROR;
	}
	
	if (I2CReadBytes(&I2CModule, bufValue, 0x02, addr) != SUCCESS)
	{
		return ERROR;
	}
	
	*value = (bufValue[0] << 8) + bufReg[1];
	return SUCCESS;
}
 
 // Read a 32-bit value from  register
ErrorStatus rangeFinderReadReg32(uint16_t reg, uint32_t* value, uint8_t addr)
{
	uint8_t bufReg [2];
	bufReg[0] = (reg >> 8) & MASK_8_BITS;
	bufReg[1] = reg & MASK_8_BITS;
	
	uint8_t bufValue [4];
	
	if (I2CSendBytes(&I2CModule, bufReg, 0x02, addr) != SUCCESS)
	{
		return ERROR;
	}
	
	if (I2CReadBytes(&I2CModule, bufValue, 0x04, addr) != SUCCESS)
	{
		return ERROR;
	}
	
	*value = (bufValue[0] << 24) + (bufValue[1] << 16) + (bufValue[2] << 8) + bufValue[3];
	return SUCCESS;
}
