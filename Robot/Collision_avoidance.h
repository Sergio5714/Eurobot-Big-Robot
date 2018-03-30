#ifndef COLLISION_AVOIDANCE
#define COLLISION_AVOIDANCE

#include "STM32F4_I2C.h"
#include "VL6180x.h"

#define EXPANDER_I2C_ADDRESS                       0x20
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

//--------------------------------------------- High level functions -------------------------------------------//

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
