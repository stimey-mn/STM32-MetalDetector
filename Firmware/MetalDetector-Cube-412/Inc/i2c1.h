/*
 * i2c1.h
 *
 *  Created on: Jun 16, 2019
 *      Author: stimey
 */

#ifndef __I2C1_H__
#define __I2C1_H__

#include <stdint.h>
#include <stdbool.h>

#undef I2C1_DEBUG_MSGS
#undef I2C1_DUMP_REGS_ENABLED

// #define I2C_USE_400KHZ

// uncomment the following line to use GPIO bit bang IO for I2C traffic
// #define I2C_USE_BITBANG

bool I2C1_InitMaster(void);
bool I2C1_ShutdownMaster(void);

bool I2C1_WriteMaster(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize, uint8_t *buffer, uint8_t buff_len);
bool I2C1_ReadMaster(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize, uint8_t *buffer, uint8_t buff_len);

bool I2C1_WriteMasterDMA(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize, uint8_t *buffer, uint8_t buff_len);
bool I2C1_ReadMasterDMA(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize, uint8_t *buffer, uint8_t buff_len);
bool I2C1_DMABusy(void);

#endif /* I2C1_H_ */
