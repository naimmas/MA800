/**
 * @file ma800.h
 * @author Naim MASRI (naimmas@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "msp430.h"
#include "../common.h"
#include "spi.h"
#include "../delay.h"
#define READ_CMD 0x2
#define WRITE_CMD 0x4
#define ZERO_SET_REG_LOW 0x0
#define ZERO_SET_REG_HIGH 0x1
#define MAG_THRESHOLD_SET 0x6
#define ROT_DIR_GET 0x9
#define MAG_SW_GET 0X1B
#define GET_REG(register_address)       (uint16_t) (READ_CMD<<13 | register_address<<8)
#define SET_REG(register_address, data) (uint16_t) (READ_CMD<<13 | register_address<<8 | (uint8_t)data)

void MA800_Init(chipSelect_t* chipselect_pin, SpiClkSource_t clk_src, uint16_t clk_prescaler);
double MA800_GetAngle();
uint8_t MA800_GetRawAngle();
OperationStatus_t MA800_SetZeroAngle(float angle);
double MA800_GetZeroAngle();
OperationStatus_t MA800_SetSwtichMagneticThreshold(uint8_t low_threshold, uint8_t high_threshold);
void MA800_GetSwtichMagneticThreshold(uint8_t* low_threshold_buffer, uint8_t* high_threshold_buffer);
bool MA800_GetRotationDirection();
uint8_t MA00_GetMagneticSwitchState();
