/*
* max30102.h
*
* Created on: Jul 31, 2025
* Author:
*
* Basic driver for the MAX30102 Pulse Oximeter and Heart-Rate Sensor.
* Target: STM32F405xx using HAL library.
*/

#ifndef INC_MAX30102_H_
#define INC_MAX30102_H_

#include "stm32f4xx_hal.h"

// The 7-bit I2C address of the MAX30102
#define MAX30102_I2C_ADDR 0x57

// The I2C address is shifted left by 1 for HAL functions
#define MAX30102_I2C_ADDR_WR (MAX30102_I2C_ADDR << 1)

// Register Addresses
#define REG_INTR_STATUS_1   0x00
#define REG_INTR_STATUS_2   0x01
#define REG_INTR_ENABLE_1   0x02
#define REG_INTR_ENABLE_2   0x03
#define REG_FIFO_WR_PTR     0x04
#define REG_OVF_COUNTER     0x05
#define REG_FIFO_RD_PTR     0x06
#define REG_FIFO_DATA       0x07
#define REG_FIFO_CONFIG     0x08
#define REG_MODE_CONFIG     0x09
#define REG_SPO2_CONFIG     0x0A
#define REG_LED1_PA         0x0C // IR LED
#define REG_LED2_PA         0x0D // Red LED
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_DIE_TEMP_INTR   0x1F
#define REG_DIE_TEMP_FRAC   0x20
#define REG_DIE_TEMP_CONFIG 0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID          0xFE
#define REG_PART_ID         0xFF // Should be 0x15

#define BUFFER_SIZE 500 // Example: 5 seconds of data at 100 Hz
extern uint32_t ir_buffer[BUFFER_SIZE];
extern uint32_t red_buffer[BUFFER_SIZE];
extern volatile int32_t buffer_index; // Use int32_t for safety with negative indices during wrap-around

//void add_sample_to_buffer(uint32_t red_val, uint32_t ir_val);

/**
* @brief Initializes the MAX30102 sensor.
* @param hi2c Pointer to a I2C_HandleTypeDef structure that contains
* the configuration information for the specified I2C.
* @retval HAL_StatusTypeDef: HAL_OK if successful, HAL_ERROR otherwise.
*/
HAL_StatusTypeDef max30102_init(I2C_HandleTypeDef *hi2c);

/**
* @brief Reads a single sample (IR and Red values) from the FIFO.
* @param hi2c Pointer to a I2C_HandleTypeDef structure.
* @param red_val Pointer to a uint32_t to store the Red LED value.
* @param ir_val Pointer to a uint32_t to store the IR LED value.
* @retval HAL_StatusTypeDef: HAL_OK if successful, HAL_ERROR otherwise.
*/
HAL_StatusTypeDef max30102_read_fifo(I2C_HandleTypeDef *hi2c, uint32_t *red_val, uint32_t *ir_val);


/**
* @brief Performs a soft reset of the sensor.
* @param hi2c Pointer to a I2C_HandleTypeDef structure.
* @retval HAL_StatusTypeDef: HAL_OK if successful, HAL_ERROR otherwise.
*/
HAL_StatusTypeDef max30102_reset(I2C_HandleTypeDef *hi2c);

#endif /* INC_MAX30102_H_ */

