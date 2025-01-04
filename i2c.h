/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */
void readBit(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
void readBits(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
HAL_StatusTypeDef readByte(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t *data);
HAL_StatusTypeDef readBytes(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t num, uint8_t *data);
HAL_StatusTypeDef readWords(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

bool writeBit(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
bool writeBits(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
bool writeByte(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t data);
bool writeBytes(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
bool writeWord(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint16_t data);
bool writeWords(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);
/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

