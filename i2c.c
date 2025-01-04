/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void readBit(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data){
	uint8_t b;
	readByte(hi2c, devAddr, regAddr, &b);
	*data = b & (1 << bitNum);
}

void readBits(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data){
	// 01101001 read byte
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	//    010   masked
	//   -> 010 shifted
	uint8_t b;

	if (readByte(hi2c, devAddr, regAddr, &b) == HAL_OK) {
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		b &= mask;
		b >>= (bitStart - length + 1);
		*data = b;
	}
}

HAL_StatusTypeDef readByte(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t *data){

	HAL_I2C_Master_Transmit(hi2c, (uint16_t) devAddr, &regAddr, 1, 100);
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(hi2c, (uint16_t) devAddr, data, 1, 100);
	return status;
}

HAL_StatusTypeDef readBytes(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t num, uint8_t *data){

	HAL_I2C_Master_Transmit(hi2c, (uint16_t) devAddr, &regAddr, 1, 100);
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(hi2c, (uint16_t) devAddr, data, num, 100);
	return status;
}

HAL_StatusTypeDef readWords(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data){

	uint8_t buff[2 * length];
	HAL_I2C_Master_Transmit(hi2c, (uint16_t) devAddr, &regAddr, 1, 100);
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(hi2c, (uint16_t) devAddr, buff, 2*length, 200);
	if(status == HAL_OK){
		for(uint8_t i = 0; i < length; i++){
			data[i] = (((uint16_t)buff[2*i]) << 8) | buff[2*i + 1];
		}
	}
	return status;
}

bool writeBit(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data){
	uint8_t b;
	readByte(hi2c, devAddr, regAddr, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	return writeByte(hi2c, devAddr, regAddr, b);
}

bool writeBits(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data){
	//      010 value to write
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	uint8_t b;
	if (readByte(hi2c, devAddr, regAddr, &b) == HAL_OK) {
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		data <<= (bitStart - length + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		b &= ~(mask); // zero all important bits in existing byte
		b |= data; // combine data with existing byte
		return writeByte(hi2c, devAddr, regAddr, b);
	} else {
		return false;
	}
}

bool writeByte(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t data){
	uint8_t datos[2] = {regAddr, data};
	if (HAL_I2C_Master_Transmit(hi2c, (uint16_t) devAddr, datos, 2, 100) == HAL_OK){
		return true;
	}
	else{
		return false;
	}
}

bool writeBytes(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data){
	uint8_t datos[length + 1];
	datos[0] = regAddr;
	for(uint8_t i = 0; i < length; i++){
		datos[i + 1] = *(data + i);
	}
	if (HAL_I2C_Master_Transmit(hi2c, (uint16_t) devAddr, datos, length + 1, 200) == HAL_OK){
		return true;
	}
	else{
		return false;
	}
}


bool writeWord(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint16_t data){
	uint8_t datos[3] = {regAddr, (uint8_t) data >>8, (uint8_t) data}; // RA, MSB, LSB
	if (HAL_I2C_Master_Transmit(hi2c, (uint16_t) devAddr, datos, 3, 100) == HAL_OK){
		return true;
	}
	else{
		return false;
	}
}

bool writeWords(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data){
	uint8_t datos[1 + 2*length];
	datos[0] = regAddr;
	for(uint8_t i = 0; i < length; i++){
		datos[2*i + 1] = (uint8_t)(data[i] >> 8); // MSB
		datos[2*(i + 1)] = (uint8_t)data[i]; // LSB
	}
	if (HAL_I2C_Master_Transmit(hi2c, (uint16_t) devAddr, datos, 1 + length, 200) == HAL_OK){
		return true;
	}
	else{
		return false;
	}
}

/* USER CODE END 1 */
