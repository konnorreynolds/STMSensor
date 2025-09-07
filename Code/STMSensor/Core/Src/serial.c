/**
  ******************************************************************************
  * @file           : serial.c
  * @brief          : Serial made simple and verbose
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/*
#include "main.h"

uint16_t timeout = 1000;
uint8_t delay = 100;

HAL_StatusTypeDef SerialWrite(const uint8_t *txData) {
	if (HAL_UART_Transmit(&huart2, txData, sizeof(txData), timeout) == HAL_OK) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(delay);
		return HAL_OK;
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		return HAL_ERROR;
	}
}
*/
