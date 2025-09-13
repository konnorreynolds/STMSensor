/*
 * uart.h
 *
 *  Created on: Sep 9, 2024
 *      Author: [Your Name]
 *  Description: Header file for UART
 *               Each method takes UART handle as parameter
 */

#ifndef UART_H_
#define UART_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "main.h"

/* Basic transmission functions */
void UARTWrite(UART_HandleTypeDef* huart, uint8_t value);
void UARTSendMessage(UART_HandleTypeDef* huart, const char* str);
void UARTSendSensor(UART_HandleTypeDef* huart, uint16_t mm);
void UARTPrintf(UART_HandleTypeDef* huart, const char* format, ...);
void UARTSendData(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t length);

/* Receive functions */
uint16_t UARTReceiveAvailable(UART_HandleTypeDef* huart, uint8_t* buffer, uint16_t maxLength);
uint8_t UARTReceive(UART_HandleTypeDef* huart, uint8_t* buffer, uint16_t length, uint32_t timeout_ms);

/* Status and control functions */
uint8_t isUARTReady(UART_HandleTypeDef* huart);
uint8_t UARTok(void);
HAL_UART_StateTypeDef UARTStatus(UART_HandleTypeDef* huart);
void UARTClearErrors(UART_HandleTypeDef* huart);
HAL_StatusTypeDef UARTGetLastStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_CONTROLLER_H_ */
