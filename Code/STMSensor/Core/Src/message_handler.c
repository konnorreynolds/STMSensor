/*
 * message_handler.c
 *
 *  Created on: Sep 8, 2024
 *      Author: Konnor Reynolds
 *  Description: Small Helper super structure for handling messaging. 
 */ 

#include "main.h"
#include "can.h"
#include "uart.h"

extern UART_HandleTypeDef huart1;

/**
 * @brief Send single byte over CAN and USB
 * @param value: Byte to send (0-255)
 *
 * Sends one byte with current device ID
 * Check CANok() or USBok() after to verify success
 */
void SendData(uint8_t *txData) {
   //CANWrite(*txData);
   UARTWrite(&huart1, *txData);
}

/**
 * @brief Send string message over CAN and USB
 * @param str: Null-terminated string to send
 *
 * Automatically splits long strings into multiple 8-byte frames
 * Sends until '\0' is reached
 */
void SendMessage(const char *msg) {
   //CANSendMessage(msg);
   UARTSendMessage(&huart1, msg);
}

/**
 * @brief Send sensor distance reading over CAN and USB
 * @param mm: Distance in millimeters (0-65535)
 *
 * Formats and sends: "Distance: XXXmm\r\n"
 */
void SendSensor(uint16_t distance) {
   //CANSendSensor(distance);
   UARTSendSensor(&huart1, distance);
}
