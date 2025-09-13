/*
 * message_handler.h
 *
 *  Created on: Sep 8, 2024
 *      Author: Konnor Reynolds
 *  Description: Helper functions for sending data, messages, and sensor readings
 *               over CAN and USB interfaces.
 */

#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <stdint.h>

/**
 * @brief Send a single byte over CAN and USB.
 *
 * @param txData Pointer to the byte to send (0-255).
 *
 * Sends one byte using the current device ID.
 * After calling, use CANok() or USBok() to verify success.
 */
void SendData(uint8_t *txData);

/**
 * @brief Send a null-terminated string message over CAN and USB.
 *
 * @param msg Pointer to the string to send.
 *
 * Automatically splits long strings into multiple 8-byte frames
 * and sends until the null terminator '\0' is reached.
 */
void SendMessage(const char *msg);

/**
 * @brief Send a distance reading from a sensor over CAN and USB.
 *
 * @param distance Distance in millimeters (0-65535).
 *
 * Formats and sends the message: "Distance: XXXmm\r\n"
 */
void SendSensor(uint16_t distance);

#endif // MESSAGE_HANDLER_H
