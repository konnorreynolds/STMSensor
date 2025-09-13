/*
 * can_controller.h
 *
 *  Created on: Sep 8, 2024
 *      Author: Konnor Reynolds
 *  Description: Simple CAN bus controller for STM32F103
 *               FRC robotics communication
 */

#ifndef CAN_CONTROLLER_H
#define CAN_CONTROLLER_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/* Function Prototypes */

/**
 * @brief Initialize CAN controller
 * @param hcan: Pointer to CAN handle from main.c (already initialized by CubeMX)
 *
 * Sets up CAN filters to accept all messages and starts CAN peripheral
 * Call this ONCE after CubeMX initialization
 */
void CAN_ControllerInit(CAN_HandleTypeDef* hcan);

/**
 * @brief Send single byte over CAN
 * @param value: Byte to send (0-255)
 *
 * Sends one byte with current device ID
 * Check CANok() after to verify success
 */
void CANWrite(uint8_t value);

/**
 * @brief Send string message over CAN
 * @param str: Null-terminated string to send
 *
 * Automatically splits long strings into multiple 8-byte frames
 * Sends until '\0' is reached
 */
void CANSendMessage(const char* str);

/**
 * @brief Send sensor distance reading over CAN
 * @param mm: Distance in millimeters (0-65535)
 *
 * Formats and sends: "Distance: XXXmm\r\n"
 * Uses CANSendMessage internally
 */
void CANSendSensor(uint16_t mm);

/**
 * @brief Set device ID for CAN messages
 * @param id: New device ID to use
 *
 * All future messages will use this ID
 */
void CANSetID(uint32_t id);

/**
 * @brief Check if CAN peripheral is ready
 * @return 1 if ready, 0 if not ready
 *
 * Also sets canOK to 0 if not ready (for debugging)
 */
uint8_t isCANReady(void);

/**
 * @brief Get last CAN operation status
 * @return 1 if last operation succeeded, 0 if failed
 *
 * Use this to check if sends worked
 * Error state persists until next successful operation
 */
uint8_t CANok(void);

/**
 * @brief Get the current CAN peripheral state
 * @return HAL CAN state
 *
 * Returns the current state of the CAN peripheral
 */
HAL_CAN_StateTypeDef CANStatus(void);

#endif /* CAN_CONTROLLER_H */
