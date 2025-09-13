/*
 * uart_controller.c
 *
 *  Created on: Sep 9, 2024
 *      Author: [Your Name]
 *  Description: Simple UART/Serial controller for STM32F103
 *               Each method takes UART handle as parameter
 *               Includes automatic ready checking with timeout
 */

#include "uart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* Configuration defines */
#define UART_READY_TIMEOUT_MS 100   // Timeout waiting for UART ready (ms)
#define UART_TX_TIMEOUT_MS    50    // Timeout for transmit operations (ms)

/* UART Message Structure - Internal use only */
typedef struct {
    uint8_t* data;
    uint16_t length;
} UARTMessage_t;

/* Global variables for status tracking */
static uint8_t uartOK = 1;                      // Last operation status (1=success, 0=fail)
static HAL_StatusTypeDef lastStatus = HAL_OK;   // Last HAL operation status

/**
 * @brief Internal function to send UART frame
 * @param huart: Pointer to UART handle
 * @param msg: Pointer to message structure to send
 *
 * Waits for UART to be ready before sending (with timeout)
 * Sets uartOK to 0 if timeout or send fails
 */
static void sendUARTFrame(UART_HandleTypeDef* huart, UARTMessage_t* msg) {
    uint32_t tickstart = HAL_GetTick();

    /* Validate parameters */
    if (huart == NULL || msg == NULL || msg->data == NULL) {
        uartOK = 0;
        lastStatus = HAL_ERROR;
        return;
    }

    /* Wait for UART to be ready (with timeout) */
    while (!isUARTReady(huart)) {
        if ((HAL_GetTick() - tickstart) > UART_READY_TIMEOUT_MS) {
            uartOK = 0;
            lastStatus = HAL_TIMEOUT;
            return;
        }
    }

    /* Send the message */
    lastStatus = HAL_UART_Transmit(huart, msg->data, msg->length, UART_TX_TIMEOUT_MS);

    /* Update status flag based on result */
    uartOK = (lastStatus == HAL_OK) ? 1 : 0;
}

/**
 * @brief Send single byte over UART
 * @param huart: Pointer to UART handle
 * @param value: Byte to send (0-255)
 *
 * Sends one byte over serial
 * Check UARTok() after to verify success
 */
void UARTWrite(UART_HandleTypeDef* huart, uint8_t value) {
    UARTMessage_t msg;

    msg.data = &value;
    msg.length = 1;

    sendUARTFrame(huart, &msg);
}

/**
 * @brief Send string message over UART
 * @param huart: Pointer to UART handle
 * @param str: Null-terminated string to send
 *
 * Sends complete string in one transmission
 * Automatically handles length calculation
 */
void UARTSendMessage(UART_HandleTypeDef* huart, const char* str) {
    UARTMessage_t msg;

    if (str == NULL) {
        uartOK = 0;
        lastStatus = HAL_ERROR;
        return;
    }

    msg.data = (uint8_t*)str;
    msg.length = strlen(str);

    sendUARTFrame(huart, &msg);
}

/**
 * @brief Send sensor distance reading over UART
 * @param huart: Pointer to UART handle
 * @param mm: Distance in millimeters (0-65535)
 *
 * Formats and sends: "Distance: XXXmm\r\n"
 * Uses UARTSendMessage internally
 */
void UARTSendSensor(UART_HandleTypeDef* huart, uint16_t mm) {
    char buffer[32];

    /* Format the distance message */
    snprintf(buffer, sizeof(buffer), "Distance: %umm\r\n", mm);

    UARTSendMessage(huart, buffer);
}

/**
 * @brief Send formatted string over UART (like printf)
 * @param huart: Pointer to UART handle
 * @param format: Printf-style format string
 * @param ...: Variable arguments
 *
 * Maximum output length is 256 characters
 */
void UARTPrintf(UART_HandleTypeDef* huart, const char* format, ...) {
    char buffer[256];
    va_list args;

    if (format == NULL) {
        uartOK = 0;
        lastStatus = HAL_ERROR;
        return;
    }

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    UARTSendMessage(huart, buffer);
}

/**
 * @brief Send raw data buffer over UART
 * @param huart: Pointer to UART handle
 * @param data: Pointer to data buffer
 * @param length: Number of bytes to send
 *
 * For sending binary data or custom protocols
 */
void UARTSendData(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t length) {
    UARTMessage_t msg;

    if (data == NULL || length == 0) {
        uartOK = 0;
        lastStatus = HAL_ERROR;
        return;
    }

    msg.data = (uint8_t*)data;
    msg.length = length;

    sendUARTFrame(huart, &msg);
}

/**
 * @brief Check if UART peripheral is ready
 * @param huart: Pointer to UART handle
 * @return 1 if ready, 0 if not ready
 *
 * Also sets uartOK to 0 if not ready (for debugging)
 */
uint8_t isUARTReady(UART_HandleTypeDef* huart) {
    HAL_UART_StateTypeDef state;
    uint8_t ready;

    /* Check if handle is valid */
    if (huart == NULL) {
        uartOK = 0;
        return 0;
    }

    /* Check UART state */
    state = HAL_UART_GetState(huart);

    /* UART is ready if in READY state or only RX is busy */
    ready = (state == HAL_UART_STATE_READY ||
             state == HAL_UART_STATE_BUSY_RX) ? 1 : 0;

    /* Set debug flag if not ready */
    if (!ready) {
        uartOK = 0;
    }

    return ready;
}

/**
 * @brief Get last UART operation status
 * @return 1 if last operation succeeded, 0 if failed
 *
 * Use this to check if sends worked
 * Error state persists until next successful operation
 */
uint8_t UARTok(void) {
    return uartOK;
}

/**
 * @brief Get the current UART peripheral state
 * @param huart: Pointer to UART handle
 * @return HAL UART state
 *
 * Returns the current state of the UART peripheral
 */
HAL_UART_StateTypeDef UARTStatus(UART_HandleTypeDef* huart) {
    if (huart == NULL) {
        return HAL_UART_STATE_RESET;
    }
    return HAL_UART_GetState(huart);
}

/**
 * @brief Clear UART error flags
 * @param huart: Pointer to UART handle
 *
 * Call this if UART gets stuck in error state
 * Note: If UART needs full reinit, call MX_USARTx_UART_Init() from main
 */
void UARTClearErrors(UART_HandleTypeDef* huart) {
    if (huart == NULL) {
        return;
    }

    /* Clear all error flags */
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE);   // Parity error
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_FE);   // Framing error
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_NE);   // Noise error
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);  // Overrun error

    /* Reset error code in handle */
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    /* If UART is in error state, abort any ongoing transfers */
    if (HAL_UART_GetState(huart) == HAL_UART_STATE_ERROR) {
        HAL_UART_Abort(huart);
    }

    uartOK = 1;  // Clear global error flag
}

/**
 * @brief Non-blocking receive check
 * @param huart: Pointer to UART handle
 * @param buffer: Buffer to store received data
 * @param maxLength: Maximum bytes to receive
 * @return Number of bytes received (0 if none)
 *
 * Checks if data is available and receives it without blocking
 */
uint16_t UARTReceiveAvailable(UART_HandleTypeDef* huart, uint8_t* buffer, uint16_t maxLength) {
    HAL_StatusTypeDef rxStatus;
    uint16_t received;

    if (huart == NULL || buffer == NULL || maxLength == 0) {
        return 0;
    }

    /* Try to receive data with minimal timeout (1ms) */
    rxStatus = HAL_UART_Receive(huart, buffer, maxLength, 1);

    if (rxStatus == HAL_OK) {
        return maxLength;  // Got all requested data
    } else if (rxStatus == HAL_TIMEOUT && huart->RxXferCount < maxLength) {
        /* Calculate how much data we actually received */
        received = maxLength - huart->RxXferCount;
        return received;
    }

    return 0;  // No data or error
}

/**
 * @brief Blocking receive with timeout
 * @param huart: Pointer to UART handle
 * @param buffer: Buffer to store received data
 * @param length: Number of bytes to receive
 * @param timeout_ms: Timeout in milliseconds
 * @return 1 if success, 0 if timeout or error
 */
uint8_t UARTReceive(UART_HandleTypeDef* huart, uint8_t* buffer, uint16_t length, uint32_t timeout_ms) {
    if (huart == NULL || buffer == NULL || length == 0) {
        uartOK = 0;
        lastStatus = HAL_ERROR;
        return 0;
    }

    lastStatus = HAL_UART_Receive(huart, buffer, length, timeout_ms);

    uartOK = (lastStatus == HAL_OK) ? 1 : 0;
    return uartOK;
}

/**
 * @brief Get last HAL status code
 * @return HAL_StatusTypeDef of last operation
 *
 * Useful for debugging specific error types
 */
HAL_StatusTypeDef UARTGetLastStatus(void) {
    return lastStatus;
}
