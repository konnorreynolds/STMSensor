/*
 * can_controller.c
 *
 *  Created on: Sep 8, 2024
 *      Author: Konnor Reynolds
 *  Description: Simple CAN bus controller for STM32F103
 *               FRC robotics communication
 */

#include "main.h"
#include <string.h>
#include <stdio.h>

/* Configuration defines */
#define CAN_READY_TIMEOUT_MS 100   // Timeout waiting for CAN ready (ms)
#define DEFAULT_DEVICE_ID 0x100     // Default CAN ID

/* CAN Message Structure */
typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t length;
} CANMessage_t;

/* Global variables */
static CAN_HandleTypeDef* hcan_ptr = NULL;     // Pointer to CAN handle
static uint32_t deviceID = DEFAULT_DEVICE_ID;  // Current device ID
static uint8_t canOK = 1;                      // Last operation status (1=success, 0=fail)

/* Global variable to track last CAN operation status */
static HAL_StatusTypeDef status = HAL_OK;

/* Forward declaration */
uint8_t isCANReady(void);

/**
 * @brief Internal function to send CAN frame
 * @param msg: Pointer to message structure to send
 *
 * Waits for CAN to be ready before sending (with timeout)
 * Sets canOK to 0 if timeout or send fails
 */
static void sendCANFrame(CANMessage_t* msg) {
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint32_t tickstart = HAL_GetTick();

    /* Wait for CAN to be ready (with timeout) */
    while (!isCANReady()) {
        if ((HAL_GetTick() - tickstart) > CAN_READY_TIMEOUT_MS) {
            canOK = 0;  // Timeout - set debug flag
            status = HAL_TIMEOUT;  // Set status to timeout
            return;
        }
    }

    /* Configure TX header */
    txHeader.StdId = msg->id;              // Standard ID
    txHeader.IDE = CAN_ID_STD;             // Using standard ID (not extended)
    txHeader.RTR = CAN_RTR_DATA;           // Data frame (not remote request)
    txHeader.DLC = msg->length;            // Data length
    txHeader.TransmitGlobalTime = DISABLE; // Don't need timestamp

    /* Send the message */
    status = HAL_CAN_AddTxMessage(hcan_ptr, &txHeader, msg->data, &txMailbox);

    /* Update debug flag based on result */
    if (status != HAL_OK) {
        canOK = 0;  // Send failed - set debug flag
    } else {
        canOK = 1;  // Send succeeded - clear debug flag
    }
}

/**
 * @brief Initialize CAN controller
 * @param hcan: Pointer to CAN handle from main.c (already initialized by CubeMX)
 *
 * Sets up CAN filters to accept all messages and starts CAN peripheral
 * Call this ONCE after CubeMX initialization
 */
void CAN_ControllerInit(CAN_HandleTypeDef* hcan) {
    hcan_ptr = hcan;  // Store handle pointer

    /* Configure filter to accept all messages */
    CAN_FilterTypeDef filter;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;  // Accept all IDs
    filter.FilterMaskIdLow = 0x0000;   // Accept all IDs
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &filter);  // Apply filter settings
    HAL_CAN_Start(hcan);                  // Start CAN peripheral
}

/**
 * @brief Send single byte over CAN
 * @param value: Byte to send (0-255)
 *
 * Sends one byte with current device ID
 * Check CANok() after to verify success
 */
void CANWrite(uint8_t value) {
    CANMessage_t msg;

    msg.id = deviceID;        // Use current device ID
    msg.data[0] = value;       // Put byte in first position
    msg.length = 1;            // Only sending 1 byte

    /* Clear unused bytes */
    memset(&msg.data[1], 0, 7);

    sendCANFrame(&msg);  // Send it
}

/**
 * @brief Send string message over CAN
 * @param str: Null-terminated string to send
 *
 * Automatically splits long strings into multiple 8-byte frames
 * Sends until '\0' is reached
 */
void CANSendMessage(const char* str) {
    CANMessage_t msg;
    msg.id = deviceID;  // Use current device ID

    int offset = 0;

    /* Keep sending 8-byte chunks until end of string */
    while (str[offset] != '\0') {
        msg.length = 0;

        /* Clear data buffer */
        memset(msg.data, 0, 8);

        /* Copy up to 8 bytes into message */
        for (int i = 0; i < 8 && str[offset + i] != '\0'; i++) {
            msg.data[i] = str[offset + i];
            msg.length++;
        }

        sendCANFrame(&msg);  // Send this chunk
        offset += 8;         // Move to next chunk
    }
}

/**
 * @brief Send sensor distance reading over CAN
 * @param mm: Distance in millimeters (0-65535)
 *
 * Formats and sends: "Distance: XXXmm\r\n"
 * Uses CANSendMessage internally
 */
void CANSendSensor(uint16_t mm) {
    char buffer[32];  // Buffer for formatted string

    /* Format the distance message */
    snprintf(buffer, sizeof(buffer), "Distance: %umm\r\n", mm);

    CANSendMessage(buffer);  // Send using message function
}

/**
 * @brief Set device ID for CAN messages
 * @param id: New device ID to use
 *
 * All future messages will use this ID
 */
void CANSetID(uint32_t id) {
    deviceID = id;
}

/**
 * @brief Check if CAN peripheral is ready
 * @return 1 if ready, 0 if not ready
 *
 * Also sets canOK to 0 if not ready (for debugging)
 */
uint8_t isCANReady(void) {
    /* Check if initialized */
    if (hcan_ptr == NULL) {
        canOK = 0;  // Not initialized - set debug flag
        return 0;
    }

    /* Check CAN state */
    HAL_CAN_StateTypeDef state = HAL_CAN_GetState(hcan_ptr);
    uint8_t ready = (state == HAL_CAN_STATE_READY || state == HAL_CAN_STATE_LISTENING) ? 1 : 0;

    /* Set debug flag if not ready */
    if (!ready) {
        canOK = 0;
    }

    return ready;
}

/**
 * @brief Get last CAN operation status
 * @return 1 if last operation succeeded, 0 if failed
 *
 * Use this to check if sends worked
 * Error state persists until next successful operation
 */
uint8_t CANok(void) {
    return canOK;
}

/**
 * @brief Get the current CAN peripheral state
 * @return HAL CAN state
 *
 * Returns the current state of the CAN peripheral
 */
HAL_CAN_StateTypeDef CANStatus(void) {
    if (hcan_ptr == NULL) {
        return HAL_CAN_STATE_RESET;  // Not initialized
    }
    return HAL_CAN_GetState(hcan_ptr);
}
