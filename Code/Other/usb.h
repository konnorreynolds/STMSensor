/*
 * usb.h
 *
 *  Created on: Sep 6, 2024
 *      Author: Konnor Reynolds
 *  Description: Simple USB Messaging helper.
 */

#ifndef USB_H_
#define USB_H_

#include "main.h"
#include "usbd_cdc_if.h"
#include <stdint.h>

/* ========== FUNCTION PROTOTYPES ========== */

/**
 * @brief  Write data to USB CDC port with retry on busy
 * @param  txData: Pointer to data to transmit (must be null-terminated for strlen)
 * @return USBD_OK if successful, USBD_FAIL if error
 *
 * NOTE: Uses strlen() to determine length, so txData must end with '\0'
 *       The '\0' itself is NOT sent over USB
 */
USBD_StatusTypeDef USBWrite(uint8_t *txData);

/**
 * @brief  Send a text message over USB
 * @param  msg: String message to send (must be null-terminated)
 * @return USBD_OK if successful, USBD_FAIL if error
 *
 * NOTE: Does NOT send the '\0' character over USB
 */
USBD_StatusTypeDef SendMessageUSB(const char *msg);

/**
 * @brief  Send sensor distance reading over USB
 * @param  distance: Distance value in mm (0-65535)
 * @return USBD_OK if successful, USBD_FAIL if error
 *
 * Sends "Distance: XXXmm\r\n" over USB (without '\0')
 */
USBD_StatusTypeDef SendSensorUSB(uint16_t distance);

/**
 * @brief Get last USB operation status
 * @return 1 if last operation succeeded, 0 if failed
 *
 * Use this to check if sends worked
 * Error state persists until next successful operation
 */
uint8_t USBok(void);

#endif /* USB_H_ */
