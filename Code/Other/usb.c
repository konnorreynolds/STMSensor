/*
 * usb.c
 *
 *  Created on: Sep 6, 2024
 *      Author: Konnor Reynolds
 *  Description: Simple USB Messaging helper.
 */
#include "usb.h"
#include "main.h"
#include "string.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "stdbool.h"

/* Global variable to track last USB operation status */
static USBD_StatusTypeDef status;

static uint8_t usbOK = 1;            // Last operation status (1=success, 0=fail)

/**
 * @brief  Write data to USB CDC port with retry on busy
 * @param  txData: Pointer to data to transmit (must be null-terminated for strlen)
 * @return USBD_OK if successful, USBD_FAIL if error
 *
 * NOTE: Uses strlen() to determine length, so txData must end with '\0'
 *       The '\0' itself is NOT sent over USB
 */
USBD_StatusTypeDef USBWrite(uint8_t *txData) {
   uint16_t len = strlen((char*)txData);  // Get length of string to send

   /* Keep trying to send until USB is not busy */
   do {
       status = CDC_Transmit_FS(txData, len);  // Attempt to transmit
       if (status == USBD_BUSY) {
           HAL_Delay(10);  // Small delay before retrying (10ms)
       }
   } while (status == USBD_BUSY);  // Loop while USB is busy

   /* Return the final status (could be USBD_OK or USBD_FAIL) */
   if (status == USBD_OK) {
	   return status;
   } else {
	   usbOK = 0; // If status is not OK then let the debugger know.
	   return status;
   }
}

/**
 * @brief  Send a text message over USB
 * @param  msg: String message to send (must be null-terminated)
 * @return USBD_OK if successful, USBD_FAIL if error
 *
 * NOTE: Uses strlen() to determine length, so msg must end with '\0'
 *       The '\0' itself is NOT sent over USB
 */
USBD_StatusTypeDef SendMessageUSB(const char *msg) {
   uint8_t buffer[256];           // Local buffer for message
   size_t msgLen = strlen(msg);   // Get message length
   memcpy(buffer, msg, msgLen);   // Copy message to buffer
   buffer[msgLen] = '\0';         // Ensure null termination

   return USBWrite(buffer);       // Send the buffer over USB
}

/**
 * @brief  Send sensor distance reading over USB
 * @param  distance: Distance value in mm (0-65535)
 * @return USBD_OK if successful, USBD_FAIL if error
 *
 * Sends "Distance: XXXmm\r\n" over USB (without '\0')
 */
USBD_StatusTypeDef SendSensorUSB(uint16_t distance) {
   uint8_t buffer[256];        // Buffer for formatted message
   char distance_str[10];      // Temporary string for distance value

   /* Convert distance (uint16_t) to string */
   snprintf(distance_str, sizeof(distance_str), "%u", distance);

   /* Create full formatted message */
   snprintf((char*)buffer, sizeof(buffer), "Distance: %smm\r\n", distance_str);

   return USBWrite(buffer);    // Send formatted message over USB
}

/**
* @brief  Get the last USB operation status
* @return Last USB status code
*
* Returns the global status variable from the most recent USB operation
*/
USBD_StatusTypeDef USBStatus() {
   return status;
}

/**
 * @brief Get last USB operation status
 * @return 1 if last operation succeeded, 0 if failed
 *
 * Use this to check if sends worked
 * Error state persists until next successful operation
 */
uint8_t USBok(void) {
    return usbOK;
}
