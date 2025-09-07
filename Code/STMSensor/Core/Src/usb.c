/*
 * usb.c
 *
 *  Created on: Sep 7, 2025
 *      Author: Konnor Reynolds
 */
#include "main.h"
#include "string.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "stdbool.h"

USBD_StatusTypeDef status;

USBD_StatusTypeDef USBWrite(uint8_t *txData) {
	uint16_t len = strlen((char*)txData);
	// Keep trying until it's not busy
	    do {
	        status = CDC_Transmit_FS(txData, len);
	        if (status == USBD_BUSY) {
	            HAL_Delay(10);  // Small delay before retrying
	        }
	    } while (status == USBD_BUSY);

	if (status == USBD_OK) {
		return status;
	} else {
		return status;
	}
}

USBD_StatusTypeDef SendMessageUSB(const char *msg) {
	uint8_t buffer[256];
	size_t msgLen = strlen(msg);
	memcpy(buffer, msg, msgLen);
	buffer[msgLen] = '\0';

	return USBWrite(buffer);
}

USBD_StatusTypeDef SendSensorUSB(uint16_t distance) {
    uint8_t buffer[256];
    char distance_str[10];

    // Convert distance to string
    snprintf(distance_str, sizeof(distance_str), "%u", distance);

    // Create full message
    snprintf((char*)buffer, sizeof(buffer), "Distance: %smm\r\n", distance_str);

    return USBWrite(buffer);
}

USBD_StatusTypeDef USBStatus() {
	return status;
}
