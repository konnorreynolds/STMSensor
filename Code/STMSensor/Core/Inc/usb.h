/*
 * usb.h
 *
 *  Created on: Sep 7, 2025
 *      Author: konno
 */

#ifndef INC_USB_H_
#define INC_USB_H_

HAL_StatusTypeDef USBWrite(uint8_t *txData);
HAL_StatusTypeDef SendMessageUSB(const char *msg);
HAL_StatusTypeDef SendSensorUSB(uint16_t distance);
USBD_StatusTypeDef USBStatus();

#endif /* INC_USB_H_ */
