/* VL53L0X.h - VL53L0X ToF (Time of Flight) Sensor Header 
 * 
 * The VL53L0X measures distance by sending invisible laser light
 * and timing how long it takes to bounce back
 * 
 * Connection to STM32 BluePill:
 * VL53L0X VIN -> 3.3V
 * VL53L0X GND -> GND  
 * VL53L0X SCL -> PB6 (I2C Clock)
 * VL53L0X SDA -> PB7 (I2C Data)
 */

#ifndef VL53L0X_H_
#define VL53L0X_H_

#include "main.h"
#include <stdint.h>

/* ========== I2C ADDRESS ========== */
/* Every I2C device has an address like a house number
 * The VL53L0X lives at address 0x29, but we shift it left by 1 bit for STM32 HAL */
#define VL53L0X_ADDR            0x52  // Binary: 0101 0010

/* ========== SENSOR REGISTERS ========== */
/* Registers are like mailboxes inside the sensor - each holds different information */

/* Control Registers */
#define REG_SYSRANGE_START      0x00  // Write 0x01 here to start a measurement

/* Status Registers */
#define REG_RESULT_RANGE_STATUS 0x14  // Bit 0 tells us if measurement is ready

/* Result Registers */
#define REG_RESULT_RANGE_VAL    0x1E  // The actual distance value (2 bytes) = 0x14 + 10

/* ID Register */
#define REG_IDENTIFICATION_ID   0xC0  // Should always read 0xEE for VL53L0X

/* ========== CONSTANTS ========== */
#define VL53L0X_EXPECTED_ID     0xEE  // The ID all VL53L0X sensors have
#define MEASUREMENT_TIMEOUT_MS  100   // Max time to wait for measurement (milliseconds)
#define I2C_TIMEOUT_MS         100   // Max time to wait for I2C communication

/* ========== SPECIAL VALUES ========== */
#define DISTANCE_ERROR         0xFFFF // Returned when something goes wrong (65535)
#define START_SINGLE_MEASURE   0x01   // Command to start one measurement
#define MEASUREMENT_READY_BIT  0x01   // Bit 0 in status = measurement done

/* ========== FUNCTION PROTOTYPES ========== */

/**
 * @brief  Check if the sensor is connected and working
 * @param  hi2c: Pointer to the I2C bus we're using
 * @return HAL_OK if sensor found, HAL_ERROR if not found
 */
HAL_StatusTypeDef VL53L0X_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Measure the distance to an object
 * @param  hi2c: Pointer to the I2C bus we're using  
 * @return Distance in millimeters (mm), or 0xFFFF if error
 * 
 * @note   Range: 50mm to 1200mm (5cm to 1.2 meters)
 * @note   Takes about 20-30ms to complete a measurement
 */
uint16_t VL53L0X_ReadDistance(I2C_HandleTypeDef *hi2c);

#endif /* VL53L0X_H_ */
