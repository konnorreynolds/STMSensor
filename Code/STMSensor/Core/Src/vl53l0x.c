/* VL53L0X.c - VL53L0X ToF Sensor Implementation 
 * 
 * The sensor works by:
 * 1. Sending out a laser pulse
 * 2. Measuring time until it bounces back
 * 3. Calculating distance using speed of light
 * 4. Giving us the answer in millimeters
 */

#include "VL53L0X.h"

/**
 * @brief  Initialize VL53L0X sensor - Makes sure it's connected and ready
 * @param  hi2c: The I2C bus connected to the sensor
 * @return HAL_OK if sensor found, HAL_ERROR if something's wrong
 * 
 * We ask the sensor for its ID number to make sure it's really there
 * and that it's the right type of sensor (VL53L0X always returns 0xEE)
 */
HAL_StatusTypeDef VL53L0X_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t sensor_id;  // Variable to store the ID we read from sensor
    
    /* Read the sensor's ID register at address 0xC0 */
    if (HAL_I2C_Mem_Read(hi2c,                    // Use this I2C bus
                         VL53L0X_ADDR,             // Sensor's I2C address (0x52)
                         REG_IDENTIFICATION_ID,     // ID register location (0xC0)
                         I2C_MEMADD_SIZE_8BIT,     // Register address is 1 byte
                         &sensor_id,               // Store the ID here
                         1,                        // Read 1 byte
                         I2C_TIMEOUT_MS) != HAL_OK) {  // Wait max 100ms
        
        /* If we can't read the ID, sensor is probably not connected */
        return HAL_ERROR;  
    }
    
    /* Check if the ID matches what we expect (0xEE)
     * All genuine VL53L0X sensors have this same ID */
    if (sensor_id != VL53L0X_EXPECTED_ID) {
        /* Wrong ID means it's not a VL53L0X sensor */
        return HAL_ERROR;  
    }
    
    /* Sensor found and responding correctly */
    return HAL_OK;  
}

/**
 * @brief  Read distance from VL53L0X sensor
 * @param  hi2c: The I2C bus connected to the sensor
 * @return Distance in millimeters, or 0xFFFF (65535) if error
 * 
 * The measurement process:
 * 1. Tell sensor to take a measurement
 * 2. Wait for it to finish (check status register)
 * 3. Read the 16-bit result from the distance register
 */
uint16_t VL53L0X_ReadDistance(I2C_HandleTypeDef *hi2c) {
    uint8_t distance_bytes[2];  // Array to store the 2-byte distance value
    uint8_t command;            // Command byte to send to sensor
    uint8_t status;             // Sensor's status byte
    uint32_t wait_loops = 0;    // Counter to prevent waiting forever
    
    /* Tell sensor to start a measurement by writing 0x01 to register 0x00 */
    command = START_SINGLE_MEASURE;  // 0x01 = take one measurement
    
    if (HAL_I2C_Mem_Write(hi2c,                   // Use this I2C bus
                          VL53L0X_ADDR,            // Sensor's I2C address
                          REG_SYSRANGE_START,      // START register (0x00)
                          I2C_MEMADD_SIZE_8BIT,    // Register address is 1 byte
                          &command,                // Send this command (0x01)
                          1,                       // Send 1 byte
                          I2C_TIMEOUT_MS) != HAL_OK) {  // Wait max 100ms
        
        /* Couldn't send command to sensor */
        return DISTANCE_ERROR;  // Return error value (0xFFFF)
    }
    
    /* Wait for measurement to complete
     * The sensor sets bit 0 of the status register to 1 when done
     * We keep checking this bit until it becomes 1 */
    do {
        /* Read the status register at address 0x14 */
        if (HAL_I2C_Mem_Read(hi2c,                    // Use this I2C bus
                             VL53L0X_ADDR,             // Sensor's I2C address  
                             REG_RESULT_RANGE_STATUS,  // Status register (0x14)
                             I2C_MEMADD_SIZE_8BIT,     // Register address is 1 byte
                             &status,                  // Store status here
                             1,                        // Read 1 byte
                             I2C_TIMEOUT_MS) != HAL_OK) {  // Wait max 100ms
            
            /* Couldn't read status */
            return DISTANCE_ERROR;  
        }
        
        /* Check if we've been waiting too long (>100ms is unusual) */
        wait_loops++;
        if (wait_loops > MEASUREMENT_TIMEOUT_MS) {
            /* Sensor is taking too long, something's wrong */
            return DISTANCE_ERROR;
        }
        
        /* Wait 1 millisecond before checking again */
        HAL_Delay(1);
        
        /* Keep looping while bit 0 is still 0 (not ready) */
    } while ((status & MEASUREMENT_READY_BIT) == 0);
    
    /* Measurement is ready! Read the distance value (2 bytes at address 0x1E) */
    if (HAL_I2C_Mem_Read(hi2c,                    // Use this I2C bus
                         VL53L0X_ADDR,             // Sensor's I2C address (0x52)
                         REG_RESULT_RANGE_VAL,     // Distance register (0x1E)
                         I2C_MEMADD_SIZE_8BIT,     // Register address is 1 byte
                         distance_bytes,           // Store 2 bytes here
                         2,                        // Read 2 bytes
                         I2C_TIMEOUT_MS) != HAL_OK) {  // Wait max 100ms
        
        /* Couldn't read the distance */
        return DISTANCE_ERROR;
    }
    
    /* Combine the 2 bytes into one 16-bit number
     * First byte is the high byte (multiply by 256)
     * Second byte is the low byte
     * Example: if bytes are [0x01, 0x2C], distance = (1 * 256) + 44 = 300mm */
    uint16_t distance = (distance_bytes[0] << 8) | distance_bytes[1];
    
    /* Return the distance in millimeters */
    return distance;
}
