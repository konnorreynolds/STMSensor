/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb.h"
#include "gpio.h"
#include "vl53l0x.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* Error tracking - using simple flags for each system */
static uint8_t initError = 0;      // Set to 1 if initial sensor setup fails
static uint8_t sensorOK = 0;       // Sensor working flag (0 = error, 1 = working)
static uint8_t usbOK = 1;          // USB working flag (assume OK initially)
static uint8_t canOK = 1;          // CAN working flag (assume OK initially)
static uint8_t reconnectTimer = 0; // Timer for reconnection attempts (counts loops)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* Attempt to initialize the VL53L0X sensor on I2C bus 1
  * This checks if sensor is connected and responding with correct ID (0xEE) */
  if (VL53L0X_Init(&hi2c1) == HAL_OK) {
     /* Sensor found and initialized successfully */
     SendMessageUSB("VL53L0X Sensor Initialize Success...\n");
  } else {
     /* Sensor initialization failed - either not connected, wrong wiring,
      * or not a VL53L0X sensor (wrong ID) */
     SendMessageUSB("VL53L0X Sensor Initialize Fail...\n");
     initError = 1;  // Set error flag to track initialization failure
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Check initial sensor state from initialization */
	  if (initError) {
	     sensorOK = 0;  // Sensor failed during init
	  }

	  /* Attempt sensor reconnection every ~1 second if it's not working */
	  if (!sensorOK) {
	     reconnectTimer++;
	     if (reconnectTimer >= 50) {  // 50 loops * 20ms = 1 second
	         reconnectTimer = 0; // Reset reconnectTimer

	         /* Reset I2C bus before trying to reconnect
	                  * This clears any stuck I2C conditions */
	                 HAL_I2C_DeInit(&hi2c1);
	                 HAL_Delay(10);
	                 MX_I2C1_Init();  // Reinitialize I2C peripheral
	                 HAL_Delay(10);

	         /* Try to initialize sensor again */
	         if (VL53L0X_Init(&hi2c1) == HAL_OK) {
	             sensorOK = 1;  // Sensor is back online
	             initError = 0;
	             SendMessageUSB("Sensor reconnected\n");
	         }
	     }
	  }

	  /* Read and transmit sensor data if sensor is working */
	  if (sensorOK) {
	     uint16_t distance = VL53L0X_ReadDistance(&hi2c1);

	     if (distance != DISTANCE_ERROR) {
	         /* Valid reading - try to send over USB */
	         if (SendSensorUSB(distance) != USBD_OK) {
	             usbOK = 0;  // USB transmission failed
	         } else {
	             usbOK = 1;  // USB working fine
	         }
	         // TODO Add CAN Imple
	     } else {
	         /* Sensor read failed - mark as not working */
	         sensorOK = 0;
	         SendMessageUSB("Sensor read failed\n");
	     }
	  }

	  /* Check USB connection health (optional - implement based on your USB stack) */
	  if (USBStatus() != USBD_OK) {
	     usbOK = 0;  // USB disconnected or not ready
	  }

	  /* LED Status Indicator (Pull-Up: LOW = ON, HIGH = OFF)
	  * Different patterns for different states:
	  * - All OK: LED ON (solid)
	  * - Any error: LED OFF */
	  if (sensorOK && usbOK && canOK) {
	     /* Everything working - LED ON */
	     HAL_GPIO_WritePin(LED_BUILTIN, GPIO_PIN_RESET);
	  } else {
	     /* Something has failed - LED OFF
	      * Could expand this to blink patterns for specific errors */
	     HAL_GPIO_WritePin(LED_BUILTIN, GPIO_PIN_SET);

	     /* Print which system failed for debugging */
	     if (!sensorOK) SendMessageUSB("ERROR: Sensor\n");
	     if (!usbOK)    SendMessageUSB("ERROR: USB\n");
	     if (!canOK)    SendMessageUSB("ERROR: CAN\n");
	  }

	  /* Wait 20ms before next loop iteration
	  * This matches FRC CAN bus update period (50Hz) */
	  HAL_Delay(20);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Status_LED_Pin */
  GPIO_InitStruct.Pin = Status_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Status_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
