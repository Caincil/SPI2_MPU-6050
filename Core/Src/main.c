#/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MPU6050_ADDR 0x68
#define ACCEL_XOUT_H 0x3B
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MPU6050_Init(void);
void MPU6050_ReadAccel(int16_t *accelX, int16_t *accelY, int16_t *accelZ);
void MPU6050_Write(uint8_t reg, uint8_t data);
uint8_t MPU6050_Read(uint8_t reg);
void send_accel_data_over_uart(int16_t accelX, int16_t accelY, int16_t accelZ);

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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MPU6050_Init();
  /* USER CODE BEGIN 2 */
  int16_t accelX, accelY, accelZ;
  char buffer[64];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // TODO: 
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
      // Read accelerometer data
    MPU6050_ReadAccel(&accelX, &accelY, &accelZ);

    // Send accelerometer data over UART
    send_accel_data_over_uart(accelX, accelY, accelZ);

    // Wait for a short period
    HAL_Delay(500);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* MPU6050 Initialization Function */
void MPU6050_Init(void)
{
    // Wake up the MPU6050 by writing 0 to the power management register
    MPU6050_Write(0x6B, 0x00);

    // Set accelerometer configuration (e.g., ±2g range)
    MPU6050_Write(0x1C, 0x00);

    // Set gyroscope configuration (e.g., ±250°/s range)
    MPU6050_Write(0x1B, 0x00);
}

/* Function to Read Accelerometer Data */
void MPU6050_ReadAccel(int16_t *accelX, int16_t *accelY, int16_t *accelZ)
{
    uint8_t data[6];

    /* Read 6 bytes of accelerometer data starting from ACCEL_XOUT_H */
    
    uint8_t reg = ACCEL_XOUT_H | 0x80; // Set MSB to 1 for read operation
    HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, data, 6, HAL_MAX_DELAY);
    

    /* Combine the high and low bytes to form the 16-bit accelerometer values */
    *accelX = (int16_t)(data[0] << 8 | data[1]);
    *accelY = (int16_t)(data[2] << 8 | data[3]);
    *accelZ = (int16_t)(data[4] << 8 | data[5]);
}

/* Function to Write to MPU6050 Register */
void MPU6050_Write(uint8_t reg, uint8_t data)
{
    
    uint8_t txData[2] = {reg, data};
    HAL_SPI_Transmit(&hspi2, txData, 2, HAL_MAX_DELAY);
    
}

/* Function to Read from MPU6050 Register */
uint8_t MPU6050_Read(uint8_t reg)
{
    
    uint8_t txData = reg | 0x80; // Set MSB to 1 for read operation
    uint8_t rxData;
    HAL_SPI_Transmit(&hspi2, &txData, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, &rxData, 1, HAL_MAX_DELAY);
    
    return rxData;
}

void int16_to_str(int16_t num, char* str) {
    char temp[7];  // Temporary buffer (6 digits max for int16_t + sign + null terminator)
    int i = 0;
    int is_negative = 0;

    // Handle negative numbers
    if (num < 0) {
        is_negative = 1;
        num = -num;
    }

    // Process number
    do {
        temp[i++] = (num % 10) + '0';  // Get the last digit and convert to character
        num /= 10;
    } while (num > 0);

    // If the number was negative, add the '-' sign
    if (is_negative) {
        temp[i++] = '-';
    }

    // Reverse the string since digits are in reverse order
    int j = 0;
    while (i > 0) {
        str[j++] = temp[--i];
    }

    str[j] = '\0';  // Null-terminate the string
}


// Function to send accelerometer data over UART
void send_accel_data_over_uart(int16_t accelX, int16_t accelY, int16_t accelZ) {
    char buffer[100] = "Accel X: ";
    char x_str[7], y_str[7], z_str[7];  // Buffers to hold string representations of accel data
    char y_label[] = ", Y: ";
    char z_label[] = ", Z: ";
    char newline[] = "\r\n";

    // Convert the accelerometer values to strings
    int16_to_str(accelX, x_str);
    int16_to_str(accelY, y_str);
    int16_to_str(accelZ, z_str);

    // Concatenate the strings together to form the complete message
    strcat(buffer, x_str);
    strcat(buffer, y_label);
    strcat(buffer, y_str);
    strcat(buffer, z_label);
    strcat(buffer, z_str);
    strcat(buffer, newline);

    // Transmit the formatted string over UART
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sizeof(buffer)-1 , 1000);
}
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

#ifdef  USE_FULL_ASSERT
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
