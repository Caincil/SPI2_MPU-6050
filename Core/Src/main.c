/* USER CODE BEGIN Header */
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MPU6050_ADDRESS 0x68 << 1 // MPU-6050 I2C address
#define PWR_MGMT_1_REG 0x6B
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MPU6050_Init(void);
void MPU6050_Read_Accel(int16_t* accel_data);
void UART_Send_Data(int16_t* accel_data);
void IntToStr(int16_t value, char* buffer);
void MPU6050_TestConnection(void);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  MPU6050_Init();
  // HAL_UART_Transmit(&huart2, (uint8_t *)"Hello, World!\r\n", 15, HAL_MAX_DELAY);
  
  int16_t accel_data[3];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Read accelerometer data
    // MPU6050_Read_Accel(accel_data);
        
    // Send accelerometer data to Serial Monitor
    // UART_Send_Data(accel_data);
    MPU6050_TestConnection();    
    // Small delay
    HAL_Delay(500);
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
}

/* USER CODE BEGIN 4 */

void MPU6050_Init(void) {
    uint8_t data;
    
    // Wake up the MPU-6050 (clear sleep mode bit 6 in register PWR_MGMT_1)
    data = 0;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, PWR_MGMT_1_REG, 1, &data, 1, 1000);
}

void MPU6050_Read_Accel(int16_t* accel_data) {
    uint8_t data[6];
    
    // Read 6 bytes starting from ACCEL_XOUT_H (accelerometer data)
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, ACCEL_XOUT_H, 1, data, 6, 1000);
    
    // Combine high and low bytes for accelerometer
    accel_data[0] = (int16_t)(data[0] << 8 | data[1]); // Accel X
    accel_data[1] = (int16_t)(data[2] << 8 | data[3]); // Accel Y
    accel_data[2] = (int16_t)(data[4] << 8 | data[5]); // Accel Z
}

void UART_Send_Data(int16_t* accel_data) {
    char buffer[20]; // Buffer to hold the string representation
    char message[60] = "Accel X="; // Message prefix for X

    // Convert accel_data[0] (X-axis) to string and append to message
    IntToStr(accel_data[0], buffer);
    strcat(message, buffer);
    strcat(message, ", Y=");

    // Convert accel_data[1] (Y-axis) to string and append to message
    IntToStr(accel_data[1], buffer);
    strcat(message, buffer);
    strcat(message, ", Z=");

    // Convert accel_data[2] (Z-axis) to string and append to message
    IntToStr(accel_data[2], buffer);
    strcat(message, buffer);
    strcat(message, "\r\n");

    // Send the formatted string over UART
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}

void IntToStr(int16_t value, char* buffer) {
    // Convert integer to string manually
    int isNegative = 0;
    int i = 0;

    // Handle negative numbers
    if (value < 0) {
        isNegative = 1;
        value = -value; // Make the value positive
    }

    // Convert the integer to string (reverse order)
    do {
        buffer[i++] = (value % 10) + '0'; // Get the last digit and convert to character
        value /= 10;
    } while (value > 0);

    // Add negative sign if necessary
    if (isNegative) {
        buffer[i++] = '-';
    }

    // Null-terminate the string
    buffer[i] = '\0';

    // Reverse the string to get the correct order
    int start = 0;
    int end = i - 1;
    while (start < end) {
        char temp = buffer[start];
        buffer[start] = buffer[end];
        buffer[end] = temp;
        start++;
        end--;
    }
}

void MPU6050_TestConnection(void) {
    uint8_t who_am_i = 0;
    // Read the WHO_AM_I register (0x75), which should return 0x68
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, 0x75, 1, &who_am_i, 1, 1000);
    
    // Manually construct the message
    char buffer[20];
    buffer[0] = 'W';
    buffer[1] = 'H';
    buffer[2] = 'O';
    buffer[3] = '_';
    buffer[4] = 'A';
    buffer[5] = 'M';
    buffer[6] = '_';
    buffer[7] = 'I';
    buffer[8] = ':';
    buffer[9] = ' ';
    
    // Convert the hexadecimal value to characters
    uint8_t high_nibble = (who_am_i >> 4) & 0x0F;
    uint8_t low_nibble = who_am_i & 0x0F;
    
    buffer[10] = (high_nibble < 10) ? ('0' + high_nibble) : ('A' + high_nibble - 10);
    buffer[11] = (low_nibble < 10) ? ('0' + low_nibble) : ('A' + low_nibble - 10);
    
    buffer[12] = '\r';
    buffer[13] = '\n';
    
    // Transmit the message over UART
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, 14, HAL_MAX_DELAY);
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
