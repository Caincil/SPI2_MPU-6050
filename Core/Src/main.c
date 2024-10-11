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
void MPU6050_Read_Accel(int16_t* accel_data, int16_t* gyro_data);
void UART_Send_Accel_Data(int16_t* accel_data, int16_t* gyro_data);
int IntToStr(int16_t value, char* buffer);
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
  int16_t accel_data[3], gyro_data[3];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Read accelerometer data
    MPU6050_Read_Accel(accel_data, gyro_data);
        
    // Send accelerometer data to Serial Monitor
    UART_Send_Accel_Data(accel_data, gyro_data);
        
    // Small delay
    HAL_Delay(100);
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
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, PWR_MGMT_1_REG, 1, &data, 1, 100);
}

void MPU6050_Read_Accel(int16_t* accel_data, int16_t* gyro_data) {
    uint8_t data[14];
    
    // Read 6 bytes starting from ACCEL_XOUT_H (accelerometer data)
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, ACCEL_XOUT_H, 1, data, 6, 100);
    
    // Combine high and low bytes for accelerometer
    accel_data[0] = (int16_t)(data[0] << 8 | data[1]); // Accel X
    accel_data[1] = (int16_t)(data[2] << 8 | data[3]); // Accel Y
    accel_data[2] = (int16_t)(data[4] << 8 | data[5]); // Accel Z

     // Combine high and low bytes for gyroscope
    gyro_data[0] = (int16_t)(data[8] << 8 | data[9]); // Gyro X
    gyro_data[1] = (int16_t)(data[10] << 8 | data[11]); // Gyro Y
    gyro_data[2] = (int16_t)(data[12] << 8 | data[13]); // Gyro Z
}

void UART_Send_Accel_Data(int16_t* accel_data, int16_t* gyro_data) {
    char buffer[100]; // Buffer to hold the message
    int index = 0;

    // Add "Accel X="
    buffer[index++] = 'A';
    buffer[index++] = 'c';
    buffer[index++] = 'c';
    buffer[index++] = 'e';
    buffer[index++] = 'l';
    buffer[index++] = ' ';
    buffer[index++] = 'X';
    buffer[index++] = '=';

    // Convert accel_data[0] (X-axis) to string and add to buffer
    index += IntToStr(accel_data[0], &buffer[index]);

    // Add ", Y="
    buffer[index++] = ',';
    buffer[index++] = ' ';
    buffer[index++] = 'Y';
    buffer[index++] = '=';

    // Convert accel_data[1] (Y-axis) to string and add to buffer
    index += IntToStr(accel_data[1], &buffer[index]);

    // Add ", Z="
    buffer[index++] = ',';
    buffer[index++] = ' ';
    buffer[index++] = 'Z';
    buffer[index++] = '=';

    // Convert accel_data[2] (Z-axis) to string and add to buffer
    index += IntToStr(accel_data[2], &buffer[index]);

     // Add " | Gyro X="
    buffer[index++] = ' ';
    buffer[index++] = '|';
    buffer[index++] = ' ';
    buffer[index++] = 'G';
    buffer[index++] = 'y';
    buffer[index++] = 'r';
    buffer[index++] = 'o';
    buffer[index++] = ' ';
    buffer[index++] = 'X';
    buffer[index++] = '=';

    // Convert gyro_data[0] (X-axis) to string and add to buffer
    index += IntToStr(gyro_data[0], &buffer[index]);

    // Add ", Y="
    buffer[index++] = ',';
    buffer[index++] = ' ';
    buffer[index++] = 'Y';
    buffer[index++] = '=';

    // Convert gyro_data[1] (Y-axis) to string and add to buffer
    index += IntToStr(gyro_data[1], &buffer[index]);

    // Add ", Z="
    buffer[index++] = ',';
    buffer[index++] = ' ';
    buffer[index++] = 'Z';
    buffer[index++] = '=';

    // Convert gyro_data[2] (Z-axis) to string and add to buffer
    index += IntToStr(gyro_data[2], &buffer[index]);

    // Add "\r\n"
    buffer[index++] = '\r';
    buffer[index++] = '\n';

    // Transmit the constructed message over UART
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, index, HAL_MAX_DELAY);
}

int IntToStr(int16_t value, char* buffer) {
    int isNegative = 0;
    int i = 0;

    // Handle negative numbers
    if (value < 0) {
        isNegative = 1;
        value = -value;
    }

    // Convert the integer to string in reverse order
    do {
        buffer[i++] = (value % 10) + '0';
        value /= 10;
    } while (value > 0);

    // Add negative sign if needed
    if (isNegative) {
        buffer[i++] = '-';
    }

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

    // Return the length of the string
    return i;
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
