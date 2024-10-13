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
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MPU6050_ADDRESS 0x68 << 1 // MPU-6050 I2C address
#define PWR_MGMT_1_REG 0x6B
#define ACCEL_XOUT_H 0x3B
#define _USE_MATH_DEFINES
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
void MPU6050_Read_Accel(int16_t* accel_data, int16_t* gyro_data, int16_t* accel_bias,int16_t* gyro_bias);
void UART_Send_Angles(float pitch, float roll, float yaw);
void Calculate_Angles(int16_t* accel_data, int16_t* gyro_data, float* pitch, float* roll, float* yaw);
int IntToStr(int16_t value, char* buffer);
int FloatToStr(float value, char* buffer);
void MPU6050_Calibrate(int16_t* accel_bias, int16_t* gyro_bias);
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
  int16_t accel_bias[3] = {0, 0, 0}, gyro_bias[3] = {0, 0, 0};
  float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;

   MPU6050_Calibrate(accel_bias, gyro_bias);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Read accelerometer data
    MPU6050_Read_Accel(accel_data, gyro_data, accel_bias, gyro_bias);

    // Calculate pitch, roll, and yaw
    Calculate_Angles(accel_data, gyro_data, &pitch, &roll, &yaw);
    
    // Send accelerometer data to Serial Monitor
    UART_Send_Angles(pitch, roll, yaw);
        
    // Small delay
    HAL_Delay(2);
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

void MPU6050_Calibrate(int16_t* accel_bias, int16_t* gyro_bias) {
    int32_t accel_sum[3] = {0, 0, 0};
    int32_t gyro_sum[3] = {0, 0, 0};
    int16_t accel_data[3], gyro_data[3];
    
    // Collect 200 samples
    for (int i = 0; i < 200; i++) {
        MPU6050_Read_Accel(accel_data, gyro_data, accel_bias, gyro_bias);

        // Sum the readings for bias calculation
        accel_sum[0] += accel_data[0];
        accel_sum[1] += accel_data[1];
        accel_sum[2] += accel_data[2];
        gyro_sum[0] += gyro_data[0];
        gyro_sum[1] += gyro_data[1];
        gyro_sum[2] += gyro_data[2];

        HAL_Delay(5); // Small delay between readings
    }

    // Calculate the average values (biases)
    accel_bias[0] = accel_sum[0] / 200;
    accel_bias[1] = accel_sum[1] / 200;
    accel_bias[2] = accel_sum[2] / 200;
    gyro_bias[0] = gyro_sum[0] / 200;
    gyro_bias[1] = gyro_sum[1] / 200;
    gyro_bias[2] = gyro_sum[2] / 200;
}


void MPU6050_Read_Accel(int16_t* accel_data, int16_t* gyro_data, int16_t* accel_bias, int16_t* gyro_bias) {
    uint8_t data[14];
    
    // Read 14 bytes starting from ACCEL_XOUT_H (accelerometer and gyroscope data)
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, ACCEL_XOUT_H, 1, data, 14, 100);
    
    // Combine high and low bytes for accelerometer
    accel_data[0] = (int16_t)(data[0] << 8 | data[1]) - accel_bias[0]; // Accel X
    accel_data[1] = (int16_t)(data[2] << 8 | data[3]) - accel_bias[1]; // Accel Y
    accel_data[2] = (int16_t)(data[4] << 8 | data[5]) - accel_bias[2]; // Accel Z

    // Combine high and low bytes for gyroscope
    gyro_data[0] = (int16_t)(data[8] << 8 | data[9]) - gyro_bias[0]; // Gyro X
    gyro_data[1] = (int16_t)(data[10] << 8 | data[11]) - gyro_bias[1]; // Gyro Y
    gyro_data[2] = (int16_t)(data[12] << 8 | data[13]) - gyro_bias[2]; // Gyro Z
}

void Calculate_Angles(int16_t* accel_data, int16_t* gyro_data, float* pitch, float* roll, float* yaw)
{
    // Complementary filter constant (adjust based on performance)
    const float alpha = 0.98f;

    // Convert accelerometer data to angles
    float accel_x = accel_data[0] / 16384.0f; // Convert to g
    float accel_y = accel_data[1] / 16384.0f; // Convert to g
    float accel_z = accel_data[2] / 16384.0f; // Convert to g

    // Calculate pitch and roll from accelerometer
    float accel_pitch = atan2f(accel_y, accel_z) * 180 / 3.14159265358979323846;
    float accel_roll = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) * 180 / 3.14159265358979323846;

    // Calculate time delta (in seconds) for gyro integration
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f; // Delta time in seconds
    last_time = current_time;

    // Integrate gyroscope data for pitch and roll (gyro data in degrees per second)
    float gyro_pitch_rate = gyro_data[0] / 131.0f; // Convert to degrees/sec
    float gyro_roll_rate = gyro_data[1] / 131.0f;  // Convert to degrees/sec
    float gyro_yaw_rate = gyro_data[2] / 131.0f;   // Convert to degrees/sec

    // Apply complementary filter for pitch and roll
    *pitch = alpha * (*pitch + gyro_pitch_rate * dt) + (1.0f - alpha) * accel_pitch;
    *roll = alpha * (*roll + gyro_roll_rate * dt) + (1.0f - alpha) * accel_roll;

    // Integrate gyroscope data for yaw (no accelerometer data available for yaw)
    *yaw += gyro_yaw_rate * dt;

    if (*yaw >= 360.0f) {
        *yaw -= 360.0f;  // If yaw exceeds 360, subtract 360 to wrap around
    } else if (*yaw < 0.0f) {
        *yaw += 360.0f;  // If yaw drops below 0, add 360 to wrap around
    }
}


void UART_Send_Angles(float pitch, float roll, float yaw) {
    char buffer[100];
    int index = 0;

    // Add "Pitch: "
    const char* pitch_label = "Pitch: ";
    for (int i = 0; pitch_label[i] != '\0'; i++) {
        buffer[index++] = pitch_label[i];
    }

    // Convert pitch to string and add to buffer
    index += FloatToStr(pitch, &buffer[index]);

    // Add ", Roll: "
    const char* roll_label = ", Roll: ";
    for (int i = 0; roll_label[i] != '\0'; i++) {
        buffer[index++] = roll_label[i];
    }

    // Convert roll to string and add to buffer
    index += FloatToStr(roll, &buffer[index]);

    // Add ", Yaw: "
    const char* yaw_label = ", Yaw: ";
    for (int i = 0; yaw_label[i] != '\0'; i++) {
        buffer[index++] = yaw_label[i];
    }

    // Convert yaw to string and add to buffer
    index += FloatToStr(yaw, &buffer[index]);

    // Add "\r\n"
    buffer[index++] = '\r';
    buffer[index++] = '\n';

    // Transmit the constructed message over UART
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, index, HAL_MAX_DELAY);
}

int FloatToStr(float value, char* buffer) {
    int i = 0;
    int isNegative = 0;

    // Handle negative numbers
    if (value < 0) {
        isNegative = 1;
        value = -value;
    }

    // Convert the integer part
    int int_part = (int)value;
    float decimal_part = value - int_part;

    // Convert the integer part to a string
    i += IntToStr(int_part, &buffer[i]);

    // Add the decimal point
    buffer[i++] = '.';

    // Convert the decimal part to two decimal places
    decimal_part *= 100;
    int decimal_int = (int)decimal_part;
    buffer[i++] = (decimal_int / 10) + '0';
    buffer[i++] = (decimal_int % 10) + '0';

    // Add negative sign if needed
    if (isNegative) {
        for (int j = i; j > 0; j--) {
            buffer[j] = buffer[j - 1];
        }
        buffer[0] = '-';
        i++;
    }

    // Null-terminate the string
    buffer[i] = '\0';

    // Return the length of the string
    return i;
}

int IntToStr(int16_t value, char* buffer) {
    int i = 0;
    int isNegative = 0;

    // Handle negative numbers
    if (value < 0) {
        isNegative = 1;
        value = -value;
    }

    // Convert the integer to a string (reverse order)
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

    // Null-terminate the string
    buffer[i] = '\0';

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
