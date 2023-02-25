/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "bme280.h"
#include "lwgps/lwgps.h"
#include "neo8m.h"
#include <math.h>
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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float RollAng, Altitude1, Temperature, Pressure, Humidity;
float acX, acY, acZ, gyX, gyY, gyZ, Alt, T, P, H; //Kalman
float gpsAlt, gpsLat, gpsLong;
unsigned char buff[54];
extern int flag;
int sayac = 0;

typedef union{
 float sayi;
 unsigned char array[4];
} float32toInt8;

lwgps_t gps;
extern uint8_t rx_data[1];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);

  lwgps_init(&gps);
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);

  BME280_Config(OSRS_2, OSRS_16, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16);

  unsigned char cs()
  {
	 int checkSum = 0;
	 for(int i=4; i<51; i++)
	 {
		 checkSum += buff[i];
	 }
	 return (unsigned char) (checkSum % 256);
  }

  void paket(int sayac)
  {
 	  buff[0] = 0xFF; //sabit
 	  buff[1] = 0xFF; //sabit
 	  buff[2] = 0x54; //sabit
 	  buff[3] = 0x52; //sabit
 	  buff[4] = 0x01; //id = 1 kabul ettik
 	  buff[5] = sayac; //paket sayaci

 	  float32toInt8 alt_donusturme;
 	  alt_donusturme.sayi = Alt;
 	  buff[6] = alt_donusturme.array[0];
 	  buff[7] = alt_donusturme.array[1];
 	  buff[8] = alt_donusturme.array[2];
 	  buff[9] = alt_donusturme.array[3];

 	  float32toInt8 gpsAltDonusturme;
 	  gpsAltDonusturme.sayi = gpsAlt;
 	  buff[10] = gpsAltDonusturme.array[0];
 	  buff[11] = gpsAltDonusturme.array[1];
 	  buff[12] = gpsAltDonusturme.array[2];
 	  buff[13] = gpsAltDonusturme.array[3];

 	  float32toInt8 gpsLatDonusturme;
 	  gpsLatDonusturme.sayi = gpsLat;
 	  buff[14] = gpsLatDonusturme.array[0];
 	  buff[15] = gpsLatDonusturme.array[1];
 	  buff[16] = gpsLatDonusturme.array[2];
 	  buff[17] = gpsLatDonusturme.array[3];

 	  float32toInt8 gpsLongDonusturme;
 	  gpsLongDonusturme.sayi = gpsLong;
 	  buff[18] = gpsLongDonusturme.array[0];
 	  buff[19] = gpsLongDonusturme.array[1];
 	  buff[20] = gpsLongDonusturme.array[2];
 	  buff[21] = gpsLongDonusturme.array[3];

 	  float32toInt8 AccX;
 	  AccX.sayi = acX;
 	  buff[22] = AccX.array[0];
 	  buff[23] = AccX.array[1];
 	  buff[24] = AccX.array[2];
 	  buff[25] = AccX.array[3];

 	  float32toInt8 AccY;
 	  AccY.sayi = acY;
 	  buff[26] = AccY.array[0];
 	  buff[27] = AccY.array[1];
 	  buff[28] = AccY.array[2];
 	  buff[29] = AccY.array[3];

 	  float32toInt8 AccZ;
 	  AccZ.sayi = acZ;
 	  buff[30] = AccZ.array[0];
 	  buff[31] = AccZ.array[1];
 	  buff[32] = AccZ.array[2];
 	  buff[33] = AccZ.array[3];

 	  float32toInt8 GyroX;
 	  GyroX.sayi = gyX;
 	  buff[34] = GyroX.array[0];
 	  buff[35] = GyroX.array[1];
 	  buff[36] = GyroX.array[2];
 	  buff[37] = GyroX.array[3];

 	  float32toInt8 GyroY;
 	  GyroY.sayi = gyY;
 	  buff[38] = GyroY.array[0];
 	  buff[39] = GyroY.array[1];
 	  buff[40] = GyroY.array[2];
 	  buff[41] = GyroY.array[3];

 	  float32toInt8 GyroZ;
 	  GyroZ.sayi = gyZ;
 	  buff[42] = GyroZ.array[0];
 	  buff[43] = GyroZ.array[1];
 	  buff[44] = GyroZ.array[2];
 	  buff[45] = GyroZ.array[3];

 	  float32toInt8 Aci;
 	  Aci.sayi = RollAng;
 	  buff[46] = Aci.array[0];
 	  buff[47] = Aci.array[1];
 	  buff[48] = Aci.array[2];
 	  buff[49] = Aci.array[3];

 	  buff[50] = flag; //durum
 	  buff[51] = cs(); //check-sum
 	  buff[52] = 0x0D; //sabit
 	  buff[53] = 0x0A; //sabit
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  acX = MPU6050_Kalman_Accel_X();
	  acY = MPU6050_Kalman_Accel_Y();
	  acZ = MPU6050_Kalman_Accel_Z();
	  gyX = MPU6050_Kalman_Gyro_X();
	  gyY = MPU6050_Kalman_Gyro_X();
	  gyZ = MPU6050_Kalman_Gyro_X();
	  RollAng = MPU6050_Kalman_Roll_Angle();
	  Temperature = BME280_Temperature();
	  Pressure = BME280_Pressure();
	  Humidity = BME280_Humidity();
	  T = BME280_Kalman_Temp(Temperature);
	  P = BME280_Kalman_Press(Pressure);
	  H = BME280_Kalman_Hum(Humidity);
	  Altitude1 = BME280_Altitude();
	  Alt = BME280_Kalman_Alt(Altitude1);
	  gpsAlt = gps.altitude;
	  gpsLat = gps.latitude;
	  gpsLong = gps.longitude;

	  paket(sayac);
	  sayac = sayac+1;

	  HAL_UART_Transmit(&huart1, buff, 54, 500);
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 35999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, first_Pin|second_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : first_Pin second_Pin */
  GPIO_InitStruct.Pin = first_Pin|second_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
