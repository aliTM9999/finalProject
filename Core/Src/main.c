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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kalman_c.h"
#define ARM_MATH_CM4
#include "arm_math.h"
#include <math.h>
#include "stm32l4s5i_iot01_magneto.h"  // LIS3MDL (magnetometer)
#include "stm32l4s5i_iot01_accelero.h" // LSM6DSL (accelerometer/gyroscope)
#include <string.h>
#include <stdio.h>


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
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

int16_t magnetometer_data[3];
int16_t accelerometer_data[3];
int16_t smoothed_accelerometer_data[3];
kalman_state stateX = {0.1, 0.1, 0.1, 5, 0};
kalman_state stateY = {0.1, 0.1, 0.1, 5, 0};
kalman_state stateZ = {0.1, 0.1, 0.1, 5, 0};
void Sensor_Read_with_Kalman(void);
const int BOARD_SIZE = 10;      // 10x10 board
const char CAR_SYMBOL = 'H';    // Car symbol
const char EMPTY_CELL = '.';    // Empty cell symbol
#define MAX_OBSTACLES 3      // Maximum number of obstacles
#define OBSTACLE_SYMBOL 'X'  // Symbol for obstacles

typedef struct {
    int x;  // X position
    int y;  // Y position
} Obstacle;

Obstacle obstacles[MAX_OBSTACLES];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//we chose counter period of 200, which gives us 600khz, because 120 Mhz/200=600kz (with 120 Mhz processor clock)
#define SINE_TABLE_SIZE 400//600khz/400 = 1.5khz
#define SINE_TABLE_SIZE2 500//600khz/500 =1.2khz
#define SINE_TABLE_SIZE3 600//600khz/600 =1khz
#define SINE_TABLE_SIZE4 600//300khz/600 =500 hz (here because counter period is modified to 400 rather than 200
#define amplitude 170// 2/3 of 256 max amplitude from 8-bit config
uint32_t sine_table[SINE_TABLE_SIZE];
uint32_t sine_table2[SINE_TABLE_SIZE2];
uint32_t sine_table3[SINE_TABLE_SIZE3];
uint32_t sine_table4[SINE_TABLE_SIZE4];

//function to fill needed arrays with samples of sine waves of different frequencies
void GenerateSineTable(void) {
	float32_t val=0;
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
    	val=(arm_sin_f32(2.0f * 3.141592653 * i / SINE_TABLE_SIZE) + 1.0f) * (amplitude / 2.0f);
        sine_table[i] = (uint32_t) val;
    }
    val=0.0f;
    for (int i = 0; i < SINE_TABLE_SIZE2; i++) {
		val=(arm_sin_f32(2.0f * 3.141592653 * i / SINE_TABLE_SIZE2) + 1.0f) * (amplitude / 2.0f);
		sine_table2[i] = (uint32_t) val;
	}
    val=0.0f;
	for (int i = 0; i < SINE_TABLE_SIZE3; i++) {
		val=(arm_sin_f32(2.0f * 3.141592653 * i / SINE_TABLE_SIZE3) + 1.0f) * (amplitude / 2.0f);
		sine_table3[i] = (uint32_t) val;
	}
	val=0.0f;
	for (int i = 0; i < SINE_TABLE_SIZE4; i++) {
		val=(arm_sin_f32(2.0f * 3.141592653 * i / SINE_TABLE_SIZE4) + 1.0f) * (amplitude / 2.0f);
		sine_table4[i] = (uint32_t) val;
	}
}
//function to read measurements of 5 sensors and send it over UART
void Sensor_Read_with_Kalman(void) {
	char buffer[100];  // Buffer to store the formatted string

	// Read sensor data
	//BSP_MAGNETO_GetXYZ(magnetometer_data);
	BSP_ACCELERO_AccGetXYZ(accelerometer_data);

	smoothed_accelerometer_data[0]=Kalmanfilter((float)accelerometer_data[0], &stateX);
	smoothed_accelerometer_data[1]=Kalmanfilter((float)accelerometer_data[1], &stateY);
	smoothed_accelerometer_data[2]=Kalmanfilter((float)accelerometer_data[2], &stateZ);

	sprintf(buffer, "Accelerometer: X=%d, Y=%d, Z=%d\r\n",
		accelerometer_data[0], accelerometer_data[1],
		accelerometer_data[2]);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer),
			HAL_MAX_DELAY);
	sprintf(buffer, "Smoothed Accelerometer: X=%d, Y=%d, Z=%d\r\n",
		smoothed_accelerometer_data[0], smoothed_accelerometer_data[1],
		smoothed_accelerometer_data[2]);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer),
			HAL_MAX_DELAY);


	HAL_UART_Transmit(&huart1, (uint8_t*) "\r\n", 2, HAL_MAX_DELAY); // Newline for clarity
}

void processAcceleration() {
    int16_t xAxisValue = smoothed_accelerometer_data[0]; // Read X-axis value
    int16_t yAxisValue = smoothed_accelerometer_data[1]; // Read Y-axis value

    // Calculate the total acceleration (resultant of X and Y)
    float totalAcceleration = sqrt((float)(xAxisValue * xAxisValue) + (float)(yAxisValue * yAxisValue));

    // Logic for sound generation based on total acceleration
    if (totalAcceleration < 500) {
        // Low acceleration: Play Sound 1
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sine_table, SINE_TABLE_SIZE, DAC_ALIGN_8B_R);
    } else if (totalAcceleration >= 300 && totalAcceleration < 600) {
        // Moderate acceleration: Play Sound 2
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sine_table2, SINE_TABLE_SIZE2, DAC_ALIGN_8B_R);
    } else if (totalAcceleration >= 600) {
        // High acceleration: Play Sound 3
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sine_table3, SINE_TABLE_SIZE3, DAC_ALIGN_8B_R);
    }
}
//// Function to map accelerometer coordinates to the board's grid
//void MapCoordinatesToBoard(int x, int y, char board[BOARD_SIZE][BOARD_SIZE]) {
//    // Clear the board
//    for (int i = 0; i < BOARD_SIZE; i++) {
//        for (int j = 0; j < BOARD_SIZE; j++) {
//            board[i][j] = EMPTY_CELL;
//        }
//    }
//    // Scale and map coordinates to the board
//    int mappedX = BOARD_SIZE / 2 + (x / 100);  // Adjust scaling factor (/100) as needed
//    int mappedY = BOARD_SIZE / 2 - (y / 100);  // Flip Y-axis
//    // Clamp the car position within board boundaries
//    if (mappedX < 0) mappedX = 0;
//    if (mappedX >= BOARD_SIZE) mappedX = BOARD_SIZE - 1;
//    if (mappedY < 0) mappedY = 0;
//    if (mappedY >= BOARD_SIZE) mappedY = BOARD_SIZE - 1;
//    // Place the car on the board
//    board[mappedY][mappedX] = CAR_SYMBOL;
//}

// Map accelerometer coordinates to board
void MapCoordinatesToBoard(int x, int y, char board[BOARD_SIZE][BOARD_SIZE], int *carX, int *carY) {
    // Clear the board
    for (int i = 0; i < BOARD_SIZE; i++) {
        for (int j = 0; j < BOARD_SIZE; j++) {
            board[i][j] = EMPTY_CELL;
        }
    }

    // Map coordinates to board
    *carX = BOARD_SIZE / 2 + (x / 100);
    *carY = BOARD_SIZE / 2 - (y / 100);

    // Clamp position to board boundaries
    if (*carX < 0) *carX = 0;
    if (*carX >= BOARD_SIZE) *carX = BOARD_SIZE - 1;
    if (*carY < 0) *carY = 0;
    if (*carY >= BOARD_SIZE) *carY = BOARD_SIZE - 1;

    // Place car on the board
    board[*carY][*carX] = CAR_SYMBOL;
}
// Function to transmit the 2D board to UART
void PrintBoardToUART(char board[BOARD_SIZE][BOARD_SIZE]) {
    char buffer[200];  // Buffer to store the board as a string
    int index = 0;
    // Convert the board to a string representation
    for (int i = 0; i < BOARD_SIZE; i++) {
        for (int j = 0; j < BOARD_SIZE; j++) {
            buffer[index++] = board[i][j];
        }
        buffer[index++] = '\r';  // Carriage return
        buffer[index++] = '\n';  // New line
    }
    buffer[index] = '\0';  // Null-terminate the string
    // Transmit the board over UART
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

// Initialize obstacles at random positions
void InitializeObstacles(void) {
    for (int i = 0; i < MAX_OBSTACLES; i++) {
        obstacles[i].x = rand() % BOARD_SIZE; // Random X position
        obstacles[i].y = rand() % BOARD_SIZE; // Random Y position
    }
}

// Move obstacles down and respawn at random X positions when out of bounds
void MoveObstacles(char board[BOARD_SIZE][BOARD_SIZE]) {
    for (int i = 0; i < MAX_OBSTACLES; i++) {
        // Clear current position
        board[obstacles[i].y][obstacles[i].x] = EMPTY_CELL;

        // Move obstacle down
        obstacles[i].y++;

        // If obstacle goes out of bounds, reset to the top
        if (obstacles[i].y >= BOARD_SIZE) {
            obstacles[i].y = 0;                  // Reset to top
            obstacles[i].x = rand() % BOARD_SIZE; // Randomize X position
        }

        // Place obstacle in the new position
        board[obstacles[i].y][obstacles[i].x] = OBSTACLE_SYMBOL;
    }
}

// Check for collisions between car and obstacles
int CheckCollision(int carX, int carY) {
    for (int i = 0; i < MAX_OBSTACLES; i++) {
        if (obstacles[i].x == carX && obstacles[i].y == carY) {
            return 1; // Collision detected
        }
    }
    return 0; // No collision
}
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  BSP_MAGNETO_Init();
  BSP_ACCELERO_Init();
  GenerateSineTable();
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

  HAL_TIM_Base_Start(&htim2);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)sine_table, SINE_TABLE_SIZE, DAC_ALIGN_8B_R);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char board[BOARD_SIZE][BOARD_SIZE];  // 2D board
  int carX, carY;
  int prevCarX = -1, prevCarY = -1;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Read accelerometer data and update smoothed values
	      Sensor_Read_with_Kalman();

	      // Map car position to board
	      MapCoordinatesToBoard(smoothed_accelerometer_data[0], smoothed_accelerometer_data[1], board, &carX, &carY);

	      // Play sound if the car moves
	          if (carX != prevCarX || carY != prevCarY) {
	              processAcceleration(); // Trigger sound
	          } else {
	              HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1); // Stop sound
	          }

	          // Update previous position
	          prevCarX = carX;
	          prevCarY = carY;
	      // Move obstacles and place on board
	      MoveObstacles(board);

	      // Check for collisions
	      if (CheckCollision(carX, carY)) {
	          HAL_UART_Transmit(&huart1, (uint8_t *)"Collision Detected!\r\n", 22, HAL_MAX_DELAY);
	          break; // Exit the game
	      }

	      // Print updated board to UART
	      PrintBoardToUART(board);
//	  processAcceleration();
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hi2c1.Init.Timing = 0x30A175AB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
