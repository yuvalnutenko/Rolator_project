/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define SONAR1_ECHO_Pin GPIO_PIN_7 //left
#define SONAR1_ECHO_GPIO_Port GPIOC
#define SONAR1_TRIG_Pin GPIO_PIN_9
#define SONAR1_TRIG_GPIO_Port GPIOA

//
#define SONAR2_TRIG_Pin GPIO_PIN_6 //right
#define SONAR2_TRIG_GPIO_Port GPIOA
#define SONAR2_ECHO_Pin GPIO_PIN_7
#define SONAR2_ECHO_GPIO_Port GPIOA
//
#define SONAR3_TRIG_Pin GPIO_PIN_10 //front
#define SONAR3_TRIG_GPIO_Port GPIOB
#define SONAR3_ECHO_Pin GPIO_PIN_6
#define SONAR3_ECHO_GPIO_Port GPIOB
//
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
//
#define Right_Vibration_Pin GPIO_PIN_8 //Left_Vibration_Pin
#define Right_Vibration_GPIO_Port GPIOB //Left_Vibration_GPIO_Port
//
#define Left_Vibration_Pin GPIO_PIN_9
#define Left_Vibration_GPIO_Port GPIOB
//
#define sound_Pin GPIO_PIN_8
#define sound_GPIO_Port GPIOA
//
uint32_t state = 1;
uint32_t Distance  = 0;
char messageSend[512];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void print_to_screen(uint32_t value, uint32_t sensor_num){
	 sprintf(messageSend, "distance: %d, sensor_num = %d\r\n", value, sensor_num);
	 HAL_UART_Transmit(&huart2, (uint8_t*) messageSend,strlen(messageSend), 5000);  // send json packet
}

uint32_t Sensor_read(GPIO_TypeDef* TRIG_Port, uint16_t TRIG_Pin, GPIO_TypeDef* ECHO_Port, uint16_t ECHO_Pin)
{
	uint32_t Value1 = 0;
	uint32_t Value2 = 0;
	uint32_t distance  = 0;
     HAL_GPIO_WritePin(TRIG_Port, TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
 	 __HAL_TIM_SET_COUNTER(&htim1, 0);
 	 while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
 	 HAL_GPIO_WritePin(TRIG_Port, TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

 	 while (!(HAL_GPIO_ReadPin (ECHO_Port, ECHO_Pin)));
 	 Value1 = __HAL_TIM_GET_COUNTER (&htim1);
 	 // wait for the echo pin to go low
 	 while ((HAL_GPIO_ReadPin (ECHO_Port, ECHO_Pin)));
 	 Value2 = __HAL_TIM_GET_COUNTER (&htim1);

 	 distance = (Value2-Value1)/58;
 	 HAL_Delay(10);
 	 return distance;
}

// Sets the vibration on the left for number of times we want with option for how long it would vibrate
void alert_left(uint16_t number, uint16_t time_up, uint16_t time_down, uint16_t time_up_sound){
	for(uint16_t i=0; i < number; i++)
	{
		HAL_GPIO_WritePin(Left_Vibration_GPIO_Port, Left_Vibration_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sound_GPIO_Port, sound_Pin, GPIO_PIN_SET);
		HAL_Delay(time_up_sound);
		HAL_GPIO_WritePin(sound_GPIO_Port, sound_Pin, GPIO_PIN_RESET);
		HAL_Delay(time_up-time_up_sound);
		HAL_GPIO_WritePin(Left_Vibration_GPIO_Port, Left_Vibration_Pin, GPIO_PIN_RESET);
		HAL_Delay(time_down);
	}
}

// Sets the vibration on the right for number of times we want with option for how long it would vibrate
void alert_right(uint16_t number, uint16_t time_up, uint16_t time_down, uint16_t time_up_sound){
	for(uint16_t i=0; i < number; i++)
	{
		HAL_GPIO_WritePin(Right_Vibration_GPIO_Port, Right_Vibration_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sound_GPIO_Port, sound_Pin, GPIO_PIN_SET);
		HAL_Delay(time_up_sound);
		HAL_GPIO_WritePin(sound_GPIO_Port, sound_Pin, GPIO_PIN_RESET);
		HAL_Delay(time_up-time_up_sound);
		HAL_GPIO_WritePin(Right_Vibration_GPIO_Port, Right_Vibration_Pin, GPIO_PIN_RESET);
		HAL_Delay(time_down);
	}
}

// Sets the vibration on the left and right and the light for number of times we want with option for how long it would vibrate
void alert_front(uint16_t number, uint16_t time_up, uint16_t time_down, uint16_t time_up_sound){
	for(uint16_t i=0; i < number; i++)
	{
		HAL_GPIO_WritePin(Left_Vibration_GPIO_Port, Left_Vibration_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Right_Vibration_GPIO_Port, Right_Vibration_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sound_GPIO_Port, sound_Pin, GPIO_PIN_SET);
		HAL_Delay(time_up_sound);
		HAL_GPIO_WritePin(sound_GPIO_Port, sound_Pin, GPIO_PIN_RESET);
		HAL_Delay(time_up-time_up_sound);
		HAL_GPIO_WritePin(Right_Vibration_GPIO_Port, Right_Vibration_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Left_Vibration_GPIO_Port, Left_Vibration_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		HAL_Delay(time_down);
	}
}

void test(GPIO_TypeDef* TRIG_Port, uint16_t TRIG_Pin, GPIO_TypeDef* ECHO_Port, uint16_t ECHO_Pin)
{
	uint32_t value = 0;
	for (uint16_t i = 0 ; i < 50 ; i++){
		value = value + Sensor_read(TRIG_Port, TRIG_Pin, ECHO_Port, ECHO_Pin);
	}
	print_to_screen(value/50, 0);
}

// Fills up an array with sensor reads and then sorts them and returns the median
uint32_t mead_distance(GPIO_TypeDef* TRIG_Port, uint16_t TRIG_Pin, GPIO_TypeDef* ECHO_Port, uint16_t ECHO_Pin)
{
	//Fill an array with 10 reads from the sensor
	uint32_t array_find_mead[11];
	uint32_t dist = 0;
	for (uint32_t i=0; i<11; i++){
		dist = Sensor_read(TRIG_Port, TRIG_Pin, ECHO_Port, ECHO_Pin);
		array_find_mead[i]=dist;
	}

	//Sort the array in ascending order
	for(int i =0; i<11; i++)
	{
		int key = array_find_mead[i];
		int j = i -1;

		while( j>= 0 && array_find_mead[j] > key)
		{
			array_find_mead[j+1] = array_find_mead[j];
			j--;
		}
		array_find_mead[j+1] = key;
	}

	//return the median value
	return array_find_mead[5];

}

uint32_t flow(uint32_t status)
{
	switch(status)
	{
		case 1: //Base state
			state = 1;
			Distance = mead_distance(SONAR3_TRIG_GPIO_Port, SONAR3_TRIG_Pin, SONAR3_ECHO_GPIO_Port, SONAR3_ECHO_Pin); //read_front
			if(Distance < 110 && Distance > 70)
			{
				alert_front(2, 250, 200, 40);
				break;
			}
			else if(Distance <= 70)
			{
				alert_front(3, 250, 200, 40);
				break;
			}
			Distance = mead_distance(SONAR2_TRIG_GPIO_Port, SONAR2_TRIG_Pin, SONAR2_ECHO_GPIO_Port, SONAR2_ECHO_Pin); //read_right
			if(Distance < 70)
						{
							alert_right(2, 250, 200, 20);
							state = 2;
							break;
						}
			Distance = mead_distance(SONAR1_TRIG_GPIO_Port, SONAR1_TRIG_Pin, SONAR1_ECHO_GPIO_Port, SONAR1_ECHO_Pin); //read_left
			if(Distance < 70)
						{
							alert_left(2, 250, 200, 20);
							state = 3;
							break;
						}
			break;
		case 2: // Follow right wall
			state = 2;
			Distance = mead_distance(SONAR3_TRIG_GPIO_Port, SONAR3_TRIG_Pin, SONAR3_ECHO_GPIO_Port, SONAR3_ECHO_Pin); //read_front
			if(Distance < 65)
			{
				alert_front(2, 250, 200, 40);
				break;
			}
			Distance = mead_distance(SONAR1_TRIG_GPIO_Port, SONAR1_TRIG_Pin, SONAR1_ECHO_GPIO_Port, SONAR1_ECHO_Pin); //read_left
			if(Distance < 50)
			{
				alert_left(2, 250, 200, 20);
				state = 4;
				break;
			}
			Distance = mead_distance(SONAR2_TRIG_GPIO_Port, SONAR2_TRIG_Pin, SONAR2_ECHO_GPIO_Port, SONAR2_ECHO_Pin); //read_right
			if(Distance < 50)
			{
				alert_right(3, 275, 180, 40);
				break;
			}
			Distance = mead_distance(SONAR2_TRIG_GPIO_Port, SONAR2_TRIG_Pin, SONAR2_ECHO_GPIO_Port, SONAR2_ECHO_Pin); //read_right
			if(Distance > 50)
			{
				state = 1;
				break;
			}
			break;
		case 3: //follow left wall
			state = 3;
			Distance = mead_distance(SONAR3_TRIG_GPIO_Port, SONAR3_TRIG_Pin, SONAR3_ECHO_GPIO_Port, SONAR3_ECHO_Pin); //read_front
			if(Distance < 65)
			{
				alert_front(2, 250, 200, 40);
				break;
			}
			Distance = mead_distance(SONAR2_TRIG_GPIO_Port, SONAR2_TRIG_Pin, SONAR2_ECHO_GPIO_Port, SONAR2_ECHO_Pin); //read_right
			if(Distance < 50)
			{
				alert_right(2, 250, 200, 20);
				state = 4;
				break;
			}
			Distance = mead_distance(SONAR1_TRIG_GPIO_Port, SONAR1_TRIG_Pin, SONAR1_ECHO_GPIO_Port, SONAR1_ECHO_Pin); //read_left
			if(Distance < 50)
			{
				alert_left(3, 275, 180, 40);
				break;
			}
			Distance = mead_distance(SONAR1_TRIG_GPIO_Port, SONAR1_TRIG_Pin, SONAR1_ECHO_GPIO_Port, SONAR1_ECHO_Pin); //read_left
			if(Distance > 50)
			{
				state = 1;
				break;
			}
			break;
		case 4:
			state = 4;
			Distance = mead_distance(SONAR3_TRIG_GPIO_Port, SONAR3_TRIG_Pin, SONAR3_ECHO_GPIO_Port, SONAR3_ECHO_Pin); //read_front
			if(Distance < 65)
			{
				alert_front(2, 250, 200, 40);
				break;
			}
			Distance = mead_distance(SONAR1_TRIG_GPIO_Port, SONAR1_TRIG_Pin, SONAR1_ECHO_GPIO_Port, SONAR1_ECHO_Pin); //read_left
			if(Distance < 50)
			{
				alert_left(3, 275, 180, 40);
				break;
			}
			Distance = mead_distance(SONAR2_TRIG_GPIO_Port, SONAR2_TRIG_Pin, SONAR2_ECHO_GPIO_Port, SONAR2_ECHO_Pin); //read_right
			if(Distance < 50)
			{
				alert_right(3, 275, 180, 40);
				break;
			}
			Distance = mead_distance(SONAR1_TRIG_GPIO_Port, SONAR1_TRIG_Pin, SONAR1_ECHO_GPIO_Port, SONAR1_ECHO_Pin); //read_left
			if(Distance > 50)
			{
				state = 2;
				break;
			}
			Distance = mead_distance(SONAR2_TRIG_GPIO_Port, SONAR2_TRIG_Pin, SONAR2_ECHO_GPIO_Port, SONAR2_ECHO_Pin); //read_right
			if(Distance > 50)
			{
				state = 3;
				break;
			}
			break;
	}
	return state;
}
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_GPIO_WritePin(SONAR1_TRIG_GPIO_Port, SONAR1_TRIG_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SONAR2_TRIG_GPIO_Port, SONAR2_TRIG_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SONAR3_TRIG_GPIO_Port, SONAR3_TRIG_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */
	  state = flow(state);
//	  Distance = mead_distance(SONAR3_TRIG_GPIO_Port, SONAR3_TRIG_Pin, SONAR3_ECHO_GPIO_Port, SONAR3_ECHO_Pin); //read_left
//	  if(Distance < 40)
//	  {
//		  alert_front(2, 250, 200, 20);
//	  }
	  //HAL_Delay(100);
	  //print_to_screen(Distance, state);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|SONAR2_TRIG_Pin|sound_Pin|SONAR1_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SONAR3_TRIG_Pin|Left_Vibration_Pin|Right_Vibration_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD2_Pin SONAR2_TRIG_Pin sound_Pin SONAR1_TRIG_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|SONAR2_TRIG_Pin|sound_Pin|SONAR1_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SONAR2_ECHO_Pin */
  GPIO_InitStruct.Pin = SONAR2_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SONAR2_ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SONAR3_TRIG_Pin Left_Vibration_Pin Right_Vibration_Pin */
  GPIO_InitStruct.Pin = SONAR3_TRIG_Pin|Left_Vibration_Pin|Right_Vibration_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SONAR1_ECHO_Pin */
  GPIO_InitStruct.Pin = SONAR1_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SONAR1_ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SONAR3_ECHO_Pin */
  GPIO_InitStruct.Pin = SONAR3_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SONAR3_ECHO_GPIO_Port, &GPIO_InitStruct);

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
