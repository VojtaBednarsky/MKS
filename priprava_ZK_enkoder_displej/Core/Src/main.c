/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sct.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BTN_PERIOD 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  sct_init();
  HAL_TIM_Encoder_Start(&htim1, htim1.Channel); // spustim TIM1 jako citac v rezimu ncoderu

  //sct_led(0x000F00FF);		// nakresli to nejaky znak na chvilku na displeji a potom to prejde do nekoncecne smycky
  //HAL_Delay(2000);			//short pause

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	  static uint32_t lastBTN = 0;
	  	  static uint16_t S1samp = 0xFFFF;
	  	  static uint16_t S2samp = 0xFFFF;
	  	  static uint8_t S1 = 0;
	  	  static uint8_t S2 = 0;
	  	  static uint32_t flag=0;

	  	  if(HAL_GetTick()>lastBTN+BTN_PERIOD){
	  		  lastBTN = HAL_GetTick();
	  		  S1samp = (S1samp<<1)+HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin);
	  		  S2samp = (S2samp<<1)+HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin);

	  	  if(S1samp == 0x8000) S1 = 1;
	  	  if(S2samp == 0x8000) S2 = 1;
	  	  }

	  if((S2 == 1)&(flag == 0)) {
	  			S2 = 0;
	  			sct_led(0x01000000);
	  			flag=1;
	  			__HAL_TIM_GET_COUNTER(&htim1) = 14;
	  			}
	  if((S1 == 1)&(flag == 1)) {
	 	  		S1 = 0;
	 	  		sct_led(0x00000000);
	 	  		flag=0;
	 	  		__HAL_TIM_GET_COUNTER(&htim1) = 12;
	 	  		}

	  if((S2 == 1)&(flag == 1)) {
	  	  		S2 = 0;
	  	  		sct_led(0x03000000);
	  	  		flag=2;
	  	  		__HAL_TIM_GET_COUNTER(&htim1) = 16;
	  	  		}
	  if((S1 == 1)&(flag == 2)) {
	  	 	  	S1 = 0;
	  	 	  	sct_led(0x01000000);
	  	 	  	flag=1;
	  	 	  	__HAL_TIM_GET_COUNTER(&htim1) = 14;
	  	 	  	}

	  if((S2 == 1)&(flag == 2)) {
	  	  	  	S2 = 0;
	  	  	  	sct_led(0x07000000);
	  	  	  	flag=3;
	  	  	  	__HAL_TIM_GET_COUNTER(&htim1) = 18;
	  	  	  	}
	  if((S1 == 1)&(flag == 3)) {
	  	  	 	  	S1 = 0;
	  	  	 	  	sct_led(0x03000000);
	  	  	 	  	flag=1;
	  	  	 	  	__HAL_TIM_GET_COUNTER(&htim1) = 16;
	  	  	 	  	}

	  if((S2 == 1)&(flag == 3)) {
	  	  	  	S2 = 0;
	  	  	  	sct_led(0x0F000000);
	  	  	  	flag=4;
	  	  	  	__HAL_TIM_GET_COUNTER(&htim1) = 20;
	  	  	  	}
	  if((S2 == 1)&(flag == 4)) {
	  	  	  	S2 = 0;
	  	  	  	sct_led(0x0F800000);
	  	  	  	flag=5;
	  	  	  	__HAL_TIM_GET_COUNTER(&htim1) = 22;
	  	  	  	}
	  if((S2 == 1)&(flag == 5)) {
	  	  	  	S2 = 0;
	  	  	  	sct_led(0x0FC00000);
	  	  	  	flag=6;
	  	  	  	__HAL_TIM_GET_COUNTER(&htim1) = 24;
	  	  	  	}
	  if((S2 == 1)&(flag == 6)) {
	  	  	  	S2 = 0;
	  	  	  	sct_led(0x0FE00000);
	  	  	  	flag=7;
	  	  	  	__HAL_TIM_GET_COUNTER(&htim1) = 26;
	  	  	  	}
	  if((S2 == 1)&(flag == 7)) {
	  	  	  	S2 = 0;
	  	  	  	sct_led(0x0FF00000);
	  	  	  	flag=8;
	  	  	  	__HAL_TIM_GET_COUNTER(&htim1) = 28;
	  	  	  	}

	  if(__HAL_TIM_GET_COUNTER(&htim1) < 13) {
	  		  	  sct_led(0x00000000);
	  		  	  flag=0;
	  	  }
	  if(__HAL_TIM_GET_COUNTER(&htim1) == 14) {
	  	  		  sct_led(0x01000000);
	  	  		  flag=1;
	  	  }
	  if(__HAL_TIM_GET_COUNTER(&htim1) == 16) {
	  	  		  sct_led(0x03000000);
	  	  		  flag=2;
	  	  }
	  if(__HAL_TIM_GET_COUNTER(&htim1) == 18) {
	  	  		  sct_led(0x07000000);
	  	  		  flag=3;
	  	  }
	  if(__HAL_TIM_GET_COUNTER(&htim1) == 20) {
	  	  		  sct_led(0x0F000000);
	  	  		  flag=4;
	  	  }
	  if(__HAL_TIM_GET_COUNTER(&htim1) == 22) {
	  	  		  sct_led(0x0F800000);
	  	  		  flag=5;
	  	  }
	  if(__HAL_TIM_GET_COUNTER(&htim1) == 24) {
	  	  	  	  sct_led(0x0FC00000);
	  	  	  	  flag=6;
	  	  }
	  if(__HAL_TIM_GET_COUNTER(&htim1) == 26) {
	  	  	  	  sct_led(0x0FE00000);
	  	  	  	  flag=7;
	  	  }
	  if(__HAL_TIM_GET_COUNTER(&htim1) > 27) {
	  	  		  sct_led(0x0FF00000);
	  	  		  flag=8;
	  	  }
	  HAL_Delay(50);
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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SCT_NOE_Pin|SCT_CLK_Pin|SCT_SDI_Pin|SCT_NLA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S2_Pin S1_Pin */
  GPIO_InitStruct.Pin = S2_Pin|S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCT_NOE_Pin SCT_CLK_Pin SCT_SDI_Pin SCT_NLA_Pin */
  GPIO_InitStruct.Pin = SCT_NOE_Pin|SCT_CLK_Pin|SCT_SDI_Pin|SCT_NLA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
