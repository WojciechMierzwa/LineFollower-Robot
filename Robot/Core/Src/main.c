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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/*Variables for object detection*/
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance  = 0;  // cm

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void detect(uint8_t *detect_states);
int detectObstacle(void);
void motor_forward(uint32_t duty_cycle);
void motor_backward(uint32_t duty_cycle);
void motor_stop(void);
void motor_left(uint32_t duty_cycle);
void motor_right(uint32_t duty_cycle);


/*display*/
void display_reset(void);
void display1(void);
void display2(void);
void display3(void);
void display4(void);
void countdown(void);


#define TIM_NO htim16
#define TIM_CH_NO TIM_CHANNEL_1

/* zakresy katowe pracy serwomechanizmu */
#define ANGLE_MIN 0
#define ANGLE_MAX 900
/* zakres PWM */
#define PWM_MIN 1100
#define PWM_MAX 2050

#define STEP ((1000 * (PWM_MAX - PWM_MIN)) / (ANGLE_MAX - ANGLE_MIN))

void set_ang(uint16_t ang, uint8_t mode);
void turnover(uint16_t *axle, uint8_t receivedChar);
void detectMotor(void);
void bluetooth(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t detect_states[5];
uint16_t axle=0;
uint8_t receivedChar;
uint32_t cycle=32768;
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);
  HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //set_ang(0, 0);

  uint16_t counter=1;
  uint16_t distance;

  /**/
  uint8_t random_number;
  srand(time(NULL));

  while (1)
  {
	  bluetooth(); //  testowane bez trybu, wrzucilem do 3

	  if(counter>4){
	  	        	counter=1;
	  	        }
	  if (HAL_GPIO_ReadPin(ButtonExt_GPIO_Port, ButtonExt_Pin) == GPIO_PIN_SET)
	      {
	        // If the button is pressed, increment the counter
	        counter++;
	        HAL_Delay(500);

	      }

	  	  display_reset();
	      // Perform different actions based on the counter value
	      switch (counter)
	      {
	      case 1:
	        display_reset();
	        display1();
	        if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
	        	      {
	        	        // If the button is pressed, increment the counter
	        	        countdown();
	        	        while(1){
	        	        	/*line follower case 1*/

	        	        	 	 	 detect_states[0] = HAL_GPIO_ReadPin(DETECT1_GPIO_Port, DETECT1_Pin);
	        	        		      detect_states[1] = HAL_GPIO_ReadPin(DETECT2_GPIO_Port, DETECT2_Pin);
	        	        		      detect_states[2] = HAL_GPIO_ReadPin(DETECT3_GPIO_Port, DETECT3_Pin);
	        	        		      detect_states[3] = HAL_GPIO_ReadPin(DETECT4_GPIO_Port, DETECT4_Pin);
	        	        		      detect_states[4] = HAL_GPIO_ReadPin(DETECT5_GPIO_Port, DETECT5_Pin);
	        	        		  if(detect_states[2]==0)
	        	        		  		  	  {

	        	        		  		  		motor_forward(cycle);
	        	        		  		  	  }
	        	        		  		  	  else{
	        	        		  		  		  if(detect_states[1]==0 || detect_states[0]==0){
	        	        		  		  			motor_right(cycle);
	        	        		  		  		  }
	        	        		  		  		  else if(detect_states[3]==0 || detect_states[4]==0){
	        	        		  		  			motor_left(cycle);
	        	        		  		  		  		  }
	        	        		  		  		  else{
	        	        		  		  			motor_backward(cycle);
	        	        		  		  		  }

	        	        }

	        	      }
	        	      }

	        break;
	      case 2:
	        display_reset();
	        display2();
	        if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET){
	        	countdown();
	        	display_reset();
	        	while(1){
	        		  Distance = detectObstacle();
	        		             random_number = rand() % 2;
	        		             motor_forward(cycle);

	        		             while(Distance < 10){
	        		                 if(Distance < 5){
	        		                     do{
	        		                         motor_backward(cycle);
	        		                         Distance = detectObstacle();
	        		                     } while(Distance < 20);
	        		                 }
	        		                 if(Distance <= 15){
	        		                     if(random_number == 0){
	        		                         motor_left(cycle);
	        		                     } else {
	        		                         motor_right(cycle);
	        		                     }
	        		                 } else {
	        		                     // Move forward if the distance is greater than 15 cm
	        		                     motor_forward(cycle);
	        		                 }

	        		                 // Update distance for the next iteration
	        		                 Distance = detectObstacle();
	        		             }
	        	}
	        }
	        break;
	      case 3:
	        display_reset();
	        display3();
	        bluetooth();
	        break;
	      case 4:
	        display_reset();
	        display4();
	        break;
	      default:
	        // Do nothing for other counter values
	        break;
	      }



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HAL_UART_Receive(&huart2, &receivedChar,1 ,HAL_MAX_DELAY);


	/*	  	  if(detect_states[2]==0)
		  	  {
		  		  receivedChar='w';
		  	  }
		  	  else{
		  		  if(detect_states[1]==0 || detect_states[0]==0){
		  			  receivedChar='a';
		  		  }
		  		  else if(detect_states[3]==0 || detect_states[4]==0){
		  		  			  receivedChar='d';
		  		  		  }
		  		  else{
		  			  receivedChar='s';
		  		  }
*/


		  	  	 /* if(receivedChar == 's')
		  	  	  	  	  	      {
		  	  	  	  	  	    		  motor_backward(cycle);


		  	  	  	  	  	      }
		  	  	  	  	  	      else if (receivedChar == 'w')
		  	  	  	  	  	      {

		  	  	  	  	  	        motor_forward(cycle);
		  	  	  	  	  	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		  	  	  	  	  	        buttonPressed = 1;
		  	  	  	  	  	      }
		  	  	  	  	  	      else if(receivedChar == 'a'){
		  	  	  	  	  	    	  motor_left(cycle);
		  	  	  	  	  	      }
		  	  	  	  	  	      else if(receivedChar == 'd'){
		  	  	  	  	  	    		  motor_right(cycle);
		  	  	  	  	  	      }
		  	  	  	  	  	      else if(receivedChar == 'q'){
		  	  	  	  	  	    	  motor_stop();
		  	  	  	  	  	      }
	  }*/



	  //detectObstacle();
	  //turnover(&axle, receivedChar); - testy
	  //detectMotor();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM16;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 71;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 19999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TRIGGER_Pin|C_Pin|B_Pin|A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|E_Pin|D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, G_Pin|F_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ECHO_Pin DETECT5_Pin DETECT1_Pin DETECT2_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin|DETECT5_Pin|DETECT1_Pin|DETECT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIGGER_Pin C_Pin B_Pin A_Pin */
  GPIO_InitStruct.Pin = TRIGGER_Pin|C_Pin|B_Pin|A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin E_Pin D_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|E_Pin|D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ButtonExt_Pin DETECT4_Pin */
  GPIO_InitStruct.Pin = ButtonExt_Pin|DETECT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : G_Pin F_Pin LD2_Pin */
  GPIO_InitStruct.Pin = G_Pin|F_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DETECT3_Pin */
  GPIO_InitStruct.Pin = DETECT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DETECT3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int detectObstacle(void){
	int samples[NUM_SAMPLES];
	int totalDistance = 0;
	for (int i = 0; i < NUM_SAMPLES; i++) {
    HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
    __HAL_TIM_SET_COUNTER(&htim16, 0);
    while (__HAL_TIM_GET_COUNTER (&htim16) < 10);  // wait for 10 us
    HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);  // pull the TRIG p
    pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
    // wait for the echo pin to go high
    while (!(HAL_GPIO_ReadPin (ECHO_GPIO_Port, ECHO_Pin)) && pMillis + 10 >  HAL_GetTick());
    Value1 = __HAL_TIM_GET_COUNTER (&htim16);

    pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
    // wait for the echo pin to go low
    while ((HAL_GPIO_ReadPin (ECHO_GPIO_Port, ECHO_Pin)) && pMillis + 50 > HAL_GetTick());
    Value2 = __HAL_TIM_GET_COUNTER (&htim16);

    samples[i] = (Value2-Value1) /58;

	}
    for (int i = 0; i < NUM_SAMPLES; i++) {
            totalDistance += samples[i];
        }
    return (totalDistance/NUM_SAMPLES)*2;
}


void bluetooth(void) { // obsługa sterowania poprzez moduł bluetooth zs-040/hc-06

    if (HAL_UART_Receive(&huart1, &receivedChar, 1, 0) == HAL_OK) {
        if (receivedChar == 's') {
            motor_backward(cycle);
        } else if (receivedChar == 'w') {
            motor_forward(cycle);
        } else if (receivedChar == 'a') {
            motor_left(cycle);
        } else if (receivedChar == 'd') {
            motor_right(cycle);
        } else if (receivedChar == 'q') {
            motor_stop();
        }
    }
    HAL_UART_Receive(&huart1, &receivedChar, 1,0);
}

void turnover(uint16_t *axle, uint8_t receivedChar){
	while ((*axle)> 0)
		  {
		      // Sprawdzenie dostępności danych w strumieniu UART
		      if (HAL_UART_Receive(&huart2, &receivedChar, 1, 0) == HAL_OK)
		      {
		          // Obsługa odebranego znaku
		          if (receivedChar == 'a')
		          {
		              // Przerwanie pętli i obsługa znaku 'a'
		              break;
		          }
		          if(receivedChar == 'd');
		          else{
		        	  goto hell;
		          }
		      }
		      // Aktualizacja wartości axle
		      (*axle)--;
		      set_ang(*axle, 0);
		      HAL_Delay(0.1);
		  }

		  // Pętla zwiększająca wartość axle
		  while ((*axle) < 900)
		  {
		      // Sprawdzenie dostępności danych w strumieniu UART
		      if (HAL_UART_Receive(&huart2, &receivedChar, 1, 0) == HAL_OK)
		      {
		          // Obsługa odebranego znaku
		          if (receivedChar == 'd')
		          {
		              // Przerwanie pętli i obsługa znaku 'd'
		              break;
		          }
		          if(receivedChar == 'a');
		          else{
		        	  goto hell;
		          }
		      }
		      // Aktualizacja wartości axle
		      (*axle)++;
		      set_ang(*axle, 0);
		      HAL_Delay(0.1);
		  }

		  // Pętla ustawiająca wartość axle na 450
		  while ((*axle) != 450)
		  {
		      // Sprawdzenie dostępności danych w strumieniu UART
		      if (HAL_UART_Receive(&huart2, &receivedChar, 1, 0) == HAL_OK)
		      {
		          // Obsługa odebranego znaku
		          if (receivedChar == 'a' || receivedChar == 'd')
		          {
		              // Przerwanie pętli i obsługa znaku 'a' lub 'd'
		              break;
		          }
		      }
		      // Aktualizacja wartości axle
		      if ((*axle) < 450)
		      {
		          (*axle)++;
		      }
		      if ((*axle) > 450)
		      {
		          (*axle)--;
		      }
		      set_ang(*axle, 0);
		      HAL_Delay(0.1);

		      hell: set_ang(450, 0);
		  }
}
void set_ang(uint16_t ang, uint8_t mode)
{
	uint16_t val;

	if(ang > ANGLE_MAX)
	{
		ang = ANGLE_MAX;
	}
	else if (ang < ANGLE_MIN)
	{
		ang = ANGLE_MIN;
	}

	if(mode)
	{
		val = PWM_MIN + ((ang - ANGLE_MIN) * STEP) / 1000;
	}
	else
	{
		val = PWM_MAX - ((ang - ANGLE_MIN) * STEP) / 1000;
	}

	__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_NO, val);
}

void detectMotor(void){ // obrot serva w zależności od detekcji
    detect(detect_states);

    (detect_states[0] != 0 && detect_states[1] != 0) ? turnover(&axle, 'a') :
    (detect_states[2] != 0 && detect_states[3] != 0 && detect_states[4] != 0) ? turnover(&axle, 'd') : 0;
}

void motor_forward(uint32_t duty_cycle)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle*3/4);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
}
void motor_left(uint32_t duty_cycle)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty_cycle/5);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
}
void motor_right(uint32_t duty_cycle)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty_cycle/5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
}

// Function to move motors backward
void motor_backward(uint32_t duty_cycle)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty_cycle);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty_cycle*3/4);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
}

// Function to stop motors
void motor_stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
}
void display1(void){
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
}
void display2(void){
	//2
		  HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);

}
void display3(void){
	//3
		  HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
		  	  HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
		  	  HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
		  	  HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
		  	  HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
}
void display4(void){
	//4
		  	HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
		  	HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
		  	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
		  	HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
}
void display_reset(void){
	HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);

}

void countdown(void){
	display_reset();
	display3();
	HAL_Delay(1000);
	display_reset();
	display2();
	HAL_Delay(1000);
	display_reset();
	display1();
	HAL_Delay(1000);
	display_reset();
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
