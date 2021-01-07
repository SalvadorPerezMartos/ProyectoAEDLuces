/* USER CODE BEGIN Header */
/**
 *
 * PLACA3
 *
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
ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t RxBuffer;
uint8_t numero_efecto;
uint8_t efecto_on;
uint8_t cont=0;

uint8_t mens_tx;
int alarma=0;

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
//  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_Base_Start(&htim7);

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  return 0;
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.RxMode = ETH_RXPOLLING_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */
    
  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  hi2c1.Init.OwnAddress1 = 2;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
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
  hi2c1.Init.ClockSpeed = 400000;
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 299;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 14000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 2999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 28000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LED1_D8_Pin|LED4_D4_Pin|LED5_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED2_D6_Pin|LED3_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Bt_USER_Pin */
  GPIO_InitStruct.Pin = Bt_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Bt_USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_D8_Pin LED4_D4_Pin LED5_D2_Pin */
  GPIO_InitStruct.Pin = LED1_D8_Pin|LED4_D4_Pin|LED5_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_D6_Pin LED3_D5_Pin */
  GPIO_InitStruct.Pin = LED2_D6_Pin|LED3_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == GPIO_PIN_13){ //Alarma USER BUTTON

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		alarma=1;
	}

}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){

	uint8_t mens_rx;

	if(HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t *)mens_rx, sizeof(mens_rx)) != HAL_OK)
		{
			/* Transfer error in transmission process */
			Error_Handler();
		}

	if (mens_rx==0x00){ //apagar led1
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_RESET);
		efecto_on=0;
		alarma=0;
	}
	else if(mens_rx==0x20){ //encender led1
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_SET);
		efecto_on=0;
		alarma=0;
	}
	else if(mens_rx==0x04){ //apagar led2
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_RESET);
			efecto_on=0;
			alarma=0;
		}
	else if(mens_rx==0x24){ //encender led2
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_SET);
			efecto_on=0;
			alarma=0;
		}
	else if(mens_rx==0x08){ //apagar led3
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);
			efecto_on=0;
			alarma=0;
		}
	else if(mens_rx==0x28){ //encender led3
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_SET);
			efecto_on=0;
			alarma=0;
		}
	else if(mens_rx==0x0C){ //apagar led4
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
			efecto_on=0;
			alarma=0;
		}
	else if(mens_rx==0x2C){ //encender led4
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_SET);
			efecto_on=0;
			alarma=0;
		}
	else if(mens_rx==0x10){ //apagar led5
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
			efecto_on=0;
			alarma=0;
		}
	else if(mens_rx==0x30){ //encender led5
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_SET);
			efecto_on=0;
			alarma=0;
		}
	else if(mens_rx==0x01){ //rafaga 0
			efecto_on=1;
			numero_efecto=0;
			alarma=0;
		}
	else if(mens_rx==0x05){ //rafaga 1
			efecto_on=1;
			numero_efecto=1;
			alarma=0;
		}
	else if(mens_rx==0x09){ //rafaga 2
			efecto_on=1;
			numero_efecto=2;
			alarma=0;
		}
	else if(mens_rx==0x0D){ //rafaga 3
			efecto_on=1;
			numero_efecto=3;
			alarma=0;
		}
	else if(mens_rx==0x03){ //ALARMA
			efecto_on=0;
			alarma=1;

			}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim->Instance == TIM6){
		if(alarma==1){
			mens_tx=0x03; //transmitir alarma
			if(HAL_I2C_Slave_Transmit_IT(&hi2c1, (uint8_t *)mens_tx, sizeof(mens_tx)) != HAL_OK){

				/* Transfer error in transmission process */
				Error_Handler();
			}
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

			alarma==0;

		} else if(alarma==0){
			mens_tx=0x00;
		}
	} else if(htim->Instance == TIM7){
		if (efecto_on==1){
			switch (numero_efecto){
			case 0:				//numero_efecto 0
				switch (cont){
				case 0:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(500);
					cont++;
					break;

				case 1:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(500);
					cont++;
					break;

				case 2:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_SET);
					HAL_Delay(500);
					cont++;
					break;

				case 3:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(500);
					cont++;
					break;

				case 4:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(500);
					cont++;
					break;

				case 5:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(500);
					cont=0;
					break;
				}
				break;

			case 1:		//numero_efecto 1
				switch (cont){

				case 0:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(500);
					cont++;
				break;

				case 1:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(500);
					cont++;
				break;

				case 2:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(500);
					cont++;
					break;

				case 3:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(500);
					cont++;
					break;

				case 4:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(500);
					cont++;
					break;

				case 5:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(500);
					cont=0;
					break;


				}
				break;

			case 2:		//numero_efectoO 2
				switch (cont){

				case 0:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(500);
					cont++;
					break;

				case 1:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(500);
					cont++;
					break;

				case 2:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(500);
					cont++;
					break;

				case 3:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(500);
					cont++;
					break;

				case 4:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_SET);
					HAL_Delay(500);
					cont++;
					break;

				case 5:
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(500);
					cont=0;
					break;

				}
				break;

			case 3:		//numero_efecto 3
				switch (cont){

				case 0:
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_SET);
				HAL_Delay(500);
				cont++;
				break;

				case 1:
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
				HAL_Delay(500);
				cont++;
				break;

				case 2:
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_SET);
				HAL_Delay(500);
				cont++;
				break;

				case 3:
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
				HAL_Delay(500);
				cont++;
				break;

				case 4:
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_SET);
				HAL_Delay(500);
				cont++;
				break;

				case 5:
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15, GPIO_PIN_RESET);
				HAL_Delay(500);
				cont=0;
				break;

				}
				break;





			}
		}

	}



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
	while(1);

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
