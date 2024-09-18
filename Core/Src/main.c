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
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_SIZE 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
char teste[32];
float umidade = 0.0f;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint32_t adc_buffer[ADC_BUF_SIZE];
uint8_t adc_ready = 0;
int currentItem = 0;
int8_t option = 0;
uint8_t flag_clear_disp = 0;
int8_t selec = 0;
int8_t aux_sub = 0;
int8_t aux_sub_modo = 0;
int8_t aux_sub_reg = 0;
int8_t aux_sub_sec = 0;

uint16_t cont = 0;
uint8_t libera_cont = 0;

uint32_t ch0;
uint32_t ch1;
uint32_t ch2;
uint32_t ch3;

uint8_t RX_msg[1];
uint8_t TX_msg[60] = "";
uint32_t Duty = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart1, RX_msg, 1);
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, adc_buffer, ADC_BUF_SIZE);
  HAL_TIM_Base_Start(&htim3);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  TIM1->CCR1 = 0;
  TIM2->CCR1 = 0;

  HAL_UART_Receive_IT(&huart1, RX_msg, 1);
  HAL_UART_Transmit_DMA(&huart1, TX_msg, sizeof(TX_msg));
  //HAL_UART_Transmit(&huart1, TX_msg, sizeof(TX_msg), 10);

  Lcd_PortType ports[] = {
		  LCD_D4_GPIO_Port, LCD_D5_GPIO_Port, LCD_D6_GPIO_Port, LCD_D7_GPIO_Port};
  Lcd_PinType pins[] = {LCD_D4_Pin, LCD_D5_Pin, LCD_D6_Pin, LCD_D7_Pin};
  Lcd_HandleTypeDef lcd;
  lcd = Lcd_create(ports, pins, LCD_RS_GPIO_Port, LCD_RS_Pin, LCD_e_GPIO_Port, LCD_e_Pin, LCD_4_BIT_MODE);

  Lcd_cursor(&lcd, 0, 4);
  Lcd_string(&lcd,"OhmPower");

  Lcd_cursor(&lcd, 1, 3);
  Lcd_string(&lcd,"Electronic");

  HAL_Delay(5000);

  Lcd_clear(&lcd);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(adc_ready == 1){
		  adc_ready = 0;
		  ch0 = 4095 - adc_buffer[0];//Sensor de Umidade Planta
		  umidade = 100*(float)ch0/4095;
		  ch1 = adc_buffer[1];//Sensor de Nivel Prato
		  ch2 = adc_buffer[2];//Sensor de Nivel Reservatorio
		  ch3 = adc_buffer[3];

		  loop();

		  /*sprintf(TX_msg, "Valor Digital = %u %u %u %u\r\n", ch0, ch1, ch2, ch3);
		  //HAL_UART_Transmit_IT(&huart1, TX_msg, sizeof(TX_msg), 20);
		  HAL_UART_Transmit_DMA(&huart1, TX_msg, sizeof(TX_msg));

		  HAL_Delay(100);
		  sprintf(TX_msg, "Valor adc_ready = %d\r\n", adc_ready);
		  HAL_UART_Transmit_DMA(&huart1, TX_msg, sizeof(TX_msg));*/



		  //Duty = 1 + (ch0*98/4095);

		  //TIM1->CCR1 = Duty;
		  //TIM2->CCR1 = Duty;
		  /*Lcd_cursor(&lcd, 0, 0);
		  sprintf(teste,"%u %u", ch0, ch1);
		  Lcd_string(&lcd,teste);
		  Lcd_cursor(&lcd, 1, 0);
		  sprintf(teste,"%u %u", ch2, ch3);
		  Lcd_string(&lcd,teste);
		  */
	  }

	  if (flag_clear_disp == 1){
		  Lcd_clear(&lcd);
		  flag_clear_disp = 0;
	  }

	  //loop();

	  switch (option) {
		  case 0://Quador de Avisos
			  switch (selec) {
				  case 0://Sub quadro principal
					Lcd_cursor(&lcd, 0, 5);
					Lcd_string(&lcd,"Avisos");
					break;
				  case 1://Sub quadro de Indicação de Umidade
					if(ch0 >= 1600){
						//Lcd_clear(&lcd);
						Lcd_cursor(&lcd, 0, 2);
						Lcd_string(&lcd,"Umidade Alta ");
					}
					else if(ch0 < 1600 && ch0 > 400){
						//Lcd_clear(&lcd);
						Lcd_cursor(&lcd, 0, 2);
						Lcd_string(&lcd,"Umidade Boa  ");
					}
					else{
						//Lcd_clear(&lcd);
						Lcd_cursor(&lcd, 0, 2);
						Lcd_string(&lcd,"Umidade Baixa");
					}
					break;
				  case 2://Sub quadro de Indicação de Risco de Dengue
						if(ch1 > 1600){
							//Lcd_clear(&lcd);
							Lcd_cursor(&lcd, 0, 0);
							Lcd_string(&lcd,"Risco de Dengue");
							Lcd_cursor(&lcd, 1, 0);
							Lcd_string(&lcd,"Esvazie o Prato");
						}
						else{
							//Lcd_clear(&lcd);
							Lcd_cursor(&lcd, 0, 0);
							Lcd_string(&lcd,"Sem Risco de");
							Lcd_cursor(&lcd, 1, 0);
							Lcd_string(&lcd,"Dengue         ");
						}
						break;
				  case 3:
						Lcd_cursor(&lcd, 0, 0);
						Lcd_string(&lcd,"Alta Umidade");
						Lcd_cursor(&lcd, 1, 0);
						Lcd_string(&lcd,"Afoga a Planta");
						break;
				  case 4:
						Lcd_cursor(&lcd, 0, 0);
						Lcd_string(&lcd,"Todos Contra o");
						Lcd_cursor(&lcd, 1, 0);
						Lcd_string(&lcd,"Mosquito");
						break;
			  }
			  break;
		  case 1://Quadro de Umidade Atual e Desejada
			  switch (selec) {
				  case 0://Sub quadro Principal
					  Lcd_cursor(&lcd, 0, 5);
					  Lcd_string(&lcd,"Umidade");
					  Lcd_cursor(&lcd, 1, 4);
					  Lcd_string(&lcd,"da Planta");
					  break;
				  case 1:
					  Lcd_cursor(&lcd, 0, 2);
					  Lcd_string(&lcd,"Umidade Atual");

					  Lcd_cursor(&lcd, 1, 6);
					  sprintf(teste,"%.2f %%", umidade);
					  Lcd_string(&lcd,teste);
					  break;
				  case 2:
					  Lcd_cursor(&lcd, 0, 0);
					  Lcd_string(&lcd,"Umidade Desejada");

					  Lcd_cursor(&lcd, 1, 6);
					  sprintf(teste,"%u %%", aux_sub*10);
					  Lcd_string(&lcd,teste);
					  if (aux_sub>4){
						  Lcd_cursor(&lcd, 1, 4);
						  Lcd_string(&lcd,"!");
						  Lcd_cursor(&lcd, 1, 11);
						  Lcd_string(&lcd,"!");
					  }
					  else if (aux_sub<4){
						  Lcd_cursor(&lcd, 1, 4);
						  Lcd_string(&lcd," ");
						  Lcd_cursor(&lcd, 1, 11);
						  Lcd_string(&lcd," ");
					  }
					  break;

			  }
			  break;
		  case 2://Quadro de Modo Automatico
			  switch (selec) {
				  case 0:
					  Lcd_cursor(&lcd, 0, 2);
					  Lcd_string(&lcd,"Regar Planta");
					  Lcd_cursor(&lcd, 1, 0);
					  Lcd_string(&lcd,"Automaticamente");
					  break;
				  case 1:
					  if(aux_sub_modo == 0){
						  Lcd_cursor(&lcd, 0, 1);
						  Lcd_string(&lcd,"Modo Automatico");
						  Lcd_cursor(&lcd, 1, 0);
						  Lcd_string(&lcd,"Desativado");
						  libera_cont = 0;
					  }
					  else if (aux_sub_modo == 1){
						  Lcd_cursor(&lcd, 0, 1);
						  Lcd_string(&lcd,"Modo Automatico");
						  Lcd_cursor(&lcd, 1, 0);
						  Lcd_string(&lcd,"Ativado");
						  libera_cont = 1;
					  }
					  break;
			  }



			  break;
		  case 3://Quadro de Regar Manualmente
			  switch (selec) {
				  case 0:
					  Lcd_cursor(&lcd, 0, 2);
					  Lcd_string(&lcd,"Regar Planta");
					  Lcd_cursor(&lcd, 1, 2);
					  Lcd_string(&lcd,"Manualmente");
					  break;
				  case 1:
					  if(aux_sub_reg == 0){
						  Lcd_cursor(&lcd, 0, 1);
						  Lcd_string(&lcd,"Regar Planta");
						  Lcd_cursor(&lcd, 1, 0);
						  Lcd_string(&lcd,"Desativado");
						  TIM1->CCR1 = 0;
					  }
					  else if (aux_sub_reg == 1){
						  Lcd_cursor(&lcd, 0, 1);
						  Lcd_string(&lcd,"Regar Planta");
						  Lcd_cursor(&lcd, 1, 0);
						  Lcd_string(&lcd,"Ativado");
						  TIM1->CCR1 = 65;
						  //libera_cont = 1;
					  }
					  break;
			  }
			  break;
		  case 4://Quadro de Secar Manulmente
			  switch (selec) {
				  case 0:
					  Lcd_cursor(&lcd, 0, 2);
					  Lcd_string(&lcd,"Secar Prato");
					  Lcd_cursor(&lcd, 1, 2);
					  Lcd_string(&lcd,"Manualmente");
					  break;
				  case 1:
					  if(aux_sub_sec == 0){
						  Lcd_cursor(&lcd, 0, 2);
						  Lcd_string(&lcd,"Secar Prato");
						  Lcd_cursor(&lcd, 1, 0);
						  Lcd_string(&lcd,"Desativado");
						  TIM2->CCR1 = 0;
					  }
					  else if (aux_sub_sec == 1){
						  Lcd_cursor(&lcd, 0, 2);
						  Lcd_string(&lcd,"Secar Prato");
						  Lcd_cursor(&lcd, 1, 0);
						  Lcd_string(&lcd,"Ativado");
						  TIM2->CCR1 = 75;
					  }
					  break;
			  }
			  break;
	  }

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 719;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin|LCD_RW_Pin|LCD_e_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RS_Pin LCD_RW_Pin LCD_e_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_RW_Pin|LCD_e_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Avanca_Pin */
  GPIO_InitStruct.Pin = Avanca_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Avanca_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Seleciona_Pin Retrocede_Pin */
  GPIO_InitStruct.Pin = Seleciona_Pin|Retrocede_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D4_Pin LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(hadc);
  adc_ready = 1;

  if (libera_cont == 1 && cont<600){
  		if(cont<35 && umidade<(aux_sub*10)){
  			TIM1->CCR1 = 65;
  		}
  		else{
  			TIM1->CCR1 = 0;
  		}
  		if(cont<45 && ch1>2500){
  			TIM2->CCR1 = 75;
  		}
  		else{
  			TIM2->CCR1 = 0;
  		}
  		cont++;
  }
  else if(cont>600 && libera_cont == 1){
	  cont = 0;
  }
  else{
	  cont = 0;
  }
  //HAL_ADC_Stop_DMA(&hadc1);
}

void loop(void) {
    // Estados atuais e anteriores dos botões
    static uint8_t lastStateAvanca = GPIO_PIN_SET;
    static uint8_t lastStateSeleciona = GPIO_PIN_SET;
    static uint8_t lastStateRetrocede = GPIO_PIN_SET;

    // Tempo da última mudança de estado dos botões
    static uint32_t lastDebounceTimeAvanca = 0;
    static uint32_t lastDebounceTimeSeleciona = 0;
    static uint32_t lastDebounceTimeRetrocede = 0;

    // Tempo de debounce (em milissegundos)
    const uint32_t debounceDelay = 140;

    // Leitura atual dos estados dos botões
    uint8_t currentStateAvanca = HAL_GPIO_ReadPin(Avanca_GPIO_Port, Avanca_Pin);
    uint8_t currentStateSeleciona = HAL_GPIO_ReadPin(Seleciona_GPIO_Port, Seleciona_Pin);
    uint8_t currentStateRetrocede = HAL_GPIO_ReadPin(Retrocede_GPIO_Port, Retrocede_Pin);

    // Processamento do botão Avanca
    if (currentStateAvanca != lastStateAvanca) {
        lastDebounceTimeAvanca = HAL_GetTick();
    }

    if ((HAL_GetTick() - lastDebounceTimeAvanca) > debounceDelay) {
        // Verifica se o estado do botão mudou após o tempo de debounce
        if (currentStateAvanca == GPIO_PIN_RESET) {
        	if (selec == 0){//Bloqueia
        		option += 1;
				flag_clear_disp = 1;
				if (option > 4){
					option = 0;
				}
        	}
        	else if (selec == 2 && option == 1){//valor desejado
        		flag_clear_disp = 1;
        		aux_sub += 1;
        		if (aux_sub > 10){
        			aux_sub = 0;
        		}
        	}
        	else if (selec == 1 && option == 2){//Modo de Operação automatico
				flag_clear_disp = 1;
				aux_sub_modo += 1;
				if (aux_sub_modo > 1){
					aux_sub_modo = 0;
				}
			}
        	else if (selec == 1 && option == 3){//Modo de Regar Manual
				flag_clear_disp = 1;
				aux_sub_reg += 1;
				if (aux_sub_reg > 1){
					aux_sub_reg = 0;
				}
			}
        	else if (selec == 1 && option == 4){//Modo de Secar Manual
				flag_clear_disp = 1;
				aux_sub_sec += 1;
				if (aux_sub_sec > 1){
					aux_sub_sec = 0;
				}
			}
        }
    }

    lastStateAvanca = currentStateAvanca;  // Atualiza o estado anterior

    // Processamento do botão Seleciona
    if (currentStateSeleciona != lastStateSeleciona) {
        lastDebounceTimeSeleciona = HAL_GetTick();
    }

    if ((HAL_GetTick() - lastDebounceTimeSeleciona) > debounceDelay) {
        if (currentStateSeleciona == GPIO_PIN_RESET) {
        	switch (option) {
			  case 0:
				  flag_clear_disp = 1;
				  selec += 1;
				  if (selec > 4){
						selec = 0;
					}
				  break;
			  case 1:
				  flag_clear_disp = 1;
				  selec += 1;
				  if (selec > 2){
						selec = 0;
					}
				  break;
			  case 2:
				  flag_clear_disp = 1;
				  selec += 1;
				  if (selec > 1){
						selec = 0;
					}
				  break;
			  case 3:
				  flag_clear_disp = 1;
				  selec += 1;
				  if (selec > 1){
						selec = 0;
					}
				  break;
			  case 4:
				  flag_clear_disp = 1;
				  selec += 1;
				  if (selec > 1){
						selec = 0;
					}
				  break;
		  }
        }
    }

    lastStateSeleciona = currentStateSeleciona;

    // Processamento do botão Retrocede
    if (currentStateRetrocede != lastStateRetrocede) {
        lastDebounceTimeRetrocede = HAL_GetTick();
    }

    if ((HAL_GetTick() - lastDebounceTimeRetrocede) > debounceDelay) {
        if (currentStateRetrocede == GPIO_PIN_RESET) {
        	if(selec == 0){
        		option -= 1;
				flag_clear_disp = 1;
				if (option < 0){
					option = 4;
				}
        	}
        	else if (selec == 2 && option == 1){//valor desejado
				flag_clear_disp = 1;
				aux_sub -= 1;
				if (aux_sub < 0){
					aux_sub = 10;
				}
			}
        	else if (selec == 1 && option == 2){//Modo de Operação automatico
				flag_clear_disp = 1;
				aux_sub_modo -= 1;
				if (aux_sub_modo < 0){
					aux_sub_modo = 1;
				}
			}
        	else if (selec == 1 && option == 3){//Modo de Regar Manual
				flag_clear_disp = 1;
				aux_sub_reg -= 1;
				if (aux_sub_reg < 0){
					aux_sub_reg = 1;
				}
			}
        	else if (selec == 1 && option == 4){//Modo de Secar Manual
				flag_clear_disp = 1;
				aux_sub_sec -= 1;
				if (aux_sub_sec < 0){
					aux_sub_sec = 1;
				}
			}
        }
    }

    lastStateRetrocede = currentStateRetrocede;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim4){
	//adc_ready = 1;
	if (libera_cont == 1){
		if(cont<10 && umidade<(100)){
			TIM1->CCR1 = 75;
		}
		else{
			TIM1->CCR1 = 0;
		}
		if(cont<10 && ch1>1600){
			TIM2->CCR1 = 75;
		}
		else{
			TIM2->CCR1 = 0;
		}
		cont++;
	}
	else if(libera_cont == 1 && cont>60){
		cont = 0;
	}
	else{
		cont = 0;
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
