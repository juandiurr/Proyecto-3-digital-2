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
#include "string.h"
#include "stdio.h"
#include "ili9341.h"
#include "bitmaps.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t RX[0]; // Buffer para recepción de datos
uint8_t adc = 0;
uint8_t xx = 0;
uint8_t yy = 0;
uint8_t uno = 0;
uint8_t dos = 0;
uint8_t tres = 0;
uint8_t cuatro = 0;
uint8_t cinco = 0;
uint8_t seis = 0;
uint8_t siete = 0;
uint8_t ocho = 0;
uint8_t unoo = 0;
uint8_t doss = 0;
uint8_t tress = 0;
uint8_t cuatroo = 0;
uint8_t cincoo = 0;
uint8_t seiss = 0;
uint8_t sietee = 0;
uint8_t ochoo = 0;
uint8_t dispo[0];
uint8_t dispos = 0;
uint8_t p_dispo = 0;
uint8_t c1 = 0;
uint8_t c2 = 0;
uint8_t c3 = 0;
uint8_t c4 = 0;
uint8_t c5 = 0;
uint8_t c6 = 0;
uint8_t c7 = 0;
uint8_t c8 = 0;
uint8_t p1 = 0;
uint8_t p2 = 0;
uint8_t p3 = 0;
uint8_t p4 = 0;
uint8_t ret = 0;
uint8_t buf[0];
uint8_t final = 0;
char buffer[4];
char buffer2[4];
const uint16_t rosa = 0xF97A;
const uint16_t verde = 	0x06A0;
const uint16_t rojo = 0xE863;
const uint16_t naranja = 0xF3C0;
const uint16_t azul = 0x023F;
const uint16_t amarillo = 0xD6A0;
const uint16_t gris = 0x4A69;
const uint8_t l1 = 80;
const uint8_t g = 5;
const uint8_t l2 = 50;
static const uint8_t esclavo1 = 0x08 << 1; //nucleo
const uint8_t esclavo2 = 0x09 << 1; //esp32

extern uint8_t explosion_[];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
void transmit_uart(char *message);
void transmit_uart2(char *message);
void parqueo1(void);
void parqueo2(void);
void parqueo3(void);
void parqueo4(void);
void explosion(uint8_t parqueo);
uint8_t random(void);
void bitmap(void);
void transmit_i2c(char* message);

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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_UART5_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UART_Receive_IT(&huart2, RX, 1);
  HAL_UART_Receive_IT(&huart5, RX, 1);
  HAL_TIM_Base_Start_IT(&htim14);
  LCD_Init();
  LCD_Clear(gris);
  //arriba
  FillRect(10,9,l2,g,amarillo);
  FillRect(10,9,g,l1,amarillo);
  FillRect(l2+10,9,g,l1,amarillo);
  FillRect(85,9,l2,g,amarillo);
  FillRect(85,9,g,l1,amarillo);
  FillRect(l2+85,9,g,l1,amarillo);
  FillRect(170,9,l2,g,amarillo);
  FillRect(170,9,g,l1,amarillo);
  FillRect(l2+170,9,g,l1,amarillo);
  FillRect(245,9,l2,g,amarillo);
  FillRect(245,9,g,l1,amarillo);
  FillRect(l2+245,9,g,l1,amarillo);

  //abajo
  FillRect(10,231,l2+g,g,amarillo);
  FillRect(10,231-l1,g,l1,amarillo);
  FillRect(l2+10,231-l1,g,l1,amarillo);
  FillRect(85,231,l2+g,g,amarillo);
  FillRect(85,231-l1,g,l1,amarillo);
  FillRect(l2+85,231-l1,g,l1,amarillo);
  FillRect(170,231,l2+g,g,amarillo);
  FillRect(170,231-l1,g,l1,amarillo);
  FillRect(l2+170,231-l1,g,l1,amarillo);
  FillRect(245,231,l2+g,g,amarillo);
  FillRect(245,231-l1,g,l1,amarillo);
  FillRect(l2+245,231-l1,g,l1,amarillo);
  LCD_Bitmap(150,10,11,11,flor);
  LCD_Bitmap(152,28,11,11,flor);
  LCD_Bitmap(151,200,11,11,flor);
  LCD_Bitmap(152,230,11,11,flor);

  /*FillRect(10+g,9+g,40,70,azul);
  FillRect(85+g,9+g,40,70,azul);
  FillRect(10+g,240-l1+1,40,70,azul);*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (c1 == 1){
		  explosion(1);
		  unoo = uno;
		  if(uno == 1){
			  FillRect(xx,yy,45,70,gris);
			  bitmap();
		  }else{
			  FillRect(xx,yy,45,70,gris);
		  }
		  c1 = 0;
	  }
	  if (c2 == 1){
		  explosion(2);
		  c2 = 0;
		  if(dos == 1){
			  FillRect(xx,yy,45,70,gris);
			  bitmap();
		  }else{
			  FillRect(xx,yy,45,70,gris);
		  }
		  doss = dos;

	  }
	  if (c3 == 1){
		  explosion(3);
		  c3 = 0;
		  if(tres == 1){
			  FillRect(xx,yy,45,70,gris);
			  bitmap();
		  }else{
			  FillRect(xx,yy,45,70,gris);
		  }
		  tress = tres;
	  }
	  if (c4 == 1){
		  explosion(4);
		  if(cuatro == 1){
			  FillRect(xx,yy,45,70,gris);
			  bitmap();
		  }else{
			  FillRect(xx,yy,45,70,gris);
		  		  }
		  c4 = 0;
		  cuatroo = cuatro;
	  }
	  if (c5 == 1){
		  explosion(5);
		  if(cinco == 1){
			  FillRect(xx,yy,45,70,gris);
			  bitmap();
		  }else{
			  FillRect(xx,yy,45,70,gris);
		  }
		  c5 = 0;
		  cincoo = cinco;
	  }
	  if (c6 == 1){
		  explosion(6);
		  if(seis == 1){
			  FillRect(xx,yy,45,70,gris);
			  bitmap();
		  }else{
			  FillRect(xx,yy,45,70,gris);
		  }
		  seiss = seis;
		  c6 = 0;
	  }
	  if (c7 == 1){
		  explosion(7);
		  sietee = siete;
		  if(siete == 1){
			  FillRect(xx,yy,45,70,gris);
			  bitmap();
		  }else{
			  FillRect(xx,yy,45,70,gris);
		  }
		  c7 = 0;
	  }
	  if (c8 == 1){
		  explosion(8);
		  ochoo = ocho;
		  if(ocho == 1){
			  FillRect(xx,yy,45,70,gris);
			  bitmap();
		  }else{
			  FillRect(xx,yy,45,70,gris);
		  }
		  c8 = 0;

	  }
	  parqueo1();
	  p1 = 1;
	  HAL_Delay(50);
	  HAL_ADC_Start_IT(&hadc1);
	  HAL_Delay(50);
	  parqueo2();
	  p2 = 1;
	  HAL_ADC_Start_IT(&hadc1);
	  HAL_Delay(50);
	  parqueo3();
	  p3 = 1;
	  HAL_ADC_Start_IT(&hadc1);
	  HAL_Delay(50);
	  parqueo4();
	  p4 = 1;
	  HAL_ADC_Start_IT(&hadc1);
	  HAL_Delay(50);
	  dispo[0] = 8 - (uno + dos + tres + cuatro + cinco + seis + siete + ocho);
	  dispos = dispo[0];
	  if(cinco == 1){
		  //FillRect(45,200,10,10,verde);
		  transmit_i2c("E");
		  p_dispo |= (1<<4);
	  }else{
		  transmit_i2c("e");
		  //FillRect(45,200,10,10,0x0000);
		  p_dispo &= ~(1<<4);
	  }
	  if(seis == 1){
		  transmit_i2c("F");
		  //FillRect(85,200,10,10,azul);
		  p_dispo |= (1<<5);
	  }else{
		  transmit_i2c("f");
		  //FillRect(85,200,10,10,0x0000);
		  p_dispo &= ~(1<<5);
	  }
	  if(siete == 1){
		  transmit_i2c("G");
		  //FillRect(125,200,10,10,naranja);
		  p_dispo |= (1<<6);
	  }else{
		  transmit_i2c("g");
		  //FillRect(125,200,10,10,0x0000);
		  p_dispo &= ~(1<<6);
	  }
	  if(ocho == 1){
		  transmit_i2c("H");
		  //FillRect(165,200,10,10,rosa);
		  p_dispo |= (1<<7);
	  }else{
		  transmit_i2c("h");
		  //FillRect(165,200,10,10,0x0000);
		  p_dispo &= ~(1<<7);
	  }
	  if(uno == 1){
		  transmit_i2c("A");
		  p_dispo |= (1<<0);
	  }else{
		  transmit_i2c("a");
		  p_dispo &= ~(1<<0);
	  }
	  if(dos == 1){
		  transmit_i2c("B");
		  p_dispo |= (1<<1);
	  }else{
		  transmit_i2c("b");
		  p_dispo &= ~(1<<1);
	  }
	  if(tres == 1){
		  transmit_i2c("C");
		  p_dispo |= (1<<2);
	  }else{
		  transmit_i2c("c");
		  p_dispo &= ~(1<<2);
	  }
	  if(cuatro == 1){
		  transmit_i2c("D");
		  p_dispo |= (1<<3);
	  }else{
		  transmit_i2c("d");
		  p_dispo &= ~(1<<3);
	  }
	  //buf[0] = dispo;
	  sprintf(buffer2, "%u",p_dispo);
	  //LCD_Print(buffer2,270,0,1,0xFFFF,0x0000);
	  sprintf(buffer, "%u",dispos);
	  transmit_uart2(buffer);
	  LCD_Print(buffer, 160,100,2,0xFFFF,gris);
	  HAL_Delay(250);
	  if(unoo != uno){
			c1 = 1;
		}
		if(doss != dos){
			c2 = 1;
		}
		if(tress != tres){
			c3 = 1;
		}
		if(cuatroo != cuatro){
			c4 = 1;
		}
		if(cincoo != cinco){
			c5 = 1;
		}
		if(seis != seiss){
			c6 = 1;
		}
		if(sietee != siete){
			c7 = 1;
		}
		if(ocho != ochoo){
			c8 = 1;
		}


  /* USER CODE END 3 */
}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_6B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 1000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 4000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  /*if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    //Error_Handler();
	  FillRect(90,90,90,90,verde);
  }*/
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin|LCD_D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin|SD_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RST_Pin LCD_D1_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LCD_D7_Pin
                           LCD_D0_Pin LCD_D2_Pin */
  GPIO_InitStruct.Pin = LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_D6_Pin LCD_D3_Pin LCD_D5_Pin
                           LCD_D4_Pin SD_SS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin|SD_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t randommm(void){
	return random() % 4;
}
void bitmap(void){
	uint8_t rand;
	uint8_t rand2;
	uint8_t rand3;
	rand = randommm();
	rand2 = randommm();
	rand3 = randommm();
	if (rand == 0){
		LCD_Bitmap(xx+rand3,yy+rand2,35,70,carro_verde);
	}else if(rand == 1){
		LCD_Bitmap(xx+rand3,yy+rand2,35,70,carro_amarillo);
	}else if(rand == 2){
		LCD_Bitmap(xx+rand3,yy+rand2,40,70,carro_morado);
	}else if(rand == 3){
		LCD_Bitmap(xx+rand3,yy+rand2,40,72,carro_rojo);
	}
}
void explosion(uint8_t parqueo){

	/*FillRect(10+g,9+g,40,70,azul);
	  FillRect(85+g,9+g,40,70,azul);
	  FillRect(10+g,240-l1+1,40,70,azul);*/
	if(parqueo == 1){
		yy = 9+g;
		xx = 10+g;
	}else if(parqueo == 2){
		yy = 9+g;
		xx = 85+g;
	}else if(parqueo == 3){
		yy = 9+g;
		xx = 170+g;
	}else if(parqueo == 4){
		yy = 9+g;
		xx = 245+g;
	}else if(parqueo == 5){
		yy = 240-l1+1;
		xx = 10+g;
	}else if(parqueo == 6){
		yy = 240-l1+1;
		xx = 85+g;
	}else if(parqueo == 7){
		yy = 240-l1+1;
		xx = 170+g;
	}else if(parqueo == 8){
		yy = 240-l1+1;
		xx = 245+g;
	}
	LCD_Sprite(xx,yy,40,70,explosion_,5,0,0,0);
	HAL_Delay(25);
	LCD_Sprite(xx,yy,40,70,explosion_,5,1,0,0);
	HAL_Delay(25);
	LCD_Sprite(xx,yy,40,70,explosion_,5,2,0,0);
	HAL_Delay(25);
	LCD_Sprite(xx,yy,40,70,explosion_,5,3,0,0);
	HAL_Delay(25);
	LCD_Sprite(xx,yy,40,70,explosion_,5,4,0,0);
	HAL_Delay(25);


}
void parqueo1(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_9;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
void parqueo2(void){
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	//FillRect(0,0,40,40,0xFFFF);
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_10;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
void parqueo3(void){
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_12;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
void parqueo4(void){
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_13;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void transmit_uart(char *message) {
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}
void transmit_uart2(char *message) {
    HAL_UART_Transmit(&huart5, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}
void transmit_i2c(char* message){
	ret = HAL_I2C_Master_Transmit(&hi2c1, esclavo1, message, 1, HAL_MAX_DELAY);//manda a la esp la informacion de los parqueos
	  if(ret != HAL_OK){
		  LCD_Print("error", 160,120, 1,0xFFFF,gris);
	  }else{
		  FillRect(160,120,50,20,gris);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == UART5){
		//FillRect(0,0,10,10,0xFFFF);
		//LCD_Print(RX[0],0,0,2,0xFFFF,0x0000);
		if(RX[0] == 'a'){//no parqueado
			cinco = 0;
			p_dispo &= ~(1<<4);
		}else if(RX[0] == 'A'){//parqueado
			cinco = 1;
			p_dispo |= (1<<4);
		}else if(RX[0] == 'b'){
			seis = 0;
			p_dispo &= ~(1<<5);
		}else if(RX[0] == 'B'){
			seis = 1;
			p_dispo |= (1<<5);
		}else if(RX[0] == 'c'){
			siete = 0;
			p_dispo &= ~(1<<6);
		}else if(RX[0] == 'C'){
			siete = 1;
			p_dispo |= (1<<6);
		}else if(RX[0] == 'd'){
			ocho = 0;
			p_dispo &= ~(1<<7);
		}else if(RX[0] == 'D'){
			ocho = 1;
			p_dispo |= (1<<7);
		}
	}
	HAL_UART_Receive_IT(&huart5, RX, 1);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc == &hadc1){
    	if(p1 == 1){
    		adc = HAL_ADC_GetValue(&hadc1);
    		if(adc < 35){
    			//p_dispo |= (1<<0);
    			uno = 1;
				//FillRect(45,60,10,10,verde);
				//FillRect(45,40,30,10,0x0000);

			}else if(adc >= 35 && adc < 100){
				//p_dispo &= ~(1<<0);
				uno = 0;
				//FillRect(45,60,10,10,0x0000);
				//FillRect(45,40,30,10,0x0000);
			}else{
				//p_dispo &= ~(1<<0);
				uno = 0;
			}
			p1 = 0;
			//sprintf(buffer, "%u",adc);
			//LCD_Print(buffer, 40, 40, 1,0xFFFF,0x0000);
    	}
    	if(p2 == 1){
    		adc = HAL_ADC_GetValue(&hadc1);
    		if(adc < 35){
    			//p_dispo |= (1<<1);
    			dos = 1;
    			//FillRect(85,60,10,10,azul);
				//FillRect(85,40,30,10,0x0000);
			}else if(adc >= 35 && adc < 100){
				//p_dispo &= ~(1<<1);
				dos = 0;
				//FillRect(85,60,10,10,0);
				//FillRect(90,40,30,10,0x0000);
			}else{
				//p_dispo &= ~(1<<1);
				dos = 0;
			}
    		p2 = 0;
    		//sprintf(buffer, "%u",adc);
    		//LCD_Print(buffer, 80, 40, 1,0xFFFF,0x0000);
    	}
    	if(p3 == 1){
    		adc = HAL_ADC_GetValue(&hadc1);
    		if(adc < 35){
    			//p_dispo |= (1<<2);
    			tres = 1;
    			//FillRect(125,60,10,10,naranja);
				//FillRect(125,40,30,10,0x0000);
			}else if(adc >= 35 && adc < 100){
				//p_dispo &= ~(1<<2);
				tres = 0;
				//FillRect(125,60,10,10,0x0000);
				//FillRect(130,40,30,10,0x0000);
			}else{
				//p_dispo &= ~(1<<2);
				tres = 0;
			}
			p3 = 0;
			//sprintf(buffer, "%u",adc);
			//LCD_Print(buffer, 120, 40, 1,0xFFFF,0x0000);
    	}
    	if(p4 == 1){
    		adc = HAL_ADC_GetValue(&hadc1);
    		if(adc < 35){
    			//p_dispo |= (1<<3);
    			cuatro = 1;
    			//FillRect(165,60,10,10,rosa);
				//FillRect(165,40,30,10,0x0000);
			}else if(adc >= 35 && adc < 100){
				//p_dispo &= ~(1<<3);
				cuatro = 0;
				//FillRect(165,60,10,10,0x0000);
				//FillRect(170,40,30,10,0x0000);
			}else{
				//p_dispo &= ~(1<<3);
				cuatro = 0;
			}
			p4 = 0;
			//sprintf(buffer, "%u",adc);
			//LCD_Print(buffer, 160, 40, 1,0xFFFF,0x0000);
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
