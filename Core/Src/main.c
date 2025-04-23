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
#include "stdio.h"
#include <stdarg.h>
#include "string.h"

#include "Uart_Handler.h"
#include "StepperController.h"
#include "RefModel.h"
#include "ADS1220.h"

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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
float speed = 0;
float force;
float LPF1_Beta = 0.6; // 0<ß<1 Force Filter constant

/*Debuging only variables */
float th = 2;
uint16_t fforce = 0;
uint16_t tMarks[10] = {0};
uint16_t timeouts = 0;
uint16_t oddDrv = 0;
/**/

rMod_t hmod1 = {0};
piCon_t hcon1 = {0}; // PI position controller
cMap_1d_t curve[255] = {{-90, 	 0},
						{-50, 	 0},
						{-0.5, 	-20},
						{ 0.5, 	 20},
						{ 50, 	 0},
						{ 90, 	 0}};
uint8_t nPoints = 6; // Number of points defined in the curve vector

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

uint8_t checkModelTimeout(uint8_t rst, uint32_t dt){

	static uint8_t ptr = 0;

	if (rst) {
		ptr = 0;
		memset(tMarks, 0, sizeof(tMarks));
	}else{
		if(ptr == 0) tMarks[ptr++] = __HAL_TIM_GET_COUNTER(&htim3);
		else tMarks[ptr++] = __HAL_TIM_GET_COUNTER(&htim3) - tMarks[ptr-1];
	}
	  if(__HAL_TIM_GET_COUNTER(&htim3) > dt){
		  HAL_TIM_Base_Stop(&htim3);
		  HAL_TIM_Base_Stop(&htim2);
		  asm("NOP");
		  timeouts++;
		  HAL_TIM_Base_Start(&htim3);
		  HAL_TIM_Base_Start(&htim2);
		  return 1;
	  }
	  return 0;
}
//void delay_us(uint16_t us){
//	__HAL_TIM_SET_COUNTER(&htim2, 0);
//	while(__HAL_TIM_GET_COUNTER(&htim2) < us);
//}

//
//
////https://cdn.sparkfun.com/assets/learn_tutorials/5/4/6/hx711F_EN.pdf
//int32_t Sensor_Receive(void){
//
//	static uint32_t pData = 0;
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//
//	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_SET) return pData;
////	while(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_SET);
//	pData = 0;
//
//	// Read ChA Gain 128
//	for(int i=0; i<24; i++){
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
//		pData = pData<<1;
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//		if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_SET) pData++;
//	}
//
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
//	//pData=pData^0x800000;
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//
//	if(pData&0x00800000) pData |= 0xff000000; // Convert 24bits 2's complement into 32bits
//	return pData;
//}
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start_IT(&htim2);
  UART1_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//https://roboticx.ps/product/ad620-instrumentation-amplifier-module/

//  ADS1220_select_mux_config(&hspi1, ADS1220_MUX_AIN1_AIN2, &regs);
//  ADS1220_set_pga_gain(&hspi1, ADS1220_PGA_GAIN_128, &regs);
//  ADS1220_set_operating_mode(&hspi1, ADS1220_MODE_TURBO, &regs);
//  ADS1220_set_data_rate(&hspi1, ADS1220_TM_2000SPS, &regs);
//  ADS1220_set_conv_mode_continuous(&hspi1, &regs);
//  ADS1220_set_voltage_ref(&hspi1, ADS1220_VREF_EXT_REF_1, &regs);
//  ADS1220_enable_PSW(&hspi1, &regs);
//  ADS1220_get_config(&hspi1, &regs);

  ADS1220_regs regs = { 0x3E, 0xD4, 0x88, 0x00 };
  ADS1220_init(&hspi1, &regs); // Optionally check for failure
  ADS1220_set_conv_mode_single_shot(&hspi1, &regs);
  //ADS1220_start_conversion(&hspi1);


  	uint32_t timeStamp = 0; /* Timer for UART tx */

  	//force emulation
	int32_t fOffset = 137;
	float scalingFactor_N = 84.5;  // bits per Newton

	hmod1.dt = 500; 	// us /* This can go lower than 500us due ADC timing limitations */

	hmod1.m = 1;
	hmod1.c = 20; 		// N.s/m
	//hmod1.k = 50; 		// N/m

	hmod1.cMap = &curve;
	hmod1.cMap_size = nPoints;

	hmod1.us = 0.2; 		// Dynamic friction coefficient
	hmod1.ud = 0.2; 		// Static friction coefficient
	hmod1.N = 5; 			// Normal Force (Weight)
	hmod1.dfv = 0.01;	// mm/s

	hmod1.posMinLim = curve[0].x;
	hmod1.posMaxLim = curve[nPoints-1].x;

	hmod1.velMaxLim = 1500;	// Hardware max reachable speed.
	hmod1.velMinLim = -1500;

	hcon1.dt = hmod1.dt;
	hcon1.kp = 100;
	hcon1.ki = 0.25;
	hcon1.outMax = hmod1.velMaxLim;
	hcon1.outMin = hmod1.velMinLim;

  while (1)
  {
	  __HAL_TIM_SET_COUNTER(&htim3, 0); // re-start monitoring timer
/*N*/  checkModelTimeout(1, hmod1.dt);

	  UART1_Handler();

	  int32_t sAux = 0;
	  if( ADS1220_read_singleshot(&hspi1, GPIOC, GPIO_PIN_4, &sAux, 10) ){

		  int16_t raw = ((sAux & 0x00FFFF00)>>8) + fOffset;
		  float nforce = (float)raw / scalingFactor_N;

		  /*Debug Foce sensor*/
		  if(fabs((double)(nforce-force)) > th){
			fforce++;
		  }

		  force = nforce;
	  }

	// Filter 1 Force
	  static float smoothForce = 0;
	  smoothForce = smoothForce - (LPF1_Beta * (smoothForce - force));

	// Reference model
	//------------------------------------------//
	 refModel_Tick(&hmod1, smoothForce);
	//------------------------------------------//

	// Position Controller
	//------------------------------------------//
	hcon1.dt = hmod1.dt;
	float refSpeed = Compute_PI(&hcon1, hmod1.pos, (StepCon_GetPosition()));

	//------------------------------------------//

	 /* Drive motor Speed with corrected ref velocity */
	 speed = (hmod1.vel + refSpeed); // in mm/s

	 StepCon_Speed(speed);

	 // Console logs
	 if(timeStamp + 50 < HAL_GetTick()){
		 UART1_printf("cmd=%.4f, %.4f\r\n", (float)StepCon_GetPosition(), smoothForce);
		 timeStamp = HAL_GetTick();
	 }

//
//	//	/* Set here a timer to wait for the elapsed time.
//	//	 * You can compare the timer counter and trigger an alarm
//	//	 * if the time was already gone by the time the program
//	//	 * reached this point
//	//	 **/

/*6*/  checkModelTimeout(0, hmod1.dt);
	 while(__HAL_TIM_GET_COUNTER(&htim3) < hmod1.dt);

    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  htim2.Init.Prescaler = 90-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 0xffff-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 90-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5|GPIO_PIN_7|LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PG5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PG7 LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_7|LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void UART1_Cmd_Callback(uint8_t* cmd, uint16_t len){

	/* Process your commands here */
	float aux = 0;

	if(!len) return; /* Ignore empty commands */


	if( isCmd("cmap=") ) {

		cMap_1d_t* points = (cMap_1d_t*) hmod1.cMap;

		// Skip the "cmap=" prefix
		const char* data_start = (const char*)cmd + 5;

		// Determine the number of pairs by counting commas
		uint16_t num_pairs = 0;
		const char* ptr = data_start;
		//while (*ptr) {
		for (int i=0;i<(len-5);i++){
			if (*ptr == ',') {
				num_pairs++;
			}
			ptr++;
		}

		// Each pair has two values, so number of pairs is half the commas
		num_pairs = (num_pairs + 1) / 2;

		if (num_pairs > 255) return;

		// Parse the data points using sscanf
		size_t index = 0;
		ptr = data_start;
		while (index < num_pairs && *ptr) {
			float x, y;
			int scanned = sscanf(ptr, "%f,%f", &x, &y);
			if (scanned == 2) {
				points[index].x = x;
				points[index].f = y;
				index++;
			}

			// Move pointer to the next pair
			while (*ptr && *ptr != ',') ptr++;
			if (*ptr == ',') ptr++;
			while (*ptr && *ptr != ',') ptr++;
			if (*ptr == ',') ptr++;
		}

		hmod1.cMap_size = num_pairs;

		hmod1.posMinLim = points[0].x;
		hmod1.posMaxLim = points[num_pairs-1].x;

	}else if( isCmd("mass=") ) {

		int res = sscanf((const char*)cmd,"mass=%f", &aux);
		if(res) {
			hmod1.m = aux;
		}

	}else if( isCmd("damp=") ) {
		int res = sscanf((const char*)cmd,"damp=%f", &aux);
		if(res) {
			hmod1.c = aux;
		}

	}else if( isCmd("frcn=") ) {
		int res = sscanf((const char*)cmd,"frcn=%f", &aux);
		if(res) {
			hmod1.N = aux;
		}

	}else if( isCmd("vmax=") ) {
		int res = sscanf((const char*)cmd,"vmax=%f", &aux);
		if(res) {
			hmod1.velMaxLim = aux;
		}

	}else if( isCmd("vmin=") ) {
		int res = sscanf((const char*)cmd,"vmin=%f", &aux);
		if(res) {
			hmod1.velMinLim = aux;
		}
	}


	/*----------------------------*/
}


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//
//	 if (htim->Instance == TIM2) {
//
//		 StepCon_pulseTick();
//	}
//
//}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

//	uint8_t SPIbuf[3] = {0};
//	long int bit24;
//
//	HAL_SPI_Receive(&hspi1, SPIbuf, 3, 1);
//
//	bit24 = SPIbuf[0];
//	bit24 = (bit24 << 8) | SPIbuf[1];
//	bit24 = (bit24 << 8) | SPIbuf[2]; //Converting 3 bytes to a 24 bit int
//
//	bit24 = (bit24 << 8);
//	fSens = (bit24 >> 8); //Converting 24 bit two's complement to 32 bit two's complement
//	asm("NOP");
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM2) {
		StepCon_pulseTick();
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
