/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

osThreadId defaultTaskHandle;
osThreadId ADC_calcHandle;
osThreadId displayHandle;
osThreadId PWM_controlHandle;
osThreadId state_controlHandle;
osThreadId pad_readHandle;
osSemaphoreId key_semHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

int sampleNB = 40; //CHANGE FOR SAMPLE NUMBER
int sample = 0;

uint16_t dutyCycle = 10;

#define MATH_ARRAY_SIZE 5
#define WKEY1 0
#define WKEY2 1
#define WENTER 2
#define DISPLAY 3
#define SLEEP 4
#define samplesize 40

int correctCounter=0;
int correctPeriod = 5;

int displayMode = 0;
float filterMemory [] = {0, 0, 0, 0, 0};
int adc_val;
float filtered_adc;
float mathResults [MATH_ARRAY_SIZE];
extern uint8_t systickFlag;
//extern uint8_t buttonFlag;

float dispNum = 0;
int padEntries [] = {0, 0, 0, 0};
int padVal = -1;

int padFlag = 0;
int holdingFlag = 0;
int correctFlag = 0;

int holdCount = 0;
int highPeriods = 100;
int state = WKEY1;

float data [samplesize];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void Start_ADC_calc(void const * argument);
void Start_display(void const * argument);
void Start_PWM_control(void const * argument);
void Start_state_control(void const * argument);
void Start_pad_read(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void FIR_C(float input, float *output);
int buttonPressed(void);
void displayNum (int num, int pos);
void C_math (float * inputArray, float * outputArray, int length);
void display (int mode, float num);
int readPad (void);
void updateEnt (int newEnt);
void pwmSetValue(uint16_t pulseValue);
void set_highPeriods(int current_period);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_IT(&hadc1);
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of key_sem */
  osSemaphoreDef(key_sem);
  key_semHandle = osSemaphoreCreate(osSemaphore(key_sem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ADC_calc */
  osThreadDef(ADC_calc, Start_ADC_calc, osPriorityIdle, 0, 256);
  ADC_calcHandle = osThreadCreate(osThread(ADC_calc), NULL);

  /* definition and creation of display */
  osThreadDef(display, Start_display, osPriorityIdle, 0, 128);
  displayHandle = osThreadCreate(osThread(display), NULL);

  /* definition and creation of PWM_control */
  osThreadDef(PWM_control, Start_PWM_control, osPriorityIdle, 0, 256);
  PWM_controlHandle = osThreadCreate(osThread(PWM_control), NULL);

  /* definition and creation of state_control */
  osThreadDef(state_control, Start_state_control, osPriorityNormal, 0, 256);
  state_controlHandle = osThreadCreate(osThread(state_control), NULL);

  /* definition and creation of pad_read */
  osThreadDef(pad_read, Start_pad_read, osPriorityIdle, 0, 256);
  pad_readHandle = osThreadCreate(osThread(pad_read), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 82;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 42;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB10   ------> I2S2_CK
     PC7   ------> I2S3_MCK
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
     PB6   ------> I2C1_SCL
     PB9   ------> I2C1_SDA
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|Dig_1_Pin|Dig_2_Pin|Dig_L_Pin 
                          |Dig_3_Pin|Dig_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Row_1_Pin|Row_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Row_3_Pin|Row_4_Pin|Seg_DP_Pin|Seg_G_Pin 
                          |Seg_F_Pin|Seg_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Seg_D_Pin|Seg_C_Pin|Seg_B_Pin|Seg_A_Pin 
                          |LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin Dig_1_Pin Dig_2_Pin Dig_L_Pin 
                           Dig_3_Pin Dig_4_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|Dig_1_Pin|Dig_2_Pin|Dig_L_Pin 
                          |Dig_3_Pin|Dig_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin Row_1_Pin Row_2_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|Row_1_Pin|Row_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Row_3_Pin Row_4_Pin Seg_DP_Pin Seg_G_Pin 
                           Seg_F_Pin Seg_E_Pin */
  GPIO_InitStruct.Pin = Row_3_Pin|Row_4_Pin|Seg_DP_Pin|Seg_G_Pin 
                          |Seg_F_Pin|Seg_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Col_1_Pin Col_2_Pin Col_3_Pin Col_4_Pin */
  GPIO_InitStruct.Pin = Col_1_Pin|Col_2_Pin|Col_3_Pin|Col_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Seg_D_Pin Seg_C_Pin Seg_B_Pin Seg_A_Pin 
                           LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = Seg_D_Pin|Seg_C_Pin|Seg_B_Pin|Seg_A_Pin 
                          |LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  This function takes 1 adc sample, filter, and store in data array
	* @param  ADC_HandleTypeDef* hadc
	* @retval none
  */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
		adc_val = HAL_ADC_GetValue(&hadc1);
		float test = (float)(adc_val*3.0);           // if 12 bit resolution we need to divide by 2^12 !! 
		FIR_C(test, &filtered_adc);                  //filter ADC value
		data [sample] = filtered_adc;                //store filtered data in array
		sample ++;
		sample = sample % sampleNB;
}

/**
  * @brief  This function probes the keyPad
	* @param  None
	* @retval Returns the value pressed by the user (0-9, 10 for the * button, 11 for the # button)
  */

int readPad (){
	
	int keymap[4][3]=
	{
	{1, 2, 3},
	{4, 5, 6},
	{7, 8, 9},
	{10, 0, 11}
	};
	
	for (int i = 0; i < 4; i++){
		switch (i) {
			case 0 : 
				HAL_GPIO_WritePin(GPIOC, Row_1_Pin, GPIO_PIN_SET);
				HAL_Delay (1);
				if (HAL_GPIO_ReadPin(GPIOE, Col_1_Pin) == 1){
					HAL_GPIO_WritePin(GPIOC, Row_1_Pin, GPIO_PIN_RESET);
					return keymap [i][0];
				}
				else if (HAL_GPIO_ReadPin(GPIOE, Col_2_Pin) == 1){
					HAL_GPIO_WritePin(GPIOC, Row_1_Pin, GPIO_PIN_RESET);
					return keymap [i][1];
				}
				else if (HAL_GPIO_ReadPin(GPIOE, Col_3_Pin) == 1){
					HAL_GPIO_WritePin(GPIOC, Row_1_Pin, GPIO_PIN_RESET);
					return keymap [i][2];
				}
				HAL_GPIO_WritePin(GPIOC, Row_1_Pin, GPIO_PIN_RESET);
			break;
			
			case 1:
				HAL_GPIO_WritePin(GPIOC, Row_2_Pin, GPIO_PIN_SET);
				HAL_Delay (1);
				if (HAL_GPIO_ReadPin(GPIOE, Col_1_Pin) == 1){
					HAL_GPIO_WritePin(GPIOC, Row_2_Pin, GPIO_PIN_RESET);
					return keymap [i][0];
				}
				else if (HAL_GPIO_ReadPin(GPIOE, Col_2_Pin) == 1){
					HAL_GPIO_WritePin(GPIOC, Row_2_Pin, GPIO_PIN_RESET);
					return keymap [i][1];
				}
				else if (HAL_GPIO_ReadPin(GPIOE, Col_3_Pin) == 1){
					HAL_GPIO_WritePin(GPIOC, Row_2_Pin, GPIO_PIN_RESET);
					return keymap [i][2];
				}
				HAL_GPIO_WritePin(GPIOC, Row_2_Pin, GPIO_PIN_RESET);
			break;
			
			case 2:
				HAL_GPIO_WritePin(GPIOB, Row_3_Pin, GPIO_PIN_SET);
				HAL_Delay (1);
				if (HAL_GPIO_ReadPin(GPIOE, Col_1_Pin) == 1){
					HAL_GPIO_WritePin(GPIOB, Row_3_Pin, GPIO_PIN_RESET);
					return keymap [i][0];
				}
				else if (HAL_GPIO_ReadPin(GPIOE, Col_2_Pin) == 1){
					HAL_GPIO_WritePin(GPIOB, Row_3_Pin, GPIO_PIN_RESET);
					return keymap [i][1];
				}
				else if (HAL_GPIO_ReadPin(GPIOE, Col_3_Pin) == 1){
					HAL_GPIO_WritePin(GPIOB, Row_3_Pin, GPIO_PIN_RESET);
					return keymap [i][2];
				}
				HAL_GPIO_WritePin(GPIOB, Row_3_Pin, GPIO_PIN_RESET);
			break;
			
			case 3:
				HAL_GPIO_WritePin(GPIOB, Row_4_Pin, GPIO_PIN_SET);
				HAL_Delay (1);
				if (HAL_GPIO_ReadPin(GPIOE, Col_1_Pin) == 1){
					HAL_GPIO_WritePin(GPIOB, Row_4_Pin, GPIO_PIN_RESET);
					return keymap [i][0];
				}
				else if (HAL_GPIO_ReadPin(GPIOE, Col_2_Pin) == 1){
					HAL_GPIO_WritePin(GPIOB, Row_4_Pin, GPIO_PIN_RESET);
					return keymap [i][1];
				}
				else if (HAL_GPIO_ReadPin(GPIOE, Col_3_Pin) == 1){
					HAL_GPIO_WritePin(GPIOB, Row_4_Pin, GPIO_PIN_RESET);
					return keymap [i][2];
				}
				HAL_GPIO_WritePin(GPIOB, Row_4_Pin, GPIO_PIN_RESET);
			break;
			
			default:
				
			break;
			
		}
	}
	return -1;
}

/**
  * @brief  This function displays a integer on one of the digits of a 7 segiment, 4 digit led display
	* @param  num: integer number to be displayed 
						pos: the location in a 4 digit display,
  * @retval none
  */
void displayNum (int num, int pos) {       //function used to diplay a single digit on LED segments
	switch (num){
		case 0:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_SET);
			break;
		
		case 3:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_SET);
			break;
		
		case 4:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_SET);
			break;
		
		case 5:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_SET);
			break;
		
		case 6:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_SET);  
			break;
		
		case 7:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_RESET);
			break;
		
		case 8:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_SET);
			break;
		case 9:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_SET);
			break;
		case -1:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_RESET);
			break;
		default :
			break;
	}
		
		switch (pos){
			case 0:
				HAL_GPIO_WritePin(GPIOE, Dig_1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, Dig_2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_L_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, Seg_DP_Pin, GPIO_PIN_RESET);
			break;
			case 1:
				HAL_GPIO_WritePin(GPIOE, Dig_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, Dig_L_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, Seg_DP_Pin, GPIO_PIN_SET);
			break;
			case 2:
				HAL_GPIO_WritePin(GPIOE, Dig_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_L_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, Dig_4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, Seg_DP_Pin, GPIO_PIN_RESET);
			break;
			case 3 :
				HAL_GPIO_WritePin(GPIOE, Dig_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_L_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_4_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, Seg_DP_Pin, GPIO_PIN_RESET);
			break;
			case -1 :
				HAL_GPIO_WritePin(GPIOE, Dig_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_L_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, Seg_DP_Pin, GPIO_PIN_RESET);
			break;
			default :
				break;
		}
	
}
/**
  * @brief  This is the FIR function from LAB 1
						Filters data using FIR to eliminate unwanted noise
	* @param  Input: data to be filtered
						Output: filtered data
  * @retval none
  */
void FIR_C(float Input, float *Output) {   
	float coef [] = {0.2, 0.2, 0.2, 0.2, 0.2};          //coefficients -> SHOULD ADD UP TO 1 !
	float out = 0.0;
	for (int i = 4; i>0; i--){
		filterMemory[i] = filterMemory[i-1];
	}
	filterMemory [0] = Input;
	for (int i = 0; i<5; i++){
		out += coef[i]*filterMemory[i];
	}
	*Output = out;
}


/**
  * @brief  This is the C?math function from lab1
						calculates rms, max, min, min index, and max index from the given array
	* @param  inputArray: contains the samples to be analyzed
						outputArray: contains the array where the results are stored
						length: length of the input array
  * @retval none
  */
void C_math (float * inputArray, float * outputArray, int length){      //C_math function from lab 1

	float RMS=0;
	float maxVal = inputArray[0];
	float minVal = inputArray[0];
	int maxInd = 0;
	int minInd = 0;
	
	
	for (int i = 0; i< (length); i++){
		
		RMS += (inputArray[i])*(inputArray[i]);
		
		if ((inputArray[i]) > maxVal){
			maxVal = inputArray[i];
			maxInd = i;
		}
		if ((inputArray[i]) < minVal){
			minVal = (inputArray[i]);
			minInd = i;
		}
	}
	
	RMS = RMS/length;
	RMS = sqrt(RMS);
	
	outputArray [0]=RMS;
	outputArray [1]= maxVal;
	outputArray [2]= minVal;
	outputArray [3]= maxInd;
	outputArray [4]= minInd;
	
}

/**
  * @brief  detects a blue button press
  * @param  none
  * @retval returns 1 when button is pressed
						0 otherwise
  */
/*
int buttonPressed(){    
	if ( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0 ) == SET) {
		HAL_Delay(200);
		printf("button (pulled)!");
		return 1;
	}
	else 
	{
		return 0;
	}
}
*/
/**
  * @brief  function used to display a float value on the LED segments
  * @param  num: the floating number to be displayed at digits 1, 2, and 3
*         	mode: the int thats displayed at the firist digit to indicate rms/min/max mode 
  * @retval None
  */
void display (int mode, float num){     //
	float temp = num;
	int digit = (int) num;
	
	if (mode == -1){
		displayNum (-1, 0);
		displayNum (-1, 3);
		
		digit = digit % 10;
		displayNum (digit, 1); //display unit at location 1
		HAL_Delay(1);
		
		temp = num*10;
		digit = (int) temp;
		digit = digit % 10;
		displayNum (digit, 2); //display digit at location 2
		HAL_Delay(1);
		
	}
	else{
		displayNum (mode, 0); //display mode at location 0 (first LED display)
		HAL_Delay(1);
		
		
		digit = digit % 10;
		displayNum (digit, 1); //display unit at location 1
		HAL_Delay(1);
		
		temp = num*10;
		digit = (int) temp;
		digit = digit % 10;
		displayNum (digit, 2); //display digit at location 2
		HAL_Delay(1);
		
		temp = num*100;
		digit = (int) temp;
		digit = digit % 10;
		displayNum (digit, 3); //display digit at location 3
		HAL_Delay(1);
	}
}

/**
  * @brief  This update the padEntries table that stores the digits pad inputs
	* @param  newEnt : newly inputted digit
  * @retval None
  */

void updateEnt (int newEnt) {
	
	for (int i = 3; i>0; i--){
		padEntries[i] = padEntries [i-1];
	}
	padEntries [0] = newEnt;
}



/**
  * @brief  This is the controller function. It adjusts the PWM duty cycle according to the desired value
	* @param  current_period : the current duty cycle of the PWM
  * @retval None
  */

void set_highPeriods(int current_period){

	float rms = mathResults [0];
	float diff = dispNum - rms;

	if (dispNum < 1.4){
			if(diff > 0){
				if (diff > 0.5){
					current_period += 20;
				}
				if (diff > 0.2){
					current_period += 10;
				}
				else{
					current_period += 1;
				}
				if( current_period >= 1000 ){
					current_period = 1000;
				}
			}
			else if (diff < 0){
				if (diff < -0.5){
					current_period -= 20;
				}
				if (diff < -0.2){
					current_period -= 10;
				}
				else{
					current_period -= 1;
				}
				current_period -= 1;
				if( current_period <= 0 ){
					current_period = 0;
				}
			}
	}
	
	
	else {
		if(diff > 0){
				if (diff > 0.5){
					current_period += 40;
				}
				if (diff > 0.2){
					current_period += 40;
				}
				else{
					current_period += 10;
				}
				if( current_period >= 1000 ){
					current_period = 1000;
				}
			}
			else if (diff < 0){
				if (diff < -0.5){
					current_period -= 20;
				}
				if (diff < -0.2){
					current_period -= 10;
				}
				else{
					current_period -= 1;
				}
				current_period -= 1;
				if( current_period <= 0 ){
					current_period = 0;
				}
			}
		
	}
	
	
	if(diff*diff < 0.04){
		correctPeriod = 20;
	}
	else if(diff*diff < 0.01){
		correctPeriod = 3000;
	}
	else{
		correctPeriod = 10;
	}
	
	highPeriods = (int)current_period;
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* Start_ADC_calc function */
void Start_ADC_calc(void const * argument)
{
  /* USER CODE BEGIN Start_ADC_calc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_ADC_calc */
}

/* Start_display function */
void Start_display(void const * argument)
{
  /* USER CODE BEGIN Start_display */
  /* Infinite loop */
  for(;;)
  {
		
		
		switch (state){
			
			case WKEY1:
					display (-1, dispNum);
			break;
			
			case WKEY2:
				display (-1, dispNum);
			break;
			
			case WENTER:
				display (-1, dispNum);
			break;
			
			case DISPLAY:
				display (0, mathResults [0]);
			break;
		
			case SLEEP :
				
			break;
		}
		
    osDelay(10);
  }
  /* USER CODE END Start_display */
}

/* Start_PWM_control function */
void Start_PWM_control(void const * argument)
{
  /* USER CODE BEGIN Start_PWM_control */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_PWM_control */
}

/* Start_state_control function */
void Start_state_control(void const * argument)
{
  /* USER CODE BEGIN Start_state_control */
  /* Infinite loop */
	
  for(;;)
  {
		dispNum = padEntries[0] + ((float)padEntries[1]/10);
		
		switch (state){
			
			case WKEY1 :
				if (padVal >= 0 && padVal < 10 && padFlag == 0){
					padEntries[0]=padVal;
					padFlag = 1;
				}
				else if (padVal == -1 && padFlag == 1){
					padFlag = 0;
					state = WKEY2;
				}
			break;
			
			case WKEY2 :
				if (padVal >= 0 && padVal < 10 && padFlag == 0){
					padEntries[1]=padVal;
					padFlag = 1;
				}
				else if (padVal == 10 && padFlag == 0){
					padEntries[0] = 0;
					padFlag = -1;
				}
				else if (padVal == -1){
					if (padFlag == 1){
						padFlag = 0;
						state = WENTER;
					}
					else if (padFlag == -1){
						padFlag = 0;
						state = WKEY1;
					}
				}
			break;
			
			case WENTER :
				if (padVal == 11 && padFlag == 0){
					padFlag = 1;
				}
				else if (padVal == 10 && padFlag == 0){
					padEntries[1] = 0;
					padFlag = -1;
				}
				else if (padVal == -1){
					if (padFlag == 1){
						padFlag = 0;
						
						
						state = DISPLAY;
					}
					else if (padFlag == -1){
						padFlag = 0;
						state = WKEY2;
					}
				}
			break;
			
			case DISPLAY :
				if (padVal == 11 && padFlag == 0){
					padFlag = 1;
				}
				else if (padVal == -1){
					if (padFlag == 1){
						padFlag = 0;
						padEntries[0] = 0;
						padEntries [1] = 0;
						state = WKEY1;
					}
				}
				
				C_math (&data[0], &mathResults [0], sampleNB);                //perform math operation on data to get RMS, min and max values
				
				//correctCounter = 0;
				
				set_highPeriods(highPeriods);
				
				htim3.Instance->CCR1 = highPeriods;									// sets the number of periods that the pwm wave is set to high
				
			break;
			
			case SLEEP :
				if (padVal == 11 && padFlag == 0){
					padFlag = 1;
				}
				else if (padVal == -1){
					if (padFlag == 1){
						padFlag = 0;
						padEntries[0] = 0;     //reset pad entries
						padEntries[1] = 0;
						state = WKEY1;
					}
				}
			break;
			
		}
		
		
		
    osDelay(1);
  }
  /* USER CODE END Start_state_control */
}

/* Start_pad_read function */
void Start_pad_read(void const * argument)
{
  /* USER CODE BEGIN Start_pad_read */
  /* Infinite loop */
  for(;;)
  {
		
		padVal = readPad();
		
    osDelay(1);
  }
  /* USER CODE END Start_pad_read */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
