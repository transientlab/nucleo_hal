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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFERSIZE            128
#define SCREEN_FONT		     Font_7x10

#define TIM1_PWM_FREQ_4K      7500
#define TIM1_PWM_FREQ_8K      15000
#define TIM1_PWM_FREQ_16K     30000

#define TIM1_PWM_4K_25DUTY      ( TIM1_PWM_FREQ_4K * 0.25 )
#define TIM1_PWM_4K_50DUTY      ( TIM1_PWM_FREQ_4K * 0.50 )
#define TIM1_PWM_4K_75DUTY      ( TIM1_PWM_FREQ_4K * 0.75 )

#define TIM1_PWM_8K_25DUTY      ( TIM1_PWM_FREQ_8K * 0.25 )
#define TIM1_PWM_8K_50DUTY      ( TIM1_PWM_FREQ_8K * 0.50 )
#define TIM1_PWM_8K_75DUTY      ( TIM1_PWM_FREQ_8K * 0.75 )

#define TIM1_PWM_16K_25DUTY      ( TIM1_PWM_FREQ_16K * 0.25 )
#define TIM1_PWM_16K_50DUTY      ( TIM1_PWM_FREQ_16K * 0.50 )
#define TIM1_PWM_16K_75DUTY      ( TIM1_PWM_FREQ_16K * 0.75 )
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t timer_ar_value = TIM1_PWM_FREQ_4K;
//uint32_t i = 0;
uint32_t j = 0;
uint32_t tmp[4];
uint32_t adc_buffer[BUFFERSIZE];
uint32_t dac_buffer[BUFFERSIZE];
char buffer [32];
static volatile uint32_t* inbufferPtr;
static volatile uint32_t* outbufferPtr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void processDSP();
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
//  inbufferPtr = &adc_buffer[0];
//  outbufferPtr = &dac_buffer[0];
//
//  HAL_TIM_Base_Start_IT( &htim1 );
//  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );
//  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2 );
//  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 );
//  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1 );
//  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2 );
//  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3 );
  HAL_TIM_Base_Start(&htim6);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, BUFFERSIZE);
  HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t *)dac_buffer, BUFFERSIZE, DAC_ALIGN_12B_R);
  ssd1306_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (j>127) {
		  HAL_GPIO_TogglePin(GPIOB, LED_Pin);
		  j = 0;
		  ssd1306_Fill(Black);
		  for(int i=0; i<BUFFERSIZE/2; i++) {
		   ssd1306_DrawPixel(2*i, (64*adc_buffer[i])/4100, White);
		  }
		  ssd1306_UpdateScreen();
	  }
	  else {
		  j ++;
	  }

//	HAL_ADC_PollForConversion(&hadc1, 100);
//	adc_buffer = 3.4 * HAL_ADC_GetValue(&hadc1) / 4095;
//	gcvt((float)adc_buffer[0], 5, buffer);
//
//	i = 10000; while(i--);
//
//	HAL_DACEx_DualSetValue(&hdac1, DAC_ALIGN_12B_L, adc_buffer[0], adc_buffer[1]);
//	HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t*)dac_ch1_buffer, 1, DAC_ALIGN_12B_R);
//	HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_2, (uint32_t*)dac_ch2_buffer, 1, DAC_ALIGN_12B_R);
//	HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
//	HAL_DAC_Start(&hdac1, DAC1_CHANNEL_2);
//
//	i = 10000; while(i--);
	  tmp[2] = j;
//  ssd1306_Fill(Black);
//  for(i=0; i<BUFFERSIZE; i++) {
//	  ssd1306_DrawPixel(i, (2+64*adc_buffer[i/2])/4100, White);
//  }
//  ssd1306_UpdateScreen();
  // ssd1306_SetCursor(0, 0); 		itoa(adc_buffer[0], buffer, 16); 	ssd1306_WriteString(buffer, SCREEN_FONT, White);
  // ssd1306_SetCursor(64, 0); 	itoa(tmp[0], buffer, 16); 			ssd1306_WriteString(buffer, SCREEN_FONT, White);
  // ssd1306_SetCursor(0, 12); 	itoa(adc_buffer[1], buffer, 16); 	ssd1306_WriteString(buffer, SCREEN_FONT, White);
  // ssd1306_SetCursor(64, 12); 	itoa(tmp[1], buffer, 16); 			ssd1306_WriteString(buffer, SCREEN_FONT, White);
  // ssd1306_SetCursor(0, 24); 	itoa(adc_buffer[2], buffer, 16); 	ssd1306_WriteString(buffer, SCREEN_FONT, White);
  // ssd1306_SetCursor(64, 24); 	itoa(tmp[2], buffer, 16); 			ssd1306_WriteString(buffer, SCREEN_FONT, White);

  processDSP();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if ( timer_ar_value == TIM1_PWM_FREQ_4K )
  {
    timer_ar_value = TIM1_PWM_FREQ_8K;
    __HAL_TIM_SET_AUTORELOAD( &htim1, TIM1_PWM_FREQ_8K );
    __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, TIM1_PWM_8K_25DUTY );
    __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, TIM1_PWM_8K_50DUTY );
    __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, TIM1_PWM_8K_75DUTY );
  }
  else if ( timer_ar_value == TIM1_PWM_FREQ_8K )
  {
    timer_ar_value = TIM1_PWM_FREQ_16K;
    __HAL_TIM_SET_AUTORELOAD( &htim1, TIM1_PWM_FREQ_16K );
    __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, TIM1_PWM_16K_25DUTY );
    __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, TIM1_PWM_16K_50DUTY );
    __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, TIM1_PWM_16K_75DUTY );
  }
  else if ( timer_ar_value == TIM1_PWM_FREQ_16K )
  {
    timer_ar_value = TIM1_PWM_FREQ_4K;
    __HAL_TIM_SET_AUTORELOAD( &htim1, TIM1_PWM_FREQ_4K );
    __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, TIM1_PWM_4K_25DUTY );
    __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, TIM1_PWM_4K_50DUTY );
    __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, TIM1_PWM_4K_75DUTY );
  }
  else
  {
    timer_ar_value = TIM1_PWM_FREQ_4K;
    __HAL_TIM_SET_AUTORELOAD( &htim1, TIM1_PWM_FREQ_4K );
    __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, TIM1_PWM_4K_25DUTY );
    __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, TIM1_PWM_4K_50DUTY );
    __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, TIM1_PWM_4K_75DUTY );
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	inbufferPtr = &adc_buffer[BUFFERSIZE/2];
	outbufferPtr = &dac_buffer[0];
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  inbufferPtr = &adc_buffer[0];
  outbufferPtr = &dac_buffer[BUFFERSIZE/2];
}

void processDSP() {
  for(int i=0; i<BUFFERSIZE/2; i++) {
    outbufferPtr[i] = inbufferPtr[i];
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
