/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

#include <string.h>
#include <stdio.h>
#include <inttypes.h>

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
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void JV_ADC_Init(void);
static void JV_ADC_Start(void);
static uint16_t JV_ADC_GetResult(void);
void Delay_ms(uint32_t delay_ms);

static void JV_TIM16_Init(void);
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
  JV_ADC_Init();
  JV_TIM16_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
     uint32_t start_time_ms = HAL_GetTick();
     JV_ADC_Start();
     uint32_t adc = JV_ADC_GetResult();
     printf("FRQ: %lu\r\n", (adc + 1) * 8);

     uint64_t freq = (adc + 1) * 8; // approx 1 to 32K
     uint64_t period = 48000000ULL / freq;
     uint32_t prescale = period >> 16;
     period = period / (uint64_t) (prescale + 1);

     TIM16->PSC = prescale;
     TIM16->ARR = period-1;
     TIM16->CCR1 = period/2;

     // wait remainder of 100ms
     while ( (HAL_GetTick() - start_time_ms) < 100)
     {
         // spin wait
     }

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void JV_ADC_Init(void)
{
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    // Reset the ADC by disabling it
    ADC1->CR  = (uint32_t) 0x00000000;     // reset the ADC

    // ADC control register (ADC_CR)
    // [31]  ADCAL   = 0: No ADC Calibration command
    // [4]   ADSTP   = 0: No ADC stop conversion commanded
    // [2]   ADSTART = 0: No ADC conversion command
    // [1]   ADDIS   = 0: No ADC Disable command
    // [0]   ADEN    = 1: ADC is enabled
    ADC1->CR  = (uint32_t) 0x00000001;     // Enable the ADC

    // ADC interrupt and status register (ADC_ISR)
    ADC1->ISR = (uint32_t) 0x0000009F;     // clear all interrupt flags

    // ADC interrupt enable register (ADC_IER)
    ADC1->IER = (uint32_t) 0x00000000;     // disable all interrupts

    // ADC configuration register 1 (ADC_CFGR1)
    // [30:26] AWDCH   = 00000: Watchdog channel
    // [23]    AWDEN   = 0: Watchdog is disabled
    // [22]    AWDSGL  = 0: Watchdog all channels
    // [16]    DISCEN  = 0: Discontinuous mode disabled
    // [15]    AUTOOFF = 0: Auto-off mode disabled
    // [14]    WAIT    = 0: Wait conversion mode off
    // [13]    CONT    = 0: Single conversion mode
    // [12]    OVRMOD  = 0: preserve old data on overrun
    // [11:10] EXTEN   = 00: Hardware trigger detection disabled (conversions can be started by software)
    // [8:6]   EXTSEL  = 000: External TRG0 [don't care]
    // [5]     ALIGN   = 0: Right alignment
    // [4:3]   RES     = 00: 12 bit resolution
    // [2]     SCANDIR = 0: Upward scan (from CHSEL0 to CHSEL17) [don't care]
    // [1]     DMACFG  = 0: DMA one shot mode selected [don't care]
    // [0]     DMAEN   = 0: DMA disabled
    ADC1->CFGR1 = (uint32_t) 0x00000000;

    // ADC configuration register 2 (ADC_CFGR2)
    //[31:30] CKMODE = 00: ADCCLK (Asynchronous clock mode),
    ADC1->CFGR2 = (uint32_t) 0x00000000;

    // ADC sampling time register (ADC_SMPR)
    //[2:0] SMP = 111: 239.5 ADC clock cycles
    ADC1->SMPR = (uint32_t) 0x00000007;

    // ADC watchdog threshold register (ADC_TR)
    // [27:16] HT = 0xFFF: Analog watchdog higher threshold
    // [11:0]  LT = 0x000: Analog watchdog lower threshold
    ADC1->TR = (uint32_t) 0xFFFF0000;

    // ADC channel selection register (ADC_CHSELR)
    // Only input channel 0 is selected for conversion
    ADC1->CHSELR = (uint32_t) 0x00000001;

    // ADC common configuration register (ADC_CCR)
    // [23] TSEN   = 0: Temperature sensor disabled
    // [22] VREFEN = 0: VREFINT disabled
    ADC1_COMMON->CCR = (uint32_t) 0x00000000;


}

/**
  * @brief ADC Start Conversion
  * @param None
  * @retval None
  */
static void JV_ADC_Start(void)
{
    // ADC control register (ADC_CR)
    // [31]  ADCAL   = 0: No ADC Calibration command
    // [4]   ADSTP   = 0: No ADC stop conversion commanded
    // [2]   ADSTART = 1: Start the ADC
    // [1]   ADDIS   = 0: No ADC Disable command
    // [0]   ADEN    = 1: ADC is enabled
    ADC1->CR  = (uint32_t) 0x00000005;  // Start the ADC conversions

    /* Clear all interrupt flags */
    ADC1->ISR = (uint32_t) 0x0000009F;
}

/**
  * @brief ADC Wait for Conversion Complete and get value
  * @param None
  * @retval None
  */
static uint16_t JV_ADC_GetResult(void)
{
    /* Wait for End of Conversion Flag */
    while ((ADC1->ISR & ADC_ISR_EOC) == 0)
    {
        // spin wait for EOC flag
        // TODO: Add timeout logic
    }

    /* Clear all interrupt flags */
    ADC1->ISR = (uint32_t) 0x0000009F;

    /* return last conversion value */
    return (uint16_t) ADC1->DR;
}



/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void JV_TIM16_Init(void)
{
    /* Peripheral clock enable */
    __HAL_RCC_TIM16_CLK_ENABLE();

    /* Config Timer16 for differential PWM output */
    TIM16->CR1   = 0x0000;   // disable timer
    TIM16->CR2   = 0x0000;
    TIM16->DIER  = 0x0000;
    TIM16->SR    = 0x0003;    // clear interrupt flags
    TIM16->CCMR1 = 0x0068;
    TIM16->CCER  = 0x0005;   // Enable CH1 and CH1N
    TIM16->CNT   = 0x0000;   // clear counter
    TIM16->PSC   =     89;
    TIM16->ARR   =   31;  // period (autoreload value)
    TIM16->RCR   = 0x000;
    TIM16->CCR1  =   16;  // PWM Value
    TIM16->BDTR  = 0xA000;
    TIM16->DCR   = 0x0000;
    TIM16->DMAR  = 0x0001;

    /* Enable Timer16 */
    TIM16->CR1   = 0x0001;   // enable timer
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
  HAL_GPIO_WritePin(LED_GRN_GPIO_Port, LED_GRN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 = BTN_USER */
  GPIO_InitStruct.Pin = BTN_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 = LED_GRN */
  GPIO_InitStruct.Pin = LED_GRN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GRN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 = TIM16_CH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF5_TIM16;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 = TIM16_CH1N */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM16;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 = ADC_IN0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void Delay_ms(uint32_t delay_ms)
{
    /**
     * This should correctly handle SysTick roll-overs.
     * See:  https://stackoverflow.com/questions/61443/
     */
    uint32_t start_time_ms = HAL_GetTick();
    while ( (HAL_GetTick() - start_time_ms) < delay_ms)
    {
        // spin wait
    }
    return;
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
