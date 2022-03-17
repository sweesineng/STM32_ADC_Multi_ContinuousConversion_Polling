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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LL_Driver
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ADC_Val[5];
uint32_t VRefInt, ADC_INP0, ADC_INP1, VBat, TSensor;
uint8_t debug = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
#ifdef LL_Driver
  /* Only applicable for internal channel like Vrefint, Vbat & TempSensor */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC3),
		  LL_ADC_CHANNEL_VREFINT | LL_ADC_CHANNEL_VBAT |
		  LL_ADC_CHANNEL_TEMPSENSOR);
  LL_ADC_StartCalibration(ADC3, LL_ADC_CALIB_OFFSET, LL_ADC_SINGLE_ENDED);
  while(LL_ADC_IsCalibrationOnGoing(ADC3) != RESET);
  /* Set ADC Channel, this is important */
  LL_ADC_SetChannelPreSelection(ADC3, LL_ADC_CHANNEL_VREFINT);
  LL_ADC_SetChannelPreSelection(ADC3, LL_ADC_CHANNEL_0);	// PC2
  LL_ADC_SetChannelPreSelection(ADC3, LL_ADC_CHANNEL_1);	// PC3
  LL_ADC_SetChannelPreSelection(ADC3, LL_ADC_CHANNEL_VBAT);
  LL_ADC_SetChannelPreSelection(ADC3, LL_ADC_CHANNEL_TEMPSENSOR);
#else
  if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED)
		  != HAL_OK)
  {
	  return HAL_ERROR;
  }
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef LL_Driver
	  LL_ADC_Enable(ADC3);
	  while (LL_ADC_IsActiveFlag_ADRDY(ADC3) == 0);
	  LL_ADC_REG_StartConversion(ADC3);
	  for (uint8_t i = 0; i < sizeof(ADC_Val); i++)
	  {
		  while(LL_ADC_IsActiveFlag_EOC(ADC3) == 0);
		  LL_ADC_ClearFlag_EOC(ADC3);
		  ADC_Val[i] = LL_ADC_REG_ReadConversionData16(ADC3);
	  }

	  LL_ADC_REG_StopConversion(ADC3);

	  VRefInt 	= __LL_ADC_CALC_VREFANALOG_VOLTAGE(ADC_Val[0],
			  LL_ADC_GetResolution(ADC3));
	  ADC_INP0 	= __LL_ADC_CALC_DATA_TO_VOLTAGE(VRefInt, ADC_Val[1],
			  LL_ADC_GetResolution(ADC3));
	  ADC_INP1 	= __LL_ADC_CALC_DATA_TO_VOLTAGE(VRefInt, ADC_Val[2],
			  LL_ADC_GetResolution(ADC3));
	  VBat 		= __LL_ADC_CALC_DATA_TO_VOLTAGE(VRefInt, ADC_Val[3],
			  LL_ADC_GetResolution(ADC3));
	  TSensor	= __LL_ADC_CALC_TEMPERATURE(VRefInt, ADC_Val[4],
			  LL_ADC_GetResolution(ADC3));

	  LL_mDelay(1000);
#else
	  HAL_ADC_Start(&hadc3);
	  for (uint8_t i = 0; i < sizeof(ADC_Val); i++)
	  {
		  HAL_ADC_PollForConversion(&hadc3, 100);
		  ADC_Val[i] = HAL_ADC_GetValue(&hadc3);
	  }
	  HAL_ADC_Stop(&hadc3);

	  VRefInt 	= __HAL_ADC_CALC_VREFANALOG_VOLTAGE(ADC_Val[0],
			  hadc3.Init.Resolution);
	  ADC_INP0 	= __HAL_ADC_CALC_DATA_TO_VOLTAGE(VRefInt, ADC_Val[1],
			  hadc3.Init.Resolution);
	  ADC_INP1 	= __HAL_ADC_CALC_DATA_TO_VOLTAGE(VRefInt, ADC_Val[2],
			  hadc3.Init.Resolution);
	  VBat 		= __HAL_ADC_CALC_DATA_TO_VOLTAGE(VRefInt, ADC_Val[3],
			  hadc3.Init.Resolution);
	  TSensor	= __HAL_ADC_CALC_TEMPERATURE(VRefInt, ADC_Val[4],
			  hadc3.Init.Resolution);

	  HAL_Delay(1000);
#endif

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(32);
  LL_RCC_HSI_SetDivider(LL_RCC_HSI_DIV1);
  LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSI);
  LL_RCC_PLL1P_Enable();
  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL1_SetM(4);
  LL_RCC_PLL1_SetN(30);
  LL_RCC_PLL1_SetP(2);
  LL_RCC_PLL1_SetQ(2);
  LL_RCC_PLL1_SetR(2);
  LL_RCC_PLL1_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL1_IsReady() != 1)
  {
  }

   /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
   LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);
  LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_1);
  LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_1);
  LL_SetSystemCoreClock(240000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  LL_RCC_PLL2P_Enable();
  LL_RCC_PLL2_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL2_SetVCOOutputRange(LL_RCC_PLLVCORANGE_MEDIUM);
  LL_RCC_PLL2_SetM(4);
  LL_RCC_PLL2_SetN(9);
  LL_RCC_PLL2_SetP(4);
  LL_RCC_PLL2_SetQ(2);
  LL_RCC_PLL2_SetR(2);
  LL_RCC_PLL2_SetFRACN(3072);
  LL_RCC_PLL2FRACN_Enable();
  LL_RCC_PLL2_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL2_IsReady() != 1)
  {
  }

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PLL2P);

  /* Peripheral clock enable */
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_ADC3);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  /**ADC3 GPIO Configuration
  PC2_C   ------> ADC3_INP0
  PC3_C   ------> ADC3_INP1
  */
  LL_SYSCFG_OpenAnalogSwitch(LL_SYSCFG_ANALOG_SWITCH_PC2|LL_SYSCFG_ANALOG_SWITCH_PC3);

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_16B;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC3, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_5RANKS;
  ADC_REG_InitStruct.SequencerDiscont = DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_Init(ADC3, &ADC_REG_InitStruct);
  LL_ADC_SetOverSamplingScope(ADC3, LL_ADC_OVS_DISABLE);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV2;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC3), &ADC_CommonInitStruct);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC3);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC3);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_VREFINT);
  LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_387CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC3, LL_ADC_CHANNEL_VREFINT, LL_ADC_SINGLE_ENDED);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_0);
  LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_387CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC3, LL_ADC_CHANNEL_0, LL_ADC_SINGLE_ENDED);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_1);
  LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_387CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC3, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_VBAT);
  LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_VBAT, LL_ADC_SAMPLINGTIME_387CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC3, LL_ADC_CHANNEL_VBAT, LL_ADC_SINGLE_ENDED);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_5, LL_ADC_CHANNEL_TEMPSENSOR);
  LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_387CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC3, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
