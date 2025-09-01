    /* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "lpuart.h"
#include "ADC.h"
#include "delay.h"
#include "stm32l4a6xx.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void StageOne(void);
void led_Config(void);
void button_Config(void);
void relay_GPIO_Config (void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  SysTick_Init();
  ADC_init();
  LPUART1_init();
  StageOne();
  led_Config();
  button_Config();
  relay_GPIO_Config();

	while (1) {
		if ((GPIOC->IDR & GPIO_IDR_ID13) != 0) { // button is pressed (active high)
			GPIOB->ODR |= GPIO_ODR_OD0;              // turn relay ON
			// test led on
			GPIOC->ODR |= GPIO_ODR_OD0;
		} else {
			GPIOB->ODR &= ~GPIO_ODR_OD0;             // turn relay OFF
			GPIOC->ODR &= ~(GPIO_ODR_OD0);			 // turn led off
		}
		if (samples_ready) {
			// Calculate min, max, and average from the 20 samples
			uint16_t min = ADC_array_minVal((uint16_t*) ADC_Samples, 20);
			uint16_t max = ADC_array_maxVal((uint16_t*) ADC_Samples, 20);
			uint16_t avg = ADC_array_avgVal((uint16_t*) ADC_Samples, 20);

			// Display each value in its respective row
			ADC_int_to_string("MIN", min, "2;5H");
			ADC_int_to_string("MAX", max, "3;5H");
			ADC_int_to_string("AVG", avg, "4;5H");

			ADC_display_current(avg, 10000, "5;5H");  // RE = 10 Ω

			samples_ready = 0;  // Reset the flag to collect new samples
			ADC1->CR |= ADC_CR_ADSTART; // Start a new sequence of conversions

		}


  /* USER CODE END 3 */
}
}
/* -----------------------------------------------------------------------------
 * function : void StageOne(void)
 * INs : none
 * OUTs : none
 * action :
 * authors :
 * version : 1
 * date : 250426
 * -------------------------------------------------------------------------- */
void StageOne(void) {
    LPUART_ESC_Print("2J", NULL);       // Clear screen
    LPUART_ESC_Print("?25l", NULL);     // Hide cursor

    LPUART_ESC_Print("1;5H", NULL);     // Row 1, column 5
    LPUART_Print("ADC   counts   volts");

    LPUART_ESC_Print("2;5H", NULL);
    LPUART_Print("MIN   0000     0.000 V");

    LPUART_ESC_Print("3;5H", NULL);
    LPUART_Print("MAX   0000     0.000 V");

    LPUART_ESC_Print("4;5H", NULL);
    LPUART_Print("AVG   0000     0.000 V");
}
/* -----------------------------------------------------------------------------
 * function : void led_config (void)	void relay_GPIO_Config (void)
 * INs : none
 * OUTs : none
 * action : configures PC0 as a high-speed push-pull output to control LED
 * authors :
 * version : 1
 * date : 250426
 * ---------------------------------------------------------------------------*/
void led_Config(void) {
// Configure GPIO Pins (PC0, PC1, PC2, PC3)
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOCEN);		// Enable clock to GPIOC
	GPIOC->MODER &= ~(GPIO_MODER_MODE0);		// Clear mode bits for PC0
	GPIOC->MODER |= (GPIO_MODER_MODE0_0);		// Set PC0 to Output mode (01)
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT0);		// Set output to push-pull (0)
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD0);		// Disable pull-up/pull-down
	GPIOC->OSPEEDR |= (							// high speed output (11)
			(3 << GPIO_OSPEEDR_OSPEED0_Pos));
	GPIOC->BRR = (GPIO_PIN_0);		// reset LED to initially off
}

void relay_GPIO_Config (void){
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN);
	GPIOB->MODER &= ~(GPIO_MODER_MODE1);
	GPIOB->MODER |= (GPIO_MODER_MODE1_0);
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT1);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD1);
	GPIOB->OSPEEDR |= (
			(3 << GPIO_OSPEEDR_OSPEED1_Pos));
	GPIOB->BRR = (GPIO_PIN_1);
}
/* -----------------------------------------------------------------------------
 * function : void button_Config(void)
 * INs : none
 * OUTs : none
 * action :
 * authors :
 * version : 1
 * date : 250426
 * ---------------------------------------------------------------------------*/
void button_Config(void) {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; // enable AHB2
	GPIOC->MODER &= ~GPIO_MODER_MODE13; // Set PC13 to Input mode
}
/* -----------------------------------------------------------------------------
 * function : void SystemClock_Config(void)
 * INs : none
 * OUTs : none
 * action :
 * authors :
 * version : 1
 * date : 250426
 * -------------------------------------------------------------------------- */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
/* -----------------------------------------------------------------------------
 * function : Error_Handler(void)
 * INs : none
 * OUTs : none
 * action :
 * authors : CUBEIDE
 * version : 1
 * date : 250423
 * -------------------------------------------------------------------------- */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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


