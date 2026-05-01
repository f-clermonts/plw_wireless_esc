/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32_mc_common_it.c
  * @brief   Common interrupt handlers for the simplified TX application.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "main.h"
#include "stm32g4xx_it.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

void HardFault_Handler(void);
void SysTick_Handler(void);

void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

void HardFault_Handler(void)
{
  while (true)
  {
  }
}

void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}
