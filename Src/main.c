/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "parameters_conversion.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint16_t bus_raw;
  uint16_t current_u_raw;
  uint16_t current_v_raw;
  uint16_t pot_raw;
  int32_t current_u_offset;
  int32_t current_v_offset;
  int32_t current_u;
  int32_t current_v;
  uint16_t compare;
  uint32_t frequency_hz;
  uint32_t requested_frequency_hz;
  GPIO_PinState button_last_raw;
  GPIO_PinState button_stable_state;
  uint32_t button_last_change_tick;
  bool tx_enabled;
  bool force_enable;
  bool manual_mode;
  bool stream_enabled;
  uint32_t last_debug_tick;
  uint32_t dead_time_ns;
} WirelessTxState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_FREQ_MIN_HZ                  100000U
#define TX_FREQ_MAX_HZ                  300000U
#define TX_POT_MAX_RAW                  4095U
#define TX_BUTTON_Pin                   GPIO_PIN_10
#define TX_BUTTON_GPIO_Port             GPIOC
#define TX_BUTTON_PRESSED_STATE         GPIO_PIN_RESET
#define TX_BUTTON_DEBOUNCE_MS           30U
#define TX_DEBUG_PERIOD_MS              250U
#define TX_CURRENT_OFFSET_SAMPLES       64U
#define TX_DAC_OVERCURRENT_THRESHOLD    3000U
#define TX_CLI_BUFFER_LENGTH            64U
#define TX_DT_MAX_NS                    11854U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;
COMP_HandleTypeDef hcomp4;

CORDIC_HandleTypeDef hcordic;

DAC_HandleTypeDef hdac3;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
static WirelessTxState_t g_tx_state;
static char g_cli_buffer[TX_CLI_BUFFER_LENGTH];
static uint8_t g_cli_length;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_COMP4_Init(void);
static void MX_CORDIC_Init(void);
static void MX_DAC3_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static void WirelessTx_Init(void);
static void WirelessTx_Loop(void);
static void WirelessTx_ApplyOutputs(uint32_t frequency_hz, bool enable_outputs);
static uint32_t WirelessTx_MapPotToFrequency(uint16_t pot_raw);
static uint32_t WirelessTx_ClampFrequency(uint32_t frequency_hz);
static uint16_t WirelessTx_FrequencyToPeriod(uint32_t frequency_hz);
static void WirelessTx_ProcessButton(void);
static bool WirelessTx_ButtonIsPressed(GPIO_PinState state);
static void WirelessTx_SampleFeedback(void);
static void WirelessTx_CalibrateCurrentOffsets(void);
static void WirelessTx_PrintDebug(void);
static void WirelessTx_ProcessCli(void);
static void WirelessTx_HandleCommand(char *command);
static void WirelessTx_Log(const char *format, ...);
static void WirelessTx_PrintPrompt(void);
static uint8_t WirelessTx_NsToDtg(uint32_t dt_ns);
static void WirelessTx_ApplyDeadTime(uint32_t dt_ns);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void WirelessTx_Init(void)
{
  g_tx_state.stream_enabled = true;
  g_tx_state.force_enable = false;
  g_tx_state.dead_time_ns = SW_DEADTIME_NS;
  g_tx_state.manual_mode = false;
  g_tx_state.frequency_hz = TX_FREQ_MIN_HZ;
  g_tx_state.requested_frequency_hz = TX_FREQ_MIN_HZ;
  g_tx_state.button_last_raw = HAL_GPIO_ReadPin(TX_BUTTON_GPIO_Port, TX_BUTTON_Pin);
  g_tx_state.button_stable_state = g_tx_state.button_last_raw;
  g_tx_state.button_last_change_tick = HAL_GetTick();

  WirelessTx_Log("\r\nwireless-tx boot\r\n");
  WirelessTx_Log("init: adc calibration start\r\n");
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  WirelessTx_Log("init: opamp start\r\n");
  if (HAL_OPAMP_Start(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_OPAMP_Start(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_OPAMP_Start(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }

  WirelessTx_Log("init: dac start\r\n");
  if (HAL_DAC_Start(&hdac3, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DAC_Start(&hdac3, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, TX_DAC_OVERCURRENT_THRESHOLD) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, TX_DAC_OVERCURRENT_THRESHOLD) != HAL_OK)
  {
    Error_Handler();
  }

  WirelessTx_Log("init: comparator start\r\n");
  if (HAL_COMP_Start(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_COMP_Start(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_COMP_Start(&hcomp4) != HAL_OK)
  {
    Error_Handler();
  }

  WirelessTx_Log("init: pwm outputs start\r\n");
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  WirelessTx_ApplyOutputs(TX_FREQ_MIN_HZ, false);
  WirelessTx_Log("init: current offset calibration\r\n");
  WirelessTx_CalibrateCurrentOffsets();
  WirelessTx_Log("init: complete, mode=pot tx=off freq=100..300kHz duty=50%% stream=on\r\n");
  WirelessTx_Log("type 'help' for commands\r\n");
  WirelessTx_PrintPrompt();
}

static void WirelessTx_Loop(void)
{
  WirelessTx_ProcessCli();
  WirelessTx_ProcessButton();
  WirelessTx_SampleFeedback();

  g_tx_state.tx_enabled = g_tx_state.force_enable;

  if (g_tx_state.manual_mode)
  {
    g_tx_state.frequency_hz = g_tx_state.requested_frequency_hz;
  }
  else
  {
    g_tx_state.frequency_hz = WirelessTx_MapPotToFrequency(g_tx_state.pot_raw);
  }

  WirelessTx_ApplyOutputs(g_tx_state.frequency_hz, g_tx_state.tx_enabled);

  if (g_tx_state.stream_enabled && ((HAL_GetTick() - g_tx_state.last_debug_tick) >= TX_DEBUG_PERIOD_MS))
  {
    g_tx_state.last_debug_tick = HAL_GetTick();
    WirelessTx_PrintDebug();
  }
}

static void WirelessTx_ApplyOutputs(uint32_t frequency_hz, bool enable_outputs)
{
  uint16_t arr = WirelessTx_FrequencyToPeriod(frequency_hz);
  uint16_t compare = arr / 2U;
  uint16_t trigger_compare = (arr > HTMIN) ? (uint16_t)(arr - HTMIN) : arr;
  uint32_t current_arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

  g_tx_state.frequency_hz = WirelessTx_ClampFrequency(frequency_hz);
  g_tx_state.compare = compare;

  if (arr < current_arr)
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, compare);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, trigger_compare);
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
  }
  else
  {
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, compare);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, trigger_compare);
  }

  if (enable_outputs)
  {
    __HAL_TIM_MOE_ENABLE(&htim1);
  }
  else
  {
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  }
}

static uint32_t WirelessTx_MapPotToFrequency(uint16_t pot_raw)
{
  uint32_t frequency_span = TX_FREQ_MAX_HZ - TX_FREQ_MIN_HZ;

  if (pot_raw >= TX_POT_MAX_RAW)
  {
    return TX_FREQ_MAX_HZ;
  }

  return TX_FREQ_MIN_HZ + (((uint32_t)pot_raw * frequency_span) / TX_POT_MAX_RAW);
}

static uint32_t WirelessTx_ClampFrequency(uint32_t frequency_hz)
{
  if (frequency_hz < TX_FREQ_MIN_HZ)
  {
    return TX_FREQ_MIN_HZ;
  }

  if (frequency_hz > TX_FREQ_MAX_HZ)
  {
    return TX_FREQ_MAX_HZ;
  }

  return frequency_hz;
}

static uint16_t WirelessTx_FrequencyToPeriod(uint32_t frequency_hz)
{
  uint32_t limited_frequency = WirelessTx_ClampFrequency(frequency_hz);
  uint32_t timer_clock_hz = (uint32_t)ADV_TIM_CLK_MHz * 1000000U;
  uint32_t arr = (timer_clock_hz + limited_frequency) / (2U * limited_frequency);

  if (arr > 0xFFFFU)
  {
    arr = 0xFFFFU;
  }

  if (arr < 2U)
  {
    arr = 2U;
  }

  return (uint16_t)arr;
}

static void WirelessTx_ProcessButton(void)
{
  GPIO_PinState raw_state = HAL_GPIO_ReadPin(TX_BUTTON_GPIO_Port, TX_BUTTON_Pin);
  uint32_t now = HAL_GetTick();

  if (raw_state != g_tx_state.button_last_raw)
  {
    g_tx_state.button_last_raw = raw_state;
    g_tx_state.button_last_change_tick = now;
  }

  if (((now - g_tx_state.button_last_change_tick) >= TX_BUTTON_DEBOUNCE_MS) &&
      (raw_state != g_tx_state.button_stable_state))
  {
    GPIO_PinState previous_state = g_tx_state.button_stable_state;
    g_tx_state.button_stable_state = raw_state;

    if (!WirelessTx_ButtonIsPressed(previous_state) && WirelessTx_ButtonIsPressed(raw_state))
    {
      g_tx_state.force_enable = !g_tx_state.force_enable;
      WirelessTx_Log("\r\nbutton tx=%u\r\n", g_tx_state.force_enable ? 1U : 0U);
      WirelessTx_PrintPrompt();
    }
  }
}

static bool WirelessTx_ButtonIsPressed(GPIO_PinState state)
{
  return state == TX_BUTTON_PRESSED_STATE;
}

static void WirelessTx_SampleFeedback(void)
{
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
  {
    Error_Handler();
  }
  g_tx_state.bus_raw = HAL_ADC_GetValue(&hadc1);

  if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
  {
    Error_Handler();
  }
  g_tx_state.current_u_raw = HAL_ADC_GetValue(&hadc1);

  if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
  {
    Error_Handler();
  }
  g_tx_state.pot_raw = HAL_ADC_GetValue(&hadc1);

  if (HAL_ADC_Stop(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_Start(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_PollForConversion(&hadc2, 10U) != HAL_OK)
  {
    Error_Handler();
  }
  g_tx_state.current_v_raw = HAL_ADC_GetValue(&hadc2);

  if (HAL_ADC_Stop(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  g_tx_state.current_u = (int32_t)g_tx_state.current_u_raw - g_tx_state.current_u_offset;
  g_tx_state.current_v = (int32_t)g_tx_state.current_v_raw - g_tx_state.current_v_offset;
}

static void WirelessTx_CalibrateCurrentOffsets(void)
{
  uint32_t i;
  int64_t current_u_acc = 0;
  int64_t current_v_acc = 0;

  for (i = 0U; i < TX_CURRENT_OFFSET_SAMPLES; i++)
  {
    WirelessTx_SampleFeedback();
    current_u_acc += g_tx_state.current_u_raw;
    current_v_acc += g_tx_state.current_v_raw;
    HAL_Delay(2U);
  }

  g_tx_state.current_u_offset = (int32_t)(current_u_acc / TX_CURRENT_OFFSET_SAMPLES);
  g_tx_state.current_v_offset = (int32_t)(current_v_acc / TX_CURRENT_OFFSET_SAMPLES);
}

static void WirelessTx_PrintDebug(void)
{
  char message[192];
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  int length = snprintf(message,
                        sizeof(message),
                        "status mode=%s tx=%u freq=%luHz arr=%lu duty=50%% cmp=%u pot=%u bus=%u iu=%4ld iv=%4ld dt=%5luns\r\n",
                        g_tx_state.manual_mode ? "manual" : "pot",
                        g_tx_state.tx_enabled ? 1U : 0U,
                        (unsigned long)g_tx_state.frequency_hz,
                        (unsigned long)arr,
                        g_tx_state.compare,
                        g_tx_state.pot_raw,
                        g_tx_state.bus_raw,
                        (long)g_tx_state.current_u,
                        (long)g_tx_state.current_v,
                        (unsigned long)g_tx_state.dead_time_ns);

  if (length > 0)
  {
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)message, (uint16_t)length, 50U);
  }
}

static void WirelessTx_ProcessCli(void)
{
  uint8_t ch;

  while (HAL_UART_Receive(&huart2, &ch, 1U, 0U) == HAL_OK)
  {
    if ((ch == '\r') || (ch == '\n'))
    {
      if (g_cli_length > 0U)
      {
        g_cli_buffer[g_cli_length] = '\0';
        WirelessTx_Log("\r\n");
        WirelessTx_HandleCommand(g_cli_buffer);
        g_cli_length = 0U;
      }
      else
      {
        WirelessTx_Log("\r\n");
      }

      WirelessTx_PrintPrompt();
      continue;
    }

    if ((ch == '\b') || (ch == 127U))
    {
      if (g_cli_length > 0U)
      {
        g_cli_length--;
        WirelessTx_Log("\b \b");
      }
      continue;
    }

    if ((ch >= 32U) && (ch < 127U) && (g_cli_length < (TX_CLI_BUFFER_LENGTH - 1U)))
    {
      g_cli_buffer[g_cli_length++] = (char)ch;
      (void)HAL_UART_Transmit(&huart2, &ch, 1U, 10U);
    }
  }
}

static void WirelessTx_HandleCommand(char *command)
{
  char *token = strtok(command, " ");

  if (token == NULL)
  {
    return;
  }

  if (strcmp(token, "help") == 0)
  {
    WirelessTx_Log("commands: help, status, mode pot, mode manual, enable 0|1, freq <100..300kHz>, dt [ns], stream on|off, cal, ocp <0..4095>\r\n");
    return;
  }

  if (strcmp(token, "status") == 0)
  {
    WirelessTx_PrintDebug();
    return;
  }

  if (strcmp(token, "mode") == 0)
  {
    char *value = strtok(NULL, " ");

    if ((value != NULL) && (strcmp(value, "pot") == 0))
    {
      g_tx_state.manual_mode = false;
      WirelessTx_Log("mode set to pot\r\n");
      return;
    }

    if ((value != NULL) && (strcmp(value, "manual") == 0))
    {
      g_tx_state.manual_mode = true;
      WirelessTx_Log("mode set to manual\r\n");
      return;
    }
  }

  if (strcmp(token, "enable") == 0)
  {
    char *value = strtok(NULL, " ");

    if (value != NULL)
    {
      g_tx_state.force_enable = (atoi(value) != 0);
      WirelessTx_Log("manual enable=%u\r\n", g_tx_state.force_enable ? 1U : 0U);
      return;
    }
  }

  if (strcmp(token, "freq") == 0)
  {
    char *value = strtok(NULL, " ");

    if (value != NULL)
    {
      unsigned long requested = strtoul(value, NULL, 10);

      if (requested <= 1000U)
      {
        requested *= 1000U;
      }

      g_tx_state.requested_frequency_hz = WirelessTx_ClampFrequency((uint32_t)requested);
      WirelessTx_Log("manual freq=%luHz\r\n", (unsigned long)g_tx_state.requested_frequency_hz);
      return;
    }
  }

  if (strcmp(token, "pwm") == 0)
  {
    WirelessTx_Log("duty is fixed at 50%%; use freq <100..300kHz>\r\n");
    return;
  }

  if (strcmp(token, "stream") == 0)
  {
    char *value = strtok(NULL, " ");

    if ((value != NULL) && ((strcmp(value, "on") == 0) || (strcmp(value, "1") == 0)))
    {
      g_tx_state.stream_enabled = true;
      WirelessTx_Log("stream on\r\n");
      return;
    }

    if ((value != NULL) && ((strcmp(value, "off") == 0) || (strcmp(value, "0") == 0)))
    {
      g_tx_state.stream_enabled = false;
      WirelessTx_Log("stream off\r\n");
      return;
    }
  }

  if (strcmp(token, "cal") == 0)
  {
    WirelessTx_Log("current offset calibration start\r\n");
    WirelessTx_CalibrateCurrentOffsets();
    WirelessTx_Log("current offset calibration done\r\n");
    return;
  }

  if (strcmp(token, "dt") == 0)
  {
    char *value = strtok(NULL, " ");

    if (value == NULL)
    {
      WirelessTx_Log("dt=%luns\r\n", (unsigned long)g_tx_state.dead_time_ns);
      return;
    }

    unsigned long requested = strtoul(value, NULL, 10);

    if (requested > TX_DT_MAX_NS)
    {
      requested = TX_DT_MAX_NS;
    }

    WirelessTx_ApplyDeadTime((uint32_t)requested);
    WirelessTx_Log("dt=%luns\r\n", (unsigned long)g_tx_state.dead_time_ns);
    return;
  }

  if (strcmp(token, "ocp") == 0)
  {
    char *value = strtok(NULL, " ");

    if (value != NULL)
    {
      unsigned long threshold = strtoul(value, NULL, 10);

      if (threshold > 4095U)
      {
        threshold = 4095U;
      }

      (void)HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, threshold);
      (void)HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, threshold);
      WirelessTx_Log("ocp dac threshold=%lu\r\n", threshold);
      return;
    }
  }

  WirelessTx_Log("unknown command: %s\r\n", token);
}

static void WirelessTx_Log(const char *format, ...)
{
  char buffer[192];
  va_list args;
  int length;

  va_start(args, format);
  length = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (length > 0)
  {
    if (length >= (int)sizeof(buffer))
    {
      length = (int)sizeof(buffer) - 1;
    }

    (void)HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)length, 100U);
  }
}

static void WirelessTx_PrintPrompt(void)
{
  WirelessTx_Log("tx> ");
}

static uint8_t WirelessTx_NsToDtg(uint32_t dt_ns)
{
  uint32_t clk = (uint32_t)ADV_TIM_CLK_MHz;
  uint32_t v;

  /* tDTS = 2 / ADV_TIM_CLK_MHz (DIV2 clock division)
   * Range 0: DTG x tDTS,            DTG 0..127
   * Range 1: (64+DTG[5:0]) x 2*tDTS, DTG[5:0] 0..63
   * Range 2: (32+DTG[4:0]) x 8*tDTS, DTG[4:0] 0..31
   * Range 3: (32+DTG[4:0]) x 16*tDTS, DTG[4:0] 0..31 */

  v = (dt_ns * clk) / 2000U;
  if (v <= 127U)
  {
    return (uint8_t)v;
  }

  v = (dt_ns * clk) / 4000U;
  if (v >= 64U && v <= 127U)
  {
    return (uint8_t)(0x80U | (v - 64U));
  }

  v = (dt_ns * clk) / 16000U;
  if (v >= 32U && v <= 63U)
  {
    return (uint8_t)(0xC0U | (v - 32U));
  }

  v = (dt_ns * clk) / 32000U;
  if (v < 32U) { v = 32U; }
  if (v > 63U) { v = 63U; }
  return (uint8_t)(0xE0U | (v - 32U));
}

static void WirelessTx_ApplyDeadTime(uint32_t dt_ns)
{
  uint8_t dtg = WirelessTx_NsToDtg(dt_ns);
  htim1.Instance->BDTR = (htim1.Instance->BDTR & ~0xFFU) | (uint32_t)dtg;
  g_tx_state.dead_time_ns = dt_ns;
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
  MX_USART2_UART_Init();
  WirelessTx_Log("\r\ninit: gpio, dma, uart ready\r\n");
  MX_ADC1_Init();
  WirelessTx_Log("init: adc1 ready\r\n");
  MX_ADC2_Init();
  WirelessTx_Log("init: adc2 ready\r\n");
  MX_COMP1_Init();
  WirelessTx_Log("init: comp1 ready\r\n");
  MX_COMP2_Init();
  WirelessTx_Log("init: comp2 ready\r\n");
  MX_COMP4_Init();
  WirelessTx_Log("init: comp4 ready\r\n");
  MX_CORDIC_Init();
  WirelessTx_Log("init: cordic ready\r\n");
  MX_DAC3_Init();
  WirelessTx_Log("init: dac3 ready\r\n");
  MX_OPAMP1_Init();
  WirelessTx_Log("init: opamp1 ready\r\n");
  MX_OPAMP2_Init();
  WirelessTx_Log("init: opamp2 ready\r\n");
  MX_OPAMP3_Init();
  WirelessTx_Log("init: opamp3 ready\r\n");
  MX_TIM1_Init();
  WirelessTx_Log("init: tim1 ready\r\n");

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  WirelessTx_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    WirelessTx_Loop();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp1.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH1;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief COMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp2.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH2;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */

}

/**
  * @brief COMP4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP4_Init(void)
{

  /* USER CODE BEGIN COMP4_Init 0 */

  /* USER CODE END COMP4_Init 0 */

  /* USER CODE BEGIN COMP4_Init 1 */

  /* USER CODE END COMP4_Init 1 */
  hcomp4.Instance = COMP4;
  hcomp4.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp4.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH2;
  hcomp4.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp4.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp4.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp4.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP4_Init 2 */

  /* USER CODE END COMP4_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief DAC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC3_Init(void)
{

  /* USER CODE BEGIN DAC3_Init 0 */

  /* USER CODE END DAC3_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC3_Init 1 */

  /* USER CODE END DAC3_Init 1 */

  /** DAC Initialization
  */
  hdac3.Instance = DAC3;
  if (HAL_DAC_Init(&hdac3) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC3_Init 2 */

  /* USER CODE END DAC3_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InternalOutput = DISABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_8_OR_MINUS_7;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp2.Init.Mode = OPAMP_PGA_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.InternalOutput = DISABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_8_OR_MINUS_7;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief OPAMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP3_Init(void)
{

  /* USER CODE BEGIN OPAMP3_Init 0 */

  /* USER CODE END OPAMP3_Init 0 */

  /* USER CODE BEGIN OPAMP3_Init 1 */

  /* USER CODE END OPAMP3_Init 1 */
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp3.Init.Mode = OPAMP_PGA_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp3.Init.InternalOutput = ENABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_8_OR_MINUS_7;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP3_Init 2 */

  /* USER CODE END OPAMP3_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = (REP_COUNTER);
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP1;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP2;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP4;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = ((PWM_PERIOD_CYCLES) / 4);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS) / 2);
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 3;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 3;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = TX_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TX_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
