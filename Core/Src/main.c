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
#include "adc.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOOP_CYCLE_MS 20

#define MODE_TIME_HM 0
#define MODE_TIME_MS 1
#define MODE_SET_HM 2
#define MODE_SHOW_LIGHT_ADC 3
#define MODE_ALARM 4
#define MODE_DISPLAY_TEMP 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char ch1, ch2, ch3, ch4;
int colon;
int brightness, current_light;
int key1, key2, key3, key4;

int mode;

int adc1_in1_res;

char uart_tx_buffer[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void delay_us(uint32_t us);
static void delay_ms(uint16_t ms);

static uint16_t LED_Control_Input(char ch);
static void LED_Display_Pos(uint16_t pos, uint16_t ch, uint32_t delay);
static void LED_Display(char ch1, char ch2, int colon, char ch3, char ch4, uint32_t delay);
static void LED_Display_Flush();
static void LED_Set_Int(int x);
static void LED_Set_CurrentTime_HM();
static void LED_Set_CurrentTime_MS();

static void TIM_Set_AlarmState(int is_on);

static void Tick_Read_Keys();
static void Tick_Set_HM();
static void Tick_Light_ADC();
static void Tick_Alarm();
static void Tick_DisplayTemp();

void UART_Init();
void UART_Write_Text(const char *text);
void UART_Write_NewLine();
void UART_Write_Int(int x);
void UART_Flush();
void UART_OnData();

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
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  UART_Init();
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_Delay(1000);
  UART_Write_Text("---LedClock---");
  UART_Flush();

  brightness = 0;
  mode = MODE_TIME_HM;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Tick_Simple_Counter();
    Tick_Read_Keys();
    Tick_Light_ADC();
    switch (mode)
    {
    case MODE_TIME_HM:
      LED_Set_CurrentTime_HM();
      if (key4)
      {
        mode = MODE_TIME_MS;
      }
      if (key1)
      {
        mode = MODE_SET_HM;
      }
      if (key2)
      {
        mode = MODE_DISPLAY_TEMP;
      }
      if (key3)
      {
        mode = MODE_ALARM;
      }
      break;
    case MODE_TIME_MS:
      LED_Set_CurrentTime_MS();
      if (key4)
      {
        mode = MODE_TIME_HM;
      }
      break;
    case MODE_SET_HM:
      Tick_Set_HM();
      break;
    case MODE_SHOW_LIGHT_ADC:
      LED_Set_Int(adc1_in1_res);
      if (key2)
      {
        mode = MODE_TIME_HM;
      }
      break;
    case MODE_ALARM:
      Tick_Alarm();
      break;
    case MODE_DISPLAY_TEMP:
      Tick_DisplayTemp();
      break;
    default:
      LED_Set_Int(0);
      mode = MODE_TIME_HM;
      break;
    }
    LED_Display_Flush();
    UART_Flush();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ----------------------------------  ---------------------------------------- */
/*                                 LED DISPLAY                                */
/* -------------------------------------------------------------------------- */

static uint16_t LED_Control_Input(char ch)
{
  switch (ch)
  {
  case '0':
    return LED_A_Pin | LED_B_Pin | LED_C_Pin | LED_D_Pin | LED_E_Pin | LED_F_Pin;
  case '1':
    return LED_B_Pin | LED_C_Pin;
  case '2':
    return LED_A_Pin | LED_B_Pin | LED_G_Pin | LED_E_Pin | LED_D_Pin;
  case '3':
    return LED_A_Pin | LED_B_Pin | LED_C_Pin | LED_D_Pin | LED_G_Pin;
  case '4':
    return LED_F_Pin | LED_G_Pin | LED_B_Pin | LED_C_Pin;
  case '5':
    return LED_A_Pin | LED_F_Pin | LED_G_Pin | LED_C_Pin | LED_D_Pin;
  case '6':
    return LED_A_Pin | LED_F_Pin | LED_G_Pin | LED_C_Pin | LED_E_Pin | LED_D_Pin;
  case '7':
    return LED_A_Pin | LED_B_Pin | LED_C_Pin;
  case '8':
    return LED_A_Pin | LED_B_Pin | LED_C_Pin | LED_D_Pin | LED_E_Pin | LED_F_Pin | LED_G_Pin;
  case '9':
    return LED_A_Pin | LED_B_Pin | LED_C_Pin | LED_D_Pin | LED_F_Pin | LED_G_Pin;
  case '_':
    return LED_D_Pin;
  default:
    return 0;
  }
}

static void LED_Display_Pos(uint16_t channel, uint16_t ch_pin, uint32_t delay)
{
  __HAL_TIM_SET_COMPARE(&htim1, channel, brightness);

  HAL_GPIO_WritePin(GPIOB, ch_pin, GPIO_PIN_SET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(GPIOB, ch_pin, GPIO_PIN_RESET);

  __HAL_TIM_SET_COMPARE(&htim1, channel, 255);
}

static void LED_Display(char ch1, char ch2, int colon, char ch3, char ch4, uint32_t delay)
{
  LED_Display_Pos(TIM_CHANNEL_1, LED_Control_Input(ch1), delay);
  LED_Display_Pos(TIM_CHANNEL_2, LED_Control_Input(ch2) | ((colon) ? LED_DP_Pin : 0), delay);
  LED_Display_Pos(TIM_CHANNEL_3, LED_Control_Input(ch3), delay);
  LED_Display_Pos(TIM_CHANNEL_4, LED_Control_Input(ch4), delay);
}

static void LED_Display_Flush()
{
  for (int i = 0; i < 5; i++)
  {
    LED_Display(ch1, ch2, colon, ch3, ch4, 1);
  }
}

static void LED_Set_Int(int x)
{
  ch4 = x % 10 + '0';
  x /= 10;
  ch3 = x % 10 + '0';
  x /= 10;
  ch2 = x % 10 + '0';
  x /= 10;
  ch1 = x % 10 + '0';
  colon = 0;
}

/* -------------------------------------------------------------------------- */
/*                                    INPUT                                   */
/* -------------------------------------------------------------------------- */

#define PROCESS_KEY(i)                        \
  if (HAL_GPIO_ReadPin(GPIOA, BTN_##i##_Pin)) \
  {                                           \
    k##i##_hits = 0;                          \
    key##i = 0;                               \
  }                                           \
  else                                        \
  {                                           \
    k##i##_hits++;                            \
    if (k##i##_hits == 2)                     \
    {                                         \
      UART_Write_Text("Key " #i " down.\n");  \
      key##i = 1;                             \
    }                                         \
    else                                      \
    {                                         \
      key##i = 0;                             \
    }                                         \
  }
static void Tick_Read_Keys()
{
  static int k1_hits = 0, k2_hits = 0, k3_hits = 0, k4_hits = 0;
  PROCESS_KEY(1);
  PROCESS_KEY(2);
  PROCESS_KEY(3);
  PROCESS_KEY(4);
}

/* -------------------------------------------------------------------------- */
/*                                    USART                                   */
/* -------------------------------------------------------------------------- */

struct
{
  char *tx_cur;
  char tx[256];
  char *rx_cur;
  char rx[32];
} uart_buf;

void UART_Init()
{
  uart_buf.tx_cur = uart_buf.tx;
  uart_buf.rx_cur = uart_buf.rx;
  HAL_UART_Receive_IT(&huart1, uart_buf.rx_cur, 1);
}

void UART_Write_Text(const char *text)
{
  while (*text != '\0')
  {
    *uart_buf.tx_cur = *text;
    text++;
    uart_buf.tx_cur++;
  }
}
void UART_Write_NewLine()
{
  UART_Write_Text("\n");
}
void UART_Write_Int(int x)
{
  if (x < 0)
  {
    *uart_buf.tx_cur = '-';
    uart_buf.tx_cur++;
    UART_Write_Int(-x);
    return;
  }
  if (x >= 10)
  {
    UART_Write_Int(x / 10);
  }
  *uart_buf.tx_cur = x % 10 + '0';
  uart_buf.tx_cur++;
}
void UART_Flush()
{
  if (uart_buf.tx_cur > uart_buf.tx)
  {
    HAL_UART_Transmit_IT(&huart1, uart_buf.tx, uart_buf.tx_cur - uart_buf.tx);
    uart_buf.tx_cur = uart_buf.tx;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (*uart_buf.rx_cur == '#')
  {
    UART_OnData();
    uart_buf.rx_cur = uart_buf.rx;
  }
  else
  {
    uart_buf.rx_cur++;
  }
  HAL_UART_Receive_IT(&huart1, uart_buf.rx_cur, 1);
}

void UART_OnData()
{
  int x = 0;
  char *cur = uart_buf.rx;
  while (*cur < '0' || *cur > '9')
    cur++;
  while (*cur >= '0' && *cur <= '9')
  {
    x *= 10;
    x += *cur - '0';
    cur++;
  }
  // counter = x;
  UART_Write_Text("Recieve integer ");
  UART_Write_Int(x);
  UART_Write_NewLine();
}

/* -------------------------------------------------------------------------- */
/*                            TIME DISPLAY AND SET                            */
/* -------------------------------------------------------------------------- */

static void LED_Set_CurrentTime_HM()
{
  RTC_TimeTypeDef sTime1;
  HAL_RTC_GetTime(&hrtc, &sTime1, RTC_FORMAT_BCD);

  ch1 = sTime1.Hours / 16 + '0';
  ch2 = sTime1.Hours % 16 + '0';
  ch3 = sTime1.Minutes / 16 + '0';
  ch4 = sTime1.Minutes % 16 + '0';

  colon = sTime1.Seconds & 1;
}

static void LED_Set_CurrentTime_MS()
{
  RTC_TimeTypeDef sTime1;
  HAL_RTC_GetTime(&hrtc, &sTime1, RTC_FORMAT_BCD);

  ch1 = sTime1.Minutes / 16 + '0';
  ch2 = sTime1.Minutes % 16 + '0';
  ch3 = sTime1.Seconds / 16 + '0';
  ch4 = sTime1.Seconds % 16 + '0';

  colon = sTime1.Seconds & 1;
}

static void Tick_Set_HM()
{
  static int timeout = 0;
  static int currentPos = 0;
  static int x1, x2, x3, x4;
  if (!key1 && !key2 && !key3 && !key4)
  {
    timeout++;
    if (timeout > 1000)
    {
      currentPos = 0;
      timeout = 0;
      mode = MODE_TIME_HM;
      return;
    }
  }
  else
  {
    timeout = 0;
  }

  RTC_TimeTypeDef sTime1;
  HAL_RTC_GetTime(&hrtc, &sTime1, RTC_FORMAT_BCD);

  if (currentPos == 0)
  {
    x1 = sTime1.Hours / 16;
    x2 = sTime1.Hours % 16;
    x3 = sTime1.Minutes / 16;
    x4 = sTime1.Minutes % 16;
    currentPos++;
  }

  if (key1)
  {
    currentPos++;
  }

  if (currentPos > 4)
  {
    sTime1.Hours = x1 * 16 + x2;
    sTime1.Minutes = x3 * 16 + x4;
    sTime1.Seconds = 0x00;
    HAL_RTC_SetTime(&hrtc, &sTime1, RTC_FORMAT_BCD);
    currentPos = 0;
    timeout = 0;
    mode = MODE_TIME_HM;
    return;
  }

  if (currentPos == 2 && x1 == 2 && x2 > 3)
  {
    x2 = 3;
  }

  if (key2)
  {
    switch (currentPos)
    {
    case 1:
      if (x1 < 2)
        x1++;
      else if (x2 > 3)
        x2 = 3;
      break;
    case 2:
      if (x1 == 2)
      {
        if (x2 < 3)
          x2++;
      }
      else
      {
        if (x2 < 9)
          x2++;
      }
      break;
    case 3:
      if (x3 < 5)
        x3++;
      break;
    case 4:
      if (x4 < 9)
        x4++;
      break;
    default:
      break;
    }
  }

  if (key3)
  {
    switch (currentPos)
    {
    case 1:
      if (x1 > 0)
        x1--;
      break;
    case 2:
      if (x2 > 0)
        x2--;
      break;
    case 3:
      if (x3 > 0)
        x3--;
      break;
    case 4:
      if (x4 > 0)
        x4--;
      break;
    default:
      break;
    }
  }

  if (sTime1.Seconds & 1)
  {
    ch1 = ch2 = ch3 = ch4 = 0;
    colon = 1;
    switch (currentPos)
    {
    case 1:
      ch1 = x1 + '0';
      break;
    case 2:
      ch2 = x2 + '0';
      break;
    case 3:
      ch3 = x3 + '0';
      break;
    case 4:
      ch4 = x4 + '0';
      break;
    }
  }
  else
  {
    ch1 = x1 + '0';
    ch2 = x2 + '0';
    ch3 = x3 + '0';
    ch4 = x4 + '0';
    colon = 0;
  }
}

/* -------------------------------------------------------------------------- */
/*                             BRIGHTNESS CONTROL                             */
/* -------------------------------------------------------------------------- */

static void Tick_Light_ADC()
{
  int expected = (current_light / 16);
  if (expected > 240)
  {
    expected = 240;
  }
  if (brightness < expected)
  {
    brightness++;
  }
  else if (brightness > expected)
  {
    brightness--;
  }
  static int hits = 0;
  hits++;
  if (hits >= 50)
  {
    hits = 0;
    HAL_ADC_Start_IT(&hadc1);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  // Read & Update The ADC Result
  current_light = HAL_ADC_GetValue(&hadc1);
}

/* -------------------------------------------------------------------------- */
/*                                    ALARM                                   */
/* -------------------------------------------------------------------------- */

static void TIM_Set_AlarmState(int is_on)
{
  static int ring_flag = 0;
  if (is_on && !ring_flag)
  {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    ring_flag = 1;
    return;
  }
  if (!is_on && ring_flag)
  {
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    ring_flag = 0;
    return;
  }
}

static void Tick_Alarm()
{
  RTC_TimeTypeDef sTime1;
  HAL_RTC_GetTime(&hrtc, &sTime1, RTC_FORMAT_BCD);

  if (sTime1.Seconds & 1)
  {
    TIM_Set_AlarmState(1);
    ch1 = ch2 = ch3 = ch4 = '8';
    colon = 1;
    brightness = 0;
  }
  else
  {
    TIM_Set_AlarmState(0);
    LED_Set_CurrentTime_HM();
  }

  if (key1 || key2 || key3 || key3)
  {
    TIM_Set_AlarmState(0);
    mode = MODE_TIME_HM;
    return;
  }
}

/* -------------------------------------------------------------------------- */
/*                                    DHT11                                   */
/* -------------------------------------------------------------------------- */

static void delay_us(uint32_t us)
{

  __IO uint32_t currentTicks = SysTick->VAL;
  /* Number of ticks per millisecond */
  const uint32_t tickPerMs = SysTick->LOAD + 1;
  /* Number of ticks to count */
  const uint32_t nbTicks = ((us - ((us > 0) ? 1 : 0)) * tickPerMs) / 1000;
  /* Number of elapsed ticks */
  uint32_t elapsedTicks = 0;
  __IO uint32_t oldTicks = currentTicks;
  do
  {

    currentTicks = SysTick->VAL;
    elapsedTicks += (oldTicks < currentTicks) ? tickPerMs + oldTicks - currentTicks : oldTicks - currentTicks;
    oldTicks = currentTicks;
  } while (nbTicks > elapsedTicks);
}
static void delay_ms(uint16_t ms)
{
  HAL_Delay(ms);
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_Start(void)
{
  // Set_Pin_Output(DHT11_GPIO_Port, DHT11_Pin);       // set the pin as output
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, 0); // pull the pin low
  delay_us(18000);                                  // wait for 18ms
  // Set_Pin_Input(DHT11_GPIO_Port, DHT11_Pin);        // set as input
}

uint8_t DHT11_Check_Response(void)
{
  uint8_t Response = 0;
  delay_us(40);
  if (!(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)))
  {
    delay_us(80);
    if ((HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)))
      Response = 1;
    else
      Response = -1;
  }
  while ((HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)))
    ; // wait for the pin to go low

  return Response;
}

uint8_t DHT11_Read(void)
{
  uint8_t i = 0, j = 0;
  for (j = 0; j < 8; j++)
  {
    while (!(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)))
      ;                                                  // wait for the pin to go high
    delay_us(40);                                        // wait for 40 us
    if (!(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))) // if the pin is low
    {
      i &= ~(1 << (7 - j)); // write 0
    }
    else
      i |= (1 << (7 - j)); // if the pin is high, write 1
    while ((HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)))
      ; // wait for the pin to go low
  }
  return i;
}

static void Tick_DisplayTemp()
{
  static uint8_t temp = 0, humi = 0;
  static int stage = 0;
  if (stage == 0)
  {
    DHT11_Start();
    int Presence = DHT11_Check_Response();
    if (Presence == 1)
    {
      uint8_t Rh_byte1 = DHT11_Read();
      uint8_t Rh_byte2 = DHT11_Read();
      uint8_t Temp_byte1 = DHT11_Read();
      uint8_t Temp_byte2 = DHT11_Read();
      uint8_t SUM = DHT11_Read();

      temp = Temp_byte1;
      humi = Rh_byte1;
    }
    stage = 1;
  }
  if (stage == 1)
  {
    LED_Set_Int(temp);
  }
  if (stage == 2)
  {
    LED_Set_Int(humi);
  }
  if (key2)
  {
    stage++;
    if (stage > 2)
    {
      stage = 0;
      mode = MODE_TIME_HM;
      return;
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
    HAL_Delay(500);
    HAL_GPIO_TogglePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin);
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
