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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOOP_CYCLE_MS 20
#define MAX_BRIGHTNESS 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char ch1, ch2, ch3, ch4;
int colon;
int brightness;
int key1, key2, key3, key4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint16_t LED_Control_Input(char ch);
static void LED_Display_Pos(uint16_t pos, uint16_t ch, uint32_t delay);
static void LED_Display(char ch1, char ch2, int colon, char ch3, char ch4, uint32_t delay);
static void LED_Display_Flush();
static void LED_Set_Int(int x);
static void LED_AllOn(uint32_t delay);

static void Tick_SimpleCounter();
static void Tick_Read_Keys();
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
  /* USER CODE BEGIN 2 */
  LED_AllOn(5000);
  brightness = 5;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int count = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Tick_Simple_Counter();
    Tick_Read_Keys();
    if (key1)
      count += 1;
    if (key2)
      count += 2;
    if (key3)
      count += 3;
    if (key4) {
      brightness++;
      if (brightness > MAX_BRIGHTNESS) {
        brightness = 1;
      }
    }
    LED_Set_Int(count);
    LED_Display_Flush();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
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

static void LED_Display_Pos(uint16_t pos_pin, uint16_t ch_pin, uint32_t delay)
{
  HAL_GPIO_WritePin(GPIOB, pos_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, ch_pin, GPIO_PIN_SET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(GPIOB, pos_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, ch_pin, GPIO_PIN_RESET);
}

static void LED_Display(char ch1, char ch2, int colon, char ch3, char ch4, uint32_t delay)
{
  const uint16_t FULL_POS = LED_1_Pin | LED_2_Pin | LED_3_Pin | LED_4_Pin;
  const uint16_t FULL_CHAR = LED_A_Pin | LED_B_Pin | LED_C_Pin | LED_D_Pin | LED_E_Pin | LED_F_Pin | LED_G_Pin | LED_DP_Pin;
  // HAL_GPIO_WritePin(GPIOB, FULL_POS, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(GPIOB, FULL_CHAR, GPIO_PIN_RESET);

  LED_Display_Pos(LED_1_Pin, LED_Control_Input(ch1), delay);
  LED_Display_Pos(LED_2_Pin, LED_Control_Input(ch2) | ((colon) ? LED_DP_Pin : 0), delay);
  LED_Display_Pos(LED_3_Pin, LED_Control_Input(ch3), delay);
  LED_Display_Pos(LED_4_Pin, LED_Control_Input(ch4), delay);
}

static void LED_Display_Flush()
{
  for (int i = 0; i < brightness; i++)
  {
    LED_Display(ch1, ch2, colon, ch3, ch4, 1);
  }
  int remain = (MAX_BRIGHTNESS - brightness) * 4;
  if (remain > 0)
  {
    HAL_Delay(remain);
  }
}

static void LED_AllOn(uint32_t delay)
{
  const uint16_t FULL_POS = LED_1_Pin | LED_2_Pin | LED_3_Pin | LED_4_Pin;
  const uint16_t FULL_CHAR = LED_A_Pin | LED_B_Pin | LED_C_Pin | LED_D_Pin | LED_E_Pin | LED_F_Pin | LED_G_Pin | LED_DP_Pin;
  HAL_GPIO_WritePin(GPIOB, FULL_POS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, FULL_CHAR, GPIO_PIN_SET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(GPIOB, FULL_POS, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, FULL_CHAR, GPIO_PIN_RESET);
}

static void Tick_Simple_Counter()
{
  static int counter = 0;
  static int hits = 0;
  hits++;
  if (hits > 50)
  {
    hits = 0;
    counter++;
    if (counter >= 10000)
    {
      counter = 0;
    }
  }
  LED_Set_Int(counter);
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

#define PROCESS_KEY(i)                         \
  if (HAL_GPIO_ReadPin(GPIOA, BTN_##i##_Pin)) \
  {                                            \
    k##i##_hits = 0;                           \
    key##i = 0;                                \
  }                                            \
  else                                         \
  {                                            \
    k##i##_hits++;                             \
    if (k##i##_hits == 2)                      \
    {                                          \
      key##i = 1;                              \
    }                                          \
    else                                       \
    {                                          \
      key##i = 0;                              \
    }                                          \
  }
static void Tick_Read_Keys()
{
  static int k1_hits = 0, k2_hits = 0, k3_hits = 0, k4_hits = 0;
  PROCESS_KEY(1);
  PROCESS_KEY(2);
  PROCESS_KEY(3);
  PROCESS_KEY(4);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
