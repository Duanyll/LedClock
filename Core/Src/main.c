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
int brightness;
int key1, key2, key3, key4;

int mode;

int adc1_in1_res;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void delay_us(uint16_t us);
static void delay_ms(uint16_t ms);

static uint16_t LED_Control_Input(char ch);
static void LED_Display_Pos(uint16_t pos, uint16_t ch, uint32_t delay);
static void LED_Display(char ch1, char ch2, int colon, char ch3, char ch4, uint32_t delay);
static void LED_Display_Flush();
static void LED_Set_Int(int x);
static void LED_AllOn(uint32_t delay);
static void LED_Set_CurrentTime_HM(      );
static void LED_Set_CurrentTime_MS();

static void TIM_Set_AlarmState(int is_on);

static void Tick_Simple_Counter();
static void Tick_Read_Keys();
static void Tick_Set_HM();
static void Tick_Light_ADC();
static void Tick_Alarm();
static void Tick_DisplayTemp();

uint8_t DHT11Init(void);
uint8_t DHT11ReadData(uint8_t *Humi, uint8_t *Temp);

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  LED_AllOn(3000);
  brightness = MAX_BRIGHTNESS;
  mode = MODE_TIME_HM;

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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

static void Tick_Light_ADC()
{
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
  adc1_in1_res = HAL_ADC_GetValue(&hadc1);
  if (adc1_in1_res <= 2200)
  {
    brightness = 5;
  }
  else if (adc1_in1_res <= 2500)
  {
    brightness = 4;
  }
  else if (adc1_in1_res <= 2800)
  {
    brightness = 3;
  }
  else if (adc1_in1_res <= 3200)
  {
    brightness = 2;
  }
  else
  {
    brightness = 1;
  }
}

static void TIM_Set_AlarmState(int is_on)
{
  static int ring_flag = 0;
  if (is_on && !ring_flag)
  {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    ring_flag = 1;
    return;
  }
  if (!is_on && ring_flag)
  {
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
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
    brightness = MAX_BRIGHTNESS;
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

static void delay_us(uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0); // set the counter value a 0
  while (__HAL_TIM_GET_COUNTER(&htim2) < us)
    ;
}
static void delay_ms(uint16_t ms)
{
  HAL_Delay(ms);
}

uint8_t DHT11RstAndCheck(void)
{
  uint8_t timer = 0;

  __set_PRIMASK(1);                                              //关�?�中�?
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET); //输出低电�?
  delay_ms(20);                                                  //拉低至少18ms
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);   //输出高电�?
  delay_us(30);                                                  //拉高20~40us
  while (!HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))          //等待总线拉低，DHT11会拉�?40~80us作为响应信号
  {
    timer++; //总线拉低时计�?
    delay_us(1);
  }
  if (timer > 100 || timer < 20) //判断响应时间
  {
    __set_PRIMASK(0); //�?总中�?
    return 0;
  }
  timer = 0;
  while (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)) //等待DHT11释放总线，持续时�?40~80us
  {
    timer++; //总线拉高时计�?
    delay_us(1);
  }
  __set_PRIMASK(0);              //�?总中�?
  if (timer > 100 || timer < 20) //�?测响应信号之后的高电�?
  {
    return 0;
  }
  return 1;
}

/*读取�?字节数据，返回�??-读到的数�?*/
uint8_t DHT11ReadByte(void)
{
  uint8_t i;
  uint8_t byt = 0;

  __set_PRIMASK(1); //关�?�中�?
  for (i = 0; i < 8; i++)
  {
    while (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))
      ; //等待低电平，数据位前都有50us低电平时�?
    while (!HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))
      ; //等待高电平，�?始传输数据位
    delay_us(40);
    byt <<= 1;                                        //因高位在前，�?以左移byt，最低位�?0
    if (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)) //将�?�线电平值读取到byt�?低位�?
    {
      byt |= 0x01;
    }
  }
  __set_PRIMASK(0); //�?总中�?

  return byt;
}

/*读取�?次数据，返回值：Humi-湿度整数部分数据,Temp-温度整数部分数据；返回�??: -1-失败�?1-成功*/
uint8_t DHT11ReadData(uint8_t *Humi, uint8_t *Temp)
{
  int sta = 0;
  uint8_t i;
  uint8_t buf[5];

  if (DHT11RstAndCheck()) //�?测响应信�?
  {
    for (i = 0; i < 5; i++) //读取40位数�?
    {
      buf[i] = DHT11ReadByte(); //读取1字节数据
    }
    if (buf[0] + buf[1] + buf[2] + buf[3] == buf[4]) //校验成功
    {
      *Humi = buf[0]; //保存湿度整数部分数据
      *Temp = buf[2]; //保存温度整数部分数据
    }
    sta = 1;
  }
  else //响应失败返回-1
  {
    *Humi = 0xFF; //响应失败返回255
    *Temp = 0xFF; //响应失败返回255
    sta = -1;
  }

  return sta;
}

static void Tick_DisplayTemp() {
  static int temp = 0, humi = 0;
  static stage = 0;
  if (stage == 0) {
    DHT11ReadData(&humi, &temp);
    stage = 1;
  }
  if (stage == 1) {
    LED_Set_Int(temp);
  }
  if (stage == 2) {
    LED_Set_Int(humi);
  }
  if (key2) {
    stage++;
    if (stage > 2) {
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
