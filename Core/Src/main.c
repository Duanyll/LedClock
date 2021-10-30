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
#include "gpio.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef unsigned char BOOL;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRUE 1
#define FALSE 0

#define LED_DIG1_CH TIM_CHANNEL_3
#define LED_DIG2_CH TIM_CHANNEL_2
#define LED_DIG3_CH TIM_CHANNEL_1
#define LED_DIG4_CH TIM_CHANNEL_4

#define KEY_ID_L 0
#define KEY_ID_R 1
#define KEY_ID_U 2
#define KEY_ID_D 3

#define KEY_L key_state[KEY_ID_L]
#define KEY_R key_state[KEY_ID_R]
#define KEY_U key_state[KEY_ID_U]
#define KEY_D key_state[KEY_ID_D]

#define KEY_MODE_ONCE 0
#define KEY_MODE_LONG 1
#define KEY_MODE_LONG_ACC 2

#define MUSIC_NYAN_CAT 1
#define MUSIC_ONLY_MY_RAILGUN 2
#define MUSIC_ZENBON_ZAKURA 3

#define LOOP_CYCLE_MS 20

#define PAGE_HOME 0
#define PAGE_MS 1
#define PAGE_ALARM 2
#define PAGE_MENU 3
#define PAGE_SET_TIME 4
#define PAGE_TEMP_HUMI 5
#define PAGE_SET_ALARM 6
#define PAGE_MUSIC 7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t display_buffer[4];
int brightness, current_light;
BOOL key_state[4];

int page;

uint8_t temp = 20, humi = 68;

BOOL enable_alarm;
RTC_TimeTypeDef alarm_time;
int alarm_music;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void delay_us(uint32_t us);

static uint16_t LED_GetFont(char ch);
static void LED_Flush();

static void Display_SetChar(const char *chs, BOOL colon);
static void Display_SetInt(int x);
static void Display_Set_CurrentTime_HM();
static void Display_Set_CurrentTime_MS();

static void Alarm_SetState(BOOL is_on, uint16_t period);
static void Alarm_Load();
static void Alarm_Save();
static void Alarm_InitMusic(int id);
static void Alarm_TickMusic();
static void Alarm_StopMusic();

void DHT11_Start(void);
static void DHT11_Run();

static void Keys_Tick();
static void Keys_SetMode(int key, uint8_t mode);

static void Light_Tick();

void UART_Init();
void UART_Write_Text(const char *text);
void UART_Write_NewLine();
void UART_Write_Int(int x);
void UART_Flush();
void UART_OnData();

static void App_Tick();
static void App_HomePage();
static void App_MSPage();
static void App_AlarmPage();
static void App_MenuPage();
static void App_SetTimePage();
static void App_TempHumiPage();
static void App_SetAlarmPage();
static void App_MusicPage();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU
     * Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick. */
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
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    UART_Init();
    HAL_ADCEx_Calibration_Start(&hadc1);
    DHT11_Start();
    HAL_Delay(1000);
    Alarm_Load();
    UART_Write_Text("---LedClock---");
    UART_Flush();

    brightness = 0;
    page = PAGE_HOME;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        Keys_Tick();
        Light_Tick();
        App_Tick();
        LED_Flush();
        UART_Flush();
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* -------------------------------------------------------------------------- */
/*                               LED AND DISPLAY                              */
/* -------------------------------------------------------------------------- */

static uint16_t LED_GetFont(char ch) {
    switch (ch) {
        case '0':
            return LED_A_Pin | LED_B_Pin | LED_C_Pin | LED_D_Pin | LED_E_Pin |
                   LED_F_Pin;
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
            return LED_A_Pin | LED_F_Pin | LED_G_Pin | LED_C_Pin | LED_E_Pin |
                   LED_D_Pin;
        case '7':
            return LED_A_Pin | LED_B_Pin | LED_C_Pin;
        case '8':
            return LED_A_Pin | LED_B_Pin | LED_C_Pin | LED_D_Pin | LED_E_Pin |
                   LED_F_Pin | LED_G_Pin;
        case '9':
            return LED_A_Pin | LED_B_Pin | LED_C_Pin | LED_D_Pin | LED_F_Pin |
                   LED_G_Pin;
        case '_':
            return LED_D_Pin;
        default:
            return 0;
    }
}

static void LED_ShowPos(uint16_t channel, uint16_t ch_pin, uint32_t delay) {
    __HAL_TIM_SET_COMPARE(&htim1, channel, brightness);

    HAL_GPIO_WritePin(GPIOB, ch_pin, GPIO_PIN_SET);
    HAL_Delay(delay);
    HAL_GPIO_WritePin(GPIOB, ch_pin, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim1, channel, 255);
}

static void LED_Flush() {
    for (int i = 0; i < 5; i++) {
        LED_ShowPos(LED_DIG1_CH, display_buffer[0], 1);
        LED_ShowPos(LED_DIG2_CH, display_buffer[1], 1);
        LED_ShowPos(LED_DIG3_CH, display_buffer[2], 1);
        LED_ShowPos(LED_DIG4_CH, display_buffer[3], 1);
    }
}

static void Display_SetChar(const char *chs, BOOL colon) {
    for (int i = 0; i < 4; i++) {
        display_buffer[i] = LED_GetFont(chs[i]);
    }
    if (colon) {
        display_buffer[1] |= LED_DP_Pin;
    }
}

static void Display_SetInt(int x) {
    display_buffer[3] = LED_GetFont(x % 10 + '0');
    x /= 10;
    display_buffer[2] = LED_GetFont(x % 10 + '0');
    x /= 10;
    display_buffer[1] = LED_GetFont(x % 10 + '0');
    x /= 10;
    display_buffer[0] = LED_GetFont(x % 10 + '0');
}

static void Display_Set_CurrentTime_HM() {
    RTC_TimeTypeDef sTime1;
    HAL_RTC_GetTime(&hrtc, &sTime1, RTC_FORMAT_BCD);

    char buf[4] = {sTime1.Hours / 16 + '0', sTime1.Hours % 16 + '0',
                   sTime1.Minutes / 16 + '0', sTime1.Minutes % 16 + '0'};

    Display_SetChar(buf, sTime1.Seconds & 1);
}

static void Display_Set_CurrentTime_MS() {
    RTC_TimeTypeDef sTime1;
    HAL_RTC_GetTime(&hrtc, &sTime1, RTC_FORMAT_BCD);

    char buf[4] = {sTime1.Minutes / 16 + '0', sTime1.Minutes % 16 + '0',
                   sTime1.Seconds / 16 + '0', sTime1.Seconds % 16 + '0'};

    Display_SetChar(buf, TRUE);
}

/* -------------------------------------------------------------------------- */
/*                                  KEY INPUT                                 */
/* -------------------------------------------------------------------------- */

uint8_t key_config[4];
static void Keys_Tick() {
    static int hits[4] = {0};
    static uint16_t keys[] = {BTN_L_Pin, BTN_R_Pin, BTN_U_Pin, BTN_D_Pin};
    for (int i = 0; i < 4; i++) {
        GPIO_PinState state = HAL_GPIO_ReadPin(GPIOB, keys[i]);
        if (state) {
            hits[i] = 0;
            key_state[i] = FALSE;
        } else {
            hits[i]++;
            switch (key_config[i]) {
                case KEY_MODE_ONCE:
                    if (hits[i] == 2) {
                        key_state[i] = TRUE;
                    } else {
                        key_state[i] = FALSE;
                    }
                    break;
                case KEY_MODE_LONG:
                    if (hits[i] == 2 || (hits[i] > 0 && hits[i] % 10 == 0)) {
                        key_state[i] = TRUE;
                    } else {
                        key_state[i] = FALSE;
                    }
                    break;
                case KEY_MODE_LONG_ACC:
                    if (hits[i] == 2 ||
                        (hits[i] >= 10 && hits[i] <= 50 && hits[i] % 10 == 0) ||
                        (hits[i] > 50 && hits[i] <= 100 && hits[i] % 4 == 0) ||
                        hits[i] > 100) {
                        key_state[i] = TRUE;
                    } else {
                        key_state[i] = FALSE;
                    }
                    break;
            }
        }
    }
}

static void Keys_SetMode(int key, uint8_t mode) { key_config[key] = mode; }

/* -------------------------------------------------------------------------- */
/*                                    USART                                   */
/* -------------------------------------------------------------------------- */

struct {
    char *tx_cur;
    char tx[256];
    char *rx_cur;
    char rx[32];
} uart_buf;

void UART_Init() {
    uart_buf.tx_cur = uart_buf.tx;
    uart_buf.rx_cur = uart_buf.rx;
    HAL_UART_Receive_IT(&huart2, uart_buf.rx_cur, 1);
}

void UART_Write_Text(const char *text) {
    while (*text != '\0') {
        *uart_buf.tx_cur = *text;
        text++;
        uart_buf.tx_cur++;
    }
}
void UART_Write_NewLine() { UART_Write_Text("\n"); }
void UART_Write_Int(int x) {
    if (x < 0) {
        *uart_buf.tx_cur = '-';
        uart_buf.tx_cur++;
        UART_Write_Int(-x);
        return;
    }
    if (x >= 10) {
        UART_Write_Int(x / 10);
    }
    *uart_buf.tx_cur = x % 10 + '0';
    uart_buf.tx_cur++;
}
void UART_Flush() {
    if (uart_buf.tx_cur > uart_buf.tx) {
        HAL_UART_Transmit_IT(&huart2, uart_buf.tx,
                             uart_buf.tx_cur - uart_buf.tx);
        uart_buf.tx_cur = uart_buf.tx;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (*uart_buf.rx_cur == '#') {
        UART_OnData();
        uart_buf.rx_cur = uart_buf.rx;
    } else {
        uart_buf.rx_cur++;
    }
    HAL_UART_Receive_IT(&huart2, uart_buf.rx_cur, 1);
}

void UART_OnData() {
    int x = 0;
    char *cur = uart_buf.rx;
    char command = *cur;
    cur++;
    while (*cur < '0' || *cur > '9') cur++;
    while (*cur >= '0' && *cur <= '9') {
        x *= 10;
        x += *cur - '0';
        cur++;
    }
    RTC_TimeTypeDef sTime1;
    int digits[6];
    switch (command) {
        case 'T':
            for (int i = 0; i < 6; i++) {
                digits[i] = x % 10;
                x /= 10;
            }
            if (digits[5] * 10 + digits[4] < 24 &&
                digits[3] * 10 + digits[2] < 60 &&
                digits[1] * 10 + digits[0] < 60) {
                sTime1.Seconds = (digits[1] << 4) + digits[0];
                sTime1.Minutes = (digits[3] << 4) + digits[2];
                sTime1.Hours = (digits[5] << 4) + digits[4];
                HAL_RTC_SetTime(&hrtc, &sTime1, RTC_FORMAT_BCD);
                UART_Write_Text("Time set.");
            } else {
                UART_Write_Text("Illegal time.");
            }
            break;
        case 'I':
            HAL_RTC_GetTime(&hrtc, &sTime1, RTC_FORMAT_BCD);
            UART_Write_Text("Current time: ");
            UART_Write_Int(sTime1.Hours >> 4);
            UART_Write_Int(sTime1.Hours % 16);
            UART_Write_Text(":");
            UART_Write_Int(sTime1.Minutes >> 4);
            UART_Write_Int(sTime1.Minutes % 16);
            UART_Write_Text(":");
            UART_Write_Int(sTime1.Seconds >> 4);
            UART_Write_Int(sTime1.Seconds % 16);
            break;
        case 'H':
            DHT11_Run();
            break;
        default:
            UART_Write_Text("Unknown command.");
            break;
    }
    UART_Write_NewLine();
}

/* -------------------------------------------------------------------------- */
/*                             BRIGHTNESS CONTROL                             */
/* -------------------------------------------------------------------------- */

static void Light_Tick() {
    int expected = (current_light / 16);
    if (expected > 240) {
        expected = 240;
    }
    if (brightness < expected) {
        brightness++;
    } else if (brightness > expected) {
        brightness--;
    }
    static int hits = 0;
    hits++;
    if (hits >= 50) {
        hits = 0;
        HAL_ADC_Start_IT(&hadc1);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    // Read & Update The ADC Result
    current_light = HAL_ADC_GetValue(hadc);
}

/* -------------------------------------------------------------------------- */
/*                                    ALARM                                   */
/* -------------------------------------------------------------------------- */

static void Alarm_SetState(BOOL is_on, uint16_t period) {
    static BOOL ring_flag = FALSE;
    static uint16_t last_period = 0;
    if (period != last_period) {
        __HAL_TIM_SetCounter(&htim2, 0);
        __HAL_TIM_SET_AUTORELOAD(&htim2, period);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (period >> 1));
        last_period = period;
    }
    if (is_on && !ring_flag) {
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
        ring_flag = 1;
        return;
    }
    if (!is_on && ring_flag) {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
        ring_flag = 0;
        return;
    }
}

static void Alarm_Load() {
    uint32_t state1 = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
    uint32_t state2 = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
    uint32_t state3 = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4);
    if (state1 == 0 || (state1 & 0xFF) == 0) {
        enable_alarm = FALSE;
    } else {
        enable_alarm = TRUE;
    }
    alarm_time.Hours = (state2 >> 8) & 0xFF;
    alarm_time.Minutes = state2 & 0xFF;
    alarm_time.Seconds = (state1 >> 8) & 0xFF;

    alarm_music = state3 & 0xFF;
}

static void Alarm_Save() {
    uint32_t state1 = 0;
    uint32_t state2 = 0;
    uint32_t state3 = alarm_music;
    if (enable_alarm) state1 |= 0x01;
    state1 |= alarm_time.Seconds << 8;
    state2 |= alarm_time.Minutes;
    state2 |= alarm_time.Hours << 8;
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, state1);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, state2);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, state3);
}

#define NYAN_CAT_LENGTH 32
#define NYAN_CAT_SPEED 3
uint32_t nyan_cat_data[] = {
    0xd1e1ab18, 0x98717191, 0xa19879bd, 0xebd9b797, 0xb1d1ebd9, 0xb7ab9879,
    0xa1789d9a, 0x87917191, 0xd1e1ab18, 0x98717191, 0xa19879bd, 0xebd9b797,
    0xb1d1ebd9, 0xb7ab9879, 0xa1789d9a, 0x87917171, 0x71457145, 0x79b7cbcd,
    0x71714574, 0xcb973234, 0x71457145, 0x779b7454, 0x71767457, 0xcbcd7161,
    0x71457145, 0x79b7cbcd, 0x71714574, 0xcb973234, 0x71457145, 0x779b7454,
    0x71767457, 0xcbcd7191,
};

#define ONLY_MY_RAILGUN_LENGTH 32
#define ONLY_MY_RAILGUN_SPEED 3
uint32_t only_my_railgun_data[] = {
    0x21031021, 0x00716141, 0x41551141, 0x41551141, 0x41551151, 0x51617191,
    0x61114111, 0xc1119111, 0x911911b1, 0x00716141, 0x41551141, 0x41551141,
    0x41551151, 0x51617191, 0x51111111, 0x51617191, 0x61141121, 0x00716141,
    0x41551141, 0x41551141, 0x41551151, 0x51617191, 0x51111111, 0x51617191,
    0x61114111, 0x21114111, 0x41551171, 0x61421141, 0x41551111, 0x11111111,
    0x11111111, 0x11111111,
};

uint16_t note_period_b[] = {
    0,     0,     25723, 24242, 21621, 19277, 17167, 16194,
    15238, 14440, 13628, 12861, 12139, 10810, 9638,
};

#define ZENBON_ZAKURA_LENGTH 80
#define ZENBON_ZAKURA_SPEED 2
uint32_t zenbon_zakura_data[] = {
    0x91a16565, 0x91a16565, 0x91a16565, 0x81716151, 0x91a16565, 0x91a16565,
    0x91a1c1f1, 0xefedc1a1, 0x91a16565, 0x91a16565, 0x91a16565, 0x81716151,
    0xa9acdca9, 0x6181a1c1, 0xd1d111c1, 0xd1111111, 0x61116115, 0x618191a1,
    0x61116115, 0x61513151, 0x61116115, 0x618191a1, 0xa1119111, 0x81116111,
    0x61116115, 0x618191a1, 0x61116115, 0x61513151, 0x61116115, 0x51618191,
    0xa1119111, 0x81116111, 0x81117111, 0x61115111, 0x51563121, 0x31111111,
    0x31516111, 0x91117111, 0x81117151, 0x61111111, 0x81117111, 0x61115111,
    0x51563121, 0x31113151, 0x61611161, 0x81119111, 0x71111111, 0x11116181,
    0x911911a1, 0xa11111a1, 0xc1d19181, 0xa1116181, 0x919111a1, 0xa111a1a1,
    0xb1a19181, 0x81116181, 0x911911a1, 0xa11111a1, 0xc1d19181, 0xa1116181,
    0xb111a111, 0x91118111, 0x81917151, 0x61116181, 0x911911a1, 0xa11111a1,
    0xc1d19181, 0xa1116181, 0x919111a1, 0xa111a1a1, 0xb1a19181, 0x81116181,
    0x911911a1, 0xa11111a1, 0xc1d19181, 0xa1116181, 0xb111a111, 0x91118111,
    0x9181a1c1, 0xd1111111,

};

uint16_t note_period_f[] = {0,     0,     20408, 18181, 17167, 15296,
                            13628, 12139, 11461, 10204, 9090,  8583,
                            7648,  6808,  6065,  5726};

int music_alt, music_tick, music_speed;
uint32_t *music_ptr;
uint32_t *music_end;
uint16_t *note_period;
int last_note;
static void Alarm_InitMusic(int id) {
    music_alt = music_tick = 0;
    last_note = 2;
    switch (id) {
        case MUSIC_NYAN_CAT:
            music_ptr = nyan_cat_data;
            music_end = nyan_cat_data + NYAN_CAT_LENGTH;
            music_speed = NYAN_CAT_SPEED;
            note_period = note_period_b;
            break;
        case MUSIC_ONLY_MY_RAILGUN:
            music_ptr = only_my_railgun_data;
            music_end = only_my_railgun_data + ONLY_MY_RAILGUN_LENGTH;
            music_speed = ONLY_MY_RAILGUN_SPEED;
            note_period = note_period_b;
            break;
        case MUSIC_ZENBON_ZAKURA:
            music_ptr = zenbon_zakura_data;
            music_end = zenbon_zakura_data + ZENBON_ZAKURA_LENGTH;
            music_speed = ZENBON_ZAKURA_SPEED;
            note_period = note_period_f;
            break;
        default:
            music_ptr = NULL;
            break;
    }
}
static void Alarm_TickMusic() {
    if (music_ptr == NULL || music_ptr >= music_end) {
        Alarm_StopMusic();
        return;
    };
    music_tick++;
    if (music_tick % music_speed == 0) {
        music_alt++;
        if (music_alt >= 8) {
            music_alt = 0;
            music_ptr++;
            if (music_ptr >= music_end) {
                Alarm_StopMusic();
                return;
            }
        }
    }
    int cur = ((*music_ptr) >> ((7 - music_alt) * 4)) & 0xF;
    if (cur == 0) {
        Alarm_SetState(FALSE, note_period[last_note]);
    } else if (cur == 1) {
        Alarm_SetState(TRUE, note_period[last_note]);
    } else {
        if (music_tick % music_speed != 0) {
            Alarm_SetState(TRUE, note_period[cur]);
            last_note = cur;
        } else {
            Alarm_SetState(FALSE, note_period[cur]);
        }
    }
}
static void Alarm_StopMusic() {
    Alarm_SetState(FALSE, 0xC00);
    music_ptr = NULL;
}

/* -------------------------------------------------------------------------- */
/*                                    DHT11                                   */
/* -------------------------------------------------------------------------- */

static void delay_us(uint32_t us) {
    __IO uint32_t currentTicks = SysTick->VAL;
    /* Number of ticks per millisecond */
    const uint32_t tickPerMs = SysTick->LOAD + 1;
    /* Number of ticks to count */
    const uint32_t nbTicks = ((us - ((us > 0) ? 1 : 0)) * tickPerMs) / 1000;
    /* Number of elapsed ticks */
    uint32_t elapsedTicks = 0;
    __IO uint32_t oldTicks = currentTicks;
    do {
        currentTicks = SysTick->VAL;
        elapsedTicks += (oldTicks < currentTicks)
                            ? tickPerMs + oldTicks - currentTicks
                            : oldTicks - currentTicks;
        oldTicks = currentTicks;
    } while (nbTicks > elapsedTicks);
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_Start(void) {
    Set_Pin_Output(DHT11_GPIO_Port, DHT11_Pin);
    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, 0);
    delay_us(18000);
    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, 1);
    delay_us(20);
    Set_Pin_Input(DHT11_GPIO_Port, DHT11_Pin);
}

int DHT11_WaitFor(int state) {
    int timer = 0;
    while (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin) != state) {
        delay_us(1);
        timer++;
        if (timer > 1000) {
            return 1;
        }
    }
    return 0;
}

uint8_t DHT11_Check_Response(void) {
    uint8_t Response = 0;

    delay_us(40);
    if (!(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))) {
        delay_us(80);
        if ((HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)))
            Response = 1;
        else
            Response = -1;
    }
    if (DHT11_WaitFor(0)) return -1;

    return Response;
}

uint8_t DHT11_Read(void) {
    uint8_t i = 0, j = 0;
    for (j = 0; j < 8; j++) {
        if (DHT11_WaitFor(1)) return 0;
        delay_us(40);
        if (!(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))) {
            i &= ~(1 << (7 - j));
        } else
            i |= (1 << (7 - j));
        if (DHT11_WaitFor(0)) return 0;
    }
    return i;
}

static void DHT11_Run() {
    DHT11_Start();
    int Presence = DHT11_Check_Response();
    if (Presence == 1) {
        uint8_t Rh_byte1 = DHT11_Read();
        uint8_t Rh_byte2 = DHT11_Read();
        uint8_t Temp_byte1 = DHT11_Read();
        uint8_t Temp_byte2 = DHT11_Read();
        uint8_t SUM = DHT11_Read();

        if (Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2 != SUM) {
            UART_Write_Text("DHT11 validation failed.");
        }

        temp = Temp_byte1;
        humi = Rh_byte1;
        UART_Write_Text("Temp: ");
        UART_Write_Int(temp);
        UART_Write_NewLine();
        UART_Write_Text("RH: ");
        UART_Write_Int(humi);
    } else {
        UART_Write_Text("No response from DHT11");
    }
}

/* -------------------------------------------------------------------------- */
/*                                     APP                                    */
/* -------------------------------------------------------------------------- */

static void App_Tick() {
    switch (page) {
        case PAGE_HOME:
            App_HomePage();
            break;
        case PAGE_MS:
            App_MSPage();
            break;
        case PAGE_ALARM:
            App_AlarmPage();
            break;
        case PAGE_MENU:
            App_MenuPage();
            break;
        case PAGE_SET_TIME:
            App_SetTimePage();
            break;
        case PAGE_TEMP_HUMI:
            App_TempHumiPage();
            break;
        case PAGE_SET_ALARM:
            App_SetAlarmPage();
            break;
        case PAGE_MUSIC:
            App_MusicPage();
            break;
        default:
            page = PAGE_HOME;
            break;
    }
}

static void App_HomePage() {
    Display_Set_CurrentTime_HM();
    if (enable_alarm) {
        RTC_TimeTypeDef sTime1;
        HAL_RTC_GetTime(&hrtc, &sTime1, RTC_FORMAT_BCD);
        if (sTime1.Hours == alarm_time.Hours &&
            sTime1.Minutes == alarm_time.Minutes &&
            sTime1.Seconds == alarm_time.Seconds) {
            Alarm_InitMusic(MUSIC_ONLY_MY_RAILGUN);
            page = PAGE_ALARM;
            return;
        }
    }
    if (KEY_R) {
        page = PAGE_MS;
        return;
    }
    if (KEY_U || KEY_D) {
        page = PAGE_MENU;
        return;
    }
}

static void App_MSPage() {
    Display_Set_CurrentTime_MS();
    if (KEY_L) {
        page = PAGE_HOME;
        return;
    }
    if (KEY_U || KEY_D) {
        page = PAGE_MENU;
        return;
    }
}

BOOL App_MenuOption(int cursor) {
    switch (cursor) {
        case 0:
            page = PAGE_SET_TIME;
            break;
        case 1:
            page = PAGE_TEMP_HUMI;
            break;
        case 2:
            page = PAGE_MUSIC;
            break;
        case 4:
            enable_alarm = !enable_alarm;
            Alarm_Save();
            return FALSE;
        case 5:
            page = PAGE_SET_ALARM;
            break;
        case 7:
            page = PAGE_HOME;
            break;
        default:
            return FALSE;
    }
    return TRUE;
}

static void App_MenuPage() {
    static int cursor = 0;
    if (cursor < 0 || cursor > 7) {
        cursor = 0;
    }
    display_buffer[0] = display_buffer[1] = display_buffer[2] =
        display_buffer[3] = 0;
    if (enable_alarm) display_buffer[0] |= LED_DP_Pin;
    if (cursor < 4) {
        display_buffer[cursor] |= LED_A_Pin;
        if (KEY_U) {
            if (App_MenuOption(cursor)) cursor = 0;
            return;
        }
        if (KEY_D) {
            cursor += 4;
            return;
        }
    } else {
        display_buffer[cursor - 4] |= LED_D_Pin;
        if (KEY_D) {
            if (App_MenuOption(cursor)) cursor = 0;
            return;
        }
        if (KEY_U) {
            cursor -= 4;
            return;
        }
    }
    if (KEY_L) {
        if (cursor > 0) cursor--;
        return;
    }
    if (KEY_R) {
        if (cursor < 7) cursor++;
    }
}

static void App_SetTimePage() {
    static int cursor = -1;
    static char buffer[4] = {0};
    RTC_TimeTypeDef sTime1;
    HAL_RTC_GetTime(&hrtc, &sTime1, RTC_FORMAT_BCD);
    if (cursor == -1) {
        buffer[0] = sTime1.Hours / 16 + '0';
        buffer[1] = sTime1.Hours % 16 + '0';
        buffer[2] = sTime1.Minutes / 16 + '0';
        buffer[3] = sTime1.Minutes % 16 + '0';
        cursor = 0;

        Keys_SetMode(KEY_ID_U, KEY_MODE_LONG);
        Keys_SetMode(KEY_ID_D, KEY_MODE_LONG);
    }

    if (sTime1.Seconds & 1) {
        Display_SetChar(buffer, TRUE);
    } else {
        char buffer1[] = {0};
        buffer1[cursor] = buffer[cursor];
        Display_SetChar(buffer1, TRUE);
    }

    if (KEY_R) {
        if (cursor >= 3) {
            cursor = -1;
            sTime1.Hours = ((buffer[0] - '0') << 4) + (buffer[1] - '0');
            sTime1.Minutes = ((buffer[2] - '0') << 4) + (buffer[3] - '0');
            sTime1.Seconds = 0x00;
            HAL_RTC_SetTime(&hrtc, &sTime1, RTC_FORMAT_BCD);

            Keys_SetMode(KEY_ID_U, KEY_MODE_ONCE);
            Keys_SetMode(KEY_ID_D, KEY_MODE_ONCE);
            page = PAGE_HOME;
        } else {
            cursor++;
        }
        return;
    }
    if (KEY_L) {
        if (cursor <= 0) {
            cursor = -1;
            page = PAGE_HOME;
        } else {
            cursor--;
        }
        return;
    }
    if (KEY_U) {
        char upper = '0';
        switch (cursor) {
            case 0:
                upper = '2';
                break;
            case 1:
                if (buffer[0] == '2') {
                    upper = '3';
                } else {
                    upper = '9';
                }
                break;
            case 2:
                upper = '5';
                break;
            case 3:
                upper = '9';
                break;
        }
        if (buffer[cursor] < upper) {
            buffer[cursor]++;
            if (cursor == 0 && buffer[0] == 2 && buffer[1] > '3') {
                buffer[1] = '3';
            }
        }
        return;
    }
    if (KEY_D) {
        if (buffer[cursor] > '0') {
            buffer[cursor]--;
        }
    }
}

static void App_AlarmPage() {
    static BOOL ringing = FALSE;

    if (!ringing) {
        ringing = TRUE;
        Alarm_InitMusic(alarm_music);
    }

    RTC_TimeTypeDef sTime1;
    HAL_RTC_GetTime(&hrtc, &sTime1, RTC_FORMAT_BCD);
    if (alarm_music) {
        Alarm_TickMusic();
    }

    if (sTime1.Seconds & 1) {
        Display_SetChar("8888", FALSE);
        if (alarm_music == 0) {
            Alarm_SetState(TRUE, 0xC00);
        }
    } else {
        Display_Set_CurrentTime_HM();
        if (alarm_music == 0) {
            Alarm_SetState(FALSE, 0xC00);
        }
    }

    if (KEY_L || KEY_R || KEY_U || KEY_D ||
        (alarm_music && music_ptr == NULL)) {
        Alarm_StopMusic();
        ringing = FALSE;
        page = PAGE_HOME;
        return;
    }
}

static void App_TempHumiPage() {
    static int stage = 0;
    if (stage == 0) {
        DHT11_Run();
        stage = 1;
    }
    if (stage == 1) {
        Display_SetInt(temp);
    }
    if (stage == 2) {
        Display_SetInt(humi);
    }
    if (KEY_R) {
        stage++;
        if (stage > 2) {
            stage = 0;
            page = PAGE_HOME;
            return;
        }
    }
}

static void App_SetAlarmPage() {
    static int cursor = -1;
    static char buffer[4] = {0};
    RTC_TimeTypeDef sTime1;
    HAL_RTC_GetTime(&hrtc, &sTime1, RTC_FORMAT_BCD);
    if (cursor == -1) {
        buffer[0] = alarm_time.Hours / 16 + '0';
        buffer[1] = alarm_time.Hours % 16 + '0';
        buffer[2] = alarm_time.Minutes / 16 + '0';
        buffer[3] = alarm_time.Minutes % 16 + '0';
        cursor = 0;

        Keys_SetMode(KEY_ID_U, KEY_MODE_LONG);
        Keys_SetMode(KEY_ID_D, KEY_MODE_LONG);
    }

    if (sTime1.Seconds & 1) {
        Display_SetChar(buffer, TRUE);
    } else {
        char buffer1[] = {0};
        buffer1[cursor] = buffer[cursor];
        Display_SetChar(buffer1, TRUE);
    }

    if (KEY_R) {
        if (cursor >= 3) {
            cursor = -1;
            alarm_time.Hours = ((buffer[0] - '0') << 4) + (buffer[1] - '0');
            alarm_time.Minutes = ((buffer[2] - '0') << 4) + (buffer[3] - '0');
            alarm_time.Seconds = 0x00;
            Alarm_Save();
            Keys_SetMode(KEY_ID_U, KEY_MODE_ONCE);
            Keys_SetMode(KEY_ID_D, KEY_MODE_ONCE);

            page = PAGE_HOME;
        } else {
            cursor++;
        }
        return;
    }
    if (KEY_L) {
        if (cursor <= 0) {
            cursor = -1;
            page = PAGE_HOME;
        } else {
            cursor--;
        }
        return;
    }
    if (KEY_U) {
        char upper = '0';
        switch (cursor) {
            case 0:
                upper = '2';
                break;
            case 1:
                if (buffer[0] == '2') {
                    upper = '3';
                } else {
                    upper = '9';
                }
                break;
            case 2:
                upper = '5';
                break;
            case 3:
                upper = '9';
                break;
        }
        if (buffer[cursor] < upper) {
            buffer[cursor]++;
            if (cursor == 0 && buffer[0] == 2 && buffer[1] > '3') {
                buffer[1] = '3';
            }
        }
        return;
    }
    if (KEY_D) {
        if (buffer[cursor] > '0') {
            buffer[cursor]--;
        }
    }
}

static void App_MusicPage() {
    Display_SetInt(alarm_music);
    Alarm_TickMusic();
    if (KEY_L) {
        page = PAGE_HOME;
        Alarm_StopMusic();
        return;
    }
    if (KEY_U) {
        if (alarm_music < 3) {
            alarm_music++;
            Alarm_Save();
        }
        return;
    }
    if (KEY_D) {
        if (alarm_music > 0) {
            alarm_music--;
            Alarm_Save();
        }
        return;
    }
    if (KEY_R) {
        Alarm_InitMusic(alarm_music);
        return;
    }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
    __disable_irq();
    while (1) {
        HAL_Delay(500);
        HAL_GPIO_TogglePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin);
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
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
