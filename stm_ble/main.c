/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include "joystick.h"
#include "lcd.h"
#include "ir.h"
#include "flash_storage.h"
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#define ARR_CNT 5
#define CMD_SIZE 600

static inline void IR_ON(void)  { TIM3->CCER |= TIM_CCER_CC1E; }
static inline void IR_OFF(void) { TIM3->CCER &= ~TIM_CCER_CC1E; }
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t rx2char;
volatile unsigned char rx2Flag = 0;
volatile char rx2Data[400];
volatile unsigned char btFlag = 0;
uint8_t btchar;
char btData[400];

char recvBuf[CMD_SIZE]={0};
char sendBuf[CMD_SIZE+20]={0};

#define MAX_SAVE 100
#define NAME_LEN 16
#define MAX_PULSES 100

#define GRID_ROWS 6
#define GRID_COLS 8
#define SLOT_WIDTH 2

static const char *selection_grid[GRID_ROWS][GRID_COLS] = {
    { "A","B","C","D","E","F","G","H", },
    { "I","J","K","L","M","N","O","P", },
    { "Q","R","S","T","U","V","W","X", },
    { "Y","Z","0","1","2","3","4","5", },
    { "6","7","8","9","-","/","!","?"  },
};

static uint8_t sel_row = 0, sel_col = 3;

extern char     save_names[MAX_SAVE][NAME_LEN];
extern uint16_t save_ir   [MAX_SAVE][MAX_PULSES];
extern uint8_t  save_counts[MAX_SAVE];
extern uint8_t  save_count;
extern uint32_t flash_next_offset;

static bool save_mode_m = false;
static bool save_mode = false;
static bool use_mode  = false;
static bool name_mode = false;

static uint8_t menu_index = 0;
static uint8_t use_index = 0;

uint32_t lastInputTime = 0;
const uint32_t input_delay = 250;

static char    name_buffer[NAME_LEN+1];
static uint8_t name_pos      = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void bluetooth_Event();

void handle_main_menu(Direction dir, uint32_t now);
void handle_save_mode(Direction dir, uint32_t now);
void handle_name_mode(Direction dir, uint32_t now);
void handle_use_mode(Direction dir, uint32_t now);
void handle_save_mode_m(Direction dir, uint32_t now);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void lcd_update_name(void) {
    lcd_clear();

    lcd_set_cursor(0, 0);
    lcd_print(name_buffer);

    if (sel_row < GRID_ROWS-1) {
        for (uint8_t c = 0; c < GRID_COLS; c++) {
            uint8_t col = c * SLOT_WIDTH;
            lcd_set_cursor(1, col);
            lcd_print((c == sel_col) ? ">" : " ");
            char ch = selection_grid[sel_row][c][0];
            lcd_print(ch ? (char[]){ch,'\0'} : " ");
        }
    } else {
        lcd_set_cursor(1, 0);
        lcd_print("    OK     NO   ");
        if (sel_col == 3) {
            lcd_set_cursor(1, 3);
        } else {
            lcd_set_cursor(1, 10);
        }
        lcd_print(">");
    }
}

void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    while (__HAL_TIM_GET_COUNTER(&htim4) < us);
}

void sendRaw(const uint16_t *signal, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        if ((i & 1) == 0) {
            IR_ON();
        } else {
            IR_OFF();
        }
        delay_us(signal[i]);
    }
    IR_OFF();
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
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  lcd_init(&hi2c1);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim4);
	ir_reset();

	joystick_init(&hadc1, &huart2);
	joystick_start();

	LoadAllIRFromFlash();

  HAL_UART_Receive_IT(&huart2, &rx2char,1);
  HAL_UART_Receive_IT(&huart6, &btchar,1);

  lcd_welcome();
  lcd_update_menu(menu_index);
  printf("start main2()\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	joystick_task();
	Direction dir = getLastDirection();
	uint32_t now = HAL_GetTick();

	if (save_mode) {
		handle_save_mode(dir, now);
		continue;
	}

	if (save_mode_m) {
		handle_save_mode_m(dir, now);
		continue;
	}

	if (name_mode) {
	    handle_name_mode(dir, now);
	    continue;
	}

	if (use_mode) {
	    handle_use_mode(dir, now);
	    continue;
	}
	handle_main_menu(dir, now);

	  if(btFlag)
	  {
			btFlag =0;
			bluetooth_Event();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 25;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 9;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void bluetooth_Event()
{

  int i=0;
  char * pToken;
  char * pArray[ARR_CNT]={0};

  strcpy(recvBuf,btData);

  printf("btData : %s\r\n",btData);

  pToken = strtok(recvBuf,"[@]");
  while(pToken != NULL)
  {
    pArray[i] =  pToken;
    if(++i >= ARR_CNT)
      break;
    pToken = strtok(NULL,"[@]");
  }

  if(!strcmp(pArray[1],"UPLOAD"))
  {
	  save_mode_m = true;
	  lcd_clear();
	  lcd_set_cursor(0,0);
	  lcd_print("Save Mode");
	  lcd_set_cursor(1,0);
	  lcd_print("Waiting IR...");
	  ir_reset();
	  return;
  }
  if (!strcmp(pArray[1], "SHOT")) {
      uint16_t pulses[MAX_PULSES];
      uint16_t cnt = 0;

      const char *s = pArray[2];
      while (*s && cnt < MAX_PULSES) {
        char *endp;
        unsigned long v = strtoul(s, &endp, 10);     // 10진수 파싱
        if (endp == s) break;                        // 숫자 아님 → 중단
        if (v > 0xFFFF) v = 0xFFFF;                  // 상한 가드
        pulses[cnt++] = (uint16_t)v;

        if (*endp == ',') s = endp + 1;
        else s = endp;
      }

      if (cnt > 0) {
        sendRaw(pulses, cnt);
      }
      return;
    }
  if (!strcmp(pArray[1], "SAVEBLE") && pArray[2] && pArray[3]) {
      char *csv   = pArray[2];
      char *iname = pArray[3];

      uint16_t pulses[MAX_PULSES];
      uint8_t  cnt = 0;
      while (*csv && cnt < MAX_PULSES) {
          char *endp;
          unsigned long v = strtoul(csv, &endp, 10);
          if (endp == csv) break;                 // 숫자 아님 → 종료
          if (v > 0xFFFF) v = 0xFFFF;             // 상한 가드
          pulses[cnt++] = (uint16_t)v;

          if (*endp == ',') csv = endp + 1;
          else csv = endp;
      }

      if (cnt == 0) {
          sprintf(sendBuf, "[%s]SAVEBLE@ERR(NO_DATA)\n", pArray[0]);
          HAL_UART_Transmit(&huart6, (uint8_t*)sendBuf, strlen(sendBuf), 0xFFFF);
          return;
      }

      HAL_StatusTypeDef st = SaveIRToFlash(iname, pulses, cnt);

      if (st == HAL_OK) {
          if (save_count < MAX_SAVE) {
              // 이름 복사 (길이 제한 적용)
              strncpy(save_names[save_count], iname, NAME_LEN);
              save_names[save_count][NAME_LEN-1] = '\0';

              // 파형 복사
              for (uint8_t i = 0; i < cnt; i++) save_ir[save_count][i] = pulses[i];
              save_counts[save_count] = cnt;
              save_count++;
          }
          sprintf(sendBuf, "[%s]SAVEBLE@OK(%u)@%s\n", pArray[0], cnt, iname);
      } else {
          sprintf(sendBuf, "[%s]SAVEBLE@ERR(FLASH)\n", pArray[0]);
      }

      HAL_UART_Transmit(&huart6, (uint8_t*)sendBuf, strlen(sendBuf), 0xFFFF);
      return;
  }


  else if(!strncmp(pArray[1]," New conn",sizeof(" New conn")))
  {
      return;
  }
  else if(!strncmp(pArray[1]," Already log",sizeof(" Already log")))
  {
      return;
  }
  else
      return;
}
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART6 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
    	static int i=0;
    	rx2Data[i] = rx2char;
    	if((rx2Data[i] == '\r') || rx2Data[i] == '\n')
    	{
    		rx2Data[i] = '\0';
    		rx2Flag = 1;
    		i = 0;
    	}
    	else
    	{
    		i++;
    	}
    	HAL_UART_Receive_IT(&huart2, &rx2char,1);
    }
    if(huart->Instance == USART6)
    {
    	static int i=0;
    	btData[i] = btchar;
    	if((btData[i] == '\n') || btData[i] == '\r')
    	{
    		btData[i] = '\0';
    		btFlag = 1;
    		i = 0;
    	}
    	else
    	{
    		i++;
    	}
    	HAL_UART_Receive_IT(&huart6, &btchar,1);
    }
}

void handle_main_menu(Direction dir, uint32_t now)
{
  if (dir == DIR_CENTER || now - lastInputTime < input_delay) return;

  switch (dir) {
    case DIR_UP:
      if (menu_index > 0) menu_index--;
      break;
    case DIR_DOWN:
      if (menu_index < 4) menu_index++;
      break;
    case DIR_BTN:
      if (menu_index == 0) {
        use_mode = true;
		use_index = 0;
        lcd_update_use(use_index, save_count, save_names);
        return;
      }
      if (menu_index == 1) {
        save_mode = true;
        lcd_clear();
		lcd_set_cursor(0,0);
		lcd_print("Save Mode");
        lcd_set_cursor(1,0);
		lcd_print("Waiting IR...");
        ir_reset();
        return;
      }
			if (menu_index == 2) {
					sendRaw(LG_Monitor_IR, LG_Monitor_IR_LEN);
					HAL_Delay(110);
					sendRaw(LG_AC_ON_IR, LG_AC_ON_IR_LEN);
					print_status("Sending IRs...");
					lcd_update_menu(menu_index);
					return;
			}
			if (menu_index == 3) {
					sendRaw(LG_Monitor_IR, LG_Monitor_IR_LEN);
					HAL_Delay(110);
					sendRaw(LG_AC_OFF_IR, LG_AC_OFF_IR_LEN);
					print_status("Sending IRs...");
					lcd_update_menu(menu_index);
					return;
			}
			// menu_index == 4 처리부
			if (menu_index == 4) {
			    lcd_clear();
			    lcd_set_cursor(0,0);
			    lcd_print("Erasing...");
			    lcd_set_cursor(1,0);
			    lcd_print("Please wait");

			    HAL_SuspendTick();
			    __disable_irq();

			    HAL_StatusTypeDef st = Storage_Erase();

			    __enable_irq();
			    HAL_ResumeTick();

			    save_count = 0;
			    flash_next_offset = 0;

			    LoadAllIRFromFlash();

			    if (st == HAL_OK) {
			        print_status("All Cleared!");
			    } else {
			        print_status("Erase Error!");
			    }
			    lcd_update_menu(menu_index);
			    return;
			}
      break;
    default: break;
  }

  lcd_update_menu(menu_index);
  lastInputTime = now;
}

void handle_save_mode(Direction dir, uint32_t now)
{
  if (dir == DIR_LEFT) {
    save_mode = false;
    lcd_update_menu(menu_index);
    lastInputTime = now;
    return;
  }

  if (ir_key_ready) {
		print_status("Received!");

    uint8_t n = ir_count > MAX_PULSES ? MAX_PULSES : ir_count;
    for (uint8_t i = 0; i < n; i++) save_ir[save_count][i] = ir_times[i];
    save_counts[save_count] = n;

    save_mode = false;
    name_mode = true;
    name_pos = 0;
    name_buffer[0] = '\0';
    ir_reset();

    lcd_update_name();
    lastInputTime = now;
  }
}

void handle_name_mode(Direction dir, uint32_t now)
{
  if (now - lastInputTime < input_delay) return;
  bool need_update = false;

  switch (dir) {
    case DIR_LEFT:
      if (sel_row == GRID_ROWS - 1) {
        sel_col = 3;
			}
			else if (sel_col > 0) {
				sel_col--;
			}
			need_update = true;
      break;
    case DIR_RIGHT:
      if (sel_row == GRID_ROWS-1) {
        sel_col = 10;
      } else if (sel_col + 1 < GRID_COLS) {
        sel_col++;
      }
      need_update = true;
      break;
    case DIR_UP:
      if (sel_row > 0) { sel_row--; need_update = true; }
      break;
    case DIR_DOWN:
      if (sel_row + 1 < GRID_ROWS) { sel_row++; need_update = true; }
      break;
    case DIR_BTN: {
      const char *sel = selection_grid[sel_row][sel_col];

		if (sel[0] && sel_row < GRID_ROWS-1) {
        if (name_pos < NAME_LEN) name_buffer[name_pos++] = sel[0];
        name_buffer[name_pos] = '\0';
        lcd_set_cursor(0,0);
				lcd_print(name_buffer);
      }
      else if (sel_row == GRID_ROWS-1 && sel_col == 3) {
					HAL_StatusTypeDef st = SaveIRToFlash(
          name_buffer, save_ir[save_count], save_counts[save_count]
        );
        if (st == HAL_OK) {
          strncpy(save_names[save_count], name_buffer,NAME_LEN);
          save_names[save_count][NAME_LEN-1] = '\0';
          save_count++;
					print_status("Saved!");
        }
        name_mode = false;
				sel_row = sel_col = 0;
        lcd_update_menu(menu_index);
      }
      else if (sel_row == GRID_ROWS - 1 && sel_col == 10) {
				print_status("Canceled!");
        name_mode = false;
				sel_row = 0;
				sel_col = 3;
        lcd_update_menu(menu_index);
      }
      break;
    }
    default:
			break;
  }

  if (need_update) {
    lcd_update_name();
    lastInputTime = now;
  }
}

void handle_use_mode(Direction dir, uint32_t now)
{
  if (now - lastInputTime < input_delay) return;

  if (dir == DIR_UP && use_index > 0) {
    use_index--;
	lcd_update_use(use_index, save_count, save_names);
    lastInputTime = now;
  }
  else if (dir == DIR_DOWN && use_index + 1 < save_count) {
    use_index++;
	lcd_update_use(use_index, save_count, save_names);
    lastInputTime = now;
  }
  else if (dir == DIR_BTN) {
    sendRaw(save_ir[use_index], save_counts[use_index]);
	lcd_clear();
	lcd_set_cursor(0, 0);
	lcd_print("Sending IR...");
	HAL_Delay(500);
	lcd_update_use(use_index, save_count, save_names);
    lastInputTime = now;
  }
  else if (dir == DIR_LEFT) {
    use_mode = false;
	lcd_update_menu(menu_index);
    lastInputTime = now;
  }
}

void handle_save_mode_m(Direction dir, uint32_t now)
{
    if (dir == DIR_LEFT) {
        save_mode_m = false;
        lcd_update_menu(menu_index);
        lastInputTime = now;
        return;
    }

    if (ir_key_ready) {
        print_status("Received!");

        uint8_t n = ir_count > MAX_PULSES ? MAX_PULSES : ir_count;
        for (uint8_t i = 0; i < n; i++) {
            save_ir[save_count][i] = ir_times[i];
        }
        save_counts[save_count] = n;

        save_mode_m = false;
        ir_reset();

        lastInputTime = now;

        char dataBuf[CMD_SIZE];
        int pos = 0;

        for (uint8_t i = 0; i < n; i++) {
            pos += sprintf(&dataBuf[pos], "%u", save_ir[save_count][i]);
            if (i < n - 1) {
                dataBuf[pos++] = ',';
            }
        }
        dataBuf[pos] = '\0';

        sprintf(sendBuf, "[LDH_SQL]IRSAVE@%s\n", dataBuf);
        HAL_UART_Transmit(&huart6, (uint8_t *)sendBuf, strlen(sendBuf), 0xFFFF);

        lcd_update_menu(menu_index);
		lastInputTime = now;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_4) {
        joystick_buttonIRQHandler(GPIO_Pin);
    } else if (GPIO_Pin == GPIO_PIN_5){
        ir_exti_callback(GPIO_Pin);
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
