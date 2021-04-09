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
#include <stdio.h>
#include <string.h>
#include "lcd.h"
#include "Features.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY_HANDLE_SUM 8

#define UI_MAIN 0x00
#define UI_SETTING 0x01

#define STATUS_NORMAL 0x00
#define STATUS_UPPER 0x01
#define STATUS_LOWER 0x02

#define ALARM_UPPER 0x00
#define ALARM_LOWER 0x01

#define LD1 0x00
#define LD2 0x01
#define LD3 0x02
#define LD4 0x03
#define LD5 0x04
#define LD6 0x05
#define LD7 0x06
#define LD8 0x07

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

/* USER CODE BEGIN PV */

typedef void (*Callback)(void);

// 高四-长按   低四-短按
volatile uint8_t KeyDown = 0x00;

volatile uint64_t TimoutTick = 0;

volatile uint8_t UI = UI_MAIN;
volatile uint8_t Status = STATUS_NORMAL;

volatile float Alarm_V[2] = {2.4, 1.2};
volatile uint8_t Alarm_LD[2] = {LD1, LD2};

volatile uint64_t Led_tim = 0x00;

volatile uint8_t Setting_Item = 0x00;

float R37V = 0;

char StatusStr[3][8] = {
    "Normal",
    "Upper",
    "Lower"};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

void KeyHandleFun(void);

void GetR37(void);
void Update_Status(void);
void LCD_UI_Main(void);
void LCD_UI_Setting(void);
void LED_Show(void);

void B1_short(void);
void B2_short(void);
void B3_short(void);
void B4_short(void);
/* USER CODE END PFP */
Callback KeyHandles[KEY_HANDLE_SUM] = {
    B1_short, // B1短按
    B2_short, // B2短按
    B3_short, // B3短按
    B4_short  // B4短按
};

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
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  LCD_Clear(White);
  LCD_SetBackColor(White);
  LCD_SetTextColor(Black);

  M4_LED_Reset(0xff);
  // M4_LED_Set(0x01);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    KeyHandleFun();
    GetR37();
    Update_Status();
    if(UI == UI_SETTING)
    {
      LCD_UI_Setting();
    }
    else
    {
      LCD_UI_Main();
    }
    LED_Show();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7
                           PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void KeyHandleFun(void)
{
  for (size_t i = 0; i < KEY_HANDLE_SUM; i++)
  {
    if (KeyDown & (0x01 << i))
    {
      KeyHandles[i]();
      KeyDown &= ~(0x01 << i);
    }
  }
  return;
}

void GetR37(void)
{
  R37V = M4_ADC_Start() / 4095.0 * 3.3;
}

void Update_Status(void)
{
  Status = (R37V > Alarm_V[ALARM_UPPER]) ? STATUS_UPPER : ((R37V < Alarm_V[ALARM_LOWER]) ? STATUS_LOWER : STATUS_NORMAL);
}

void LCD_UI_Main(void)
{
  char str[2][21] = {};

  sprintf(str[0], "   Volt: %.2fV     ", R37V);
  sprintf(str[1], "   Status: %s  ", StatusStr[Status]);

  LCD_DisplayStringLine(Line2, (u8 *)"        Main       ");
  LCD_DisplayStringLine(Line4, (u8 *)str[0]);
  LCD_DisplayStringLine(Line5, (u8 *)"                   ");
  LCD_DisplayStringLine(Line6, (u8 *)str[1]);
  LCD_DisplayStringLine(Line7, (u8 *)"                   ");
}

void LCD_UI_Setting(void)
{
  char str[4][22] = {};

  sprintf(str[0], "  Max Volt: %.2fV   ", Alarm_V[ALARM_UPPER]);
  sprintf(str[1], "  Min Volt: %.2fV   ", Alarm_V[ALARM_LOWER]);
  sprintf(str[2], "  Upper: LD%d       ", Alarm_LD[ALARM_UPPER] + 1);
  sprintf(str[3], "  Lower: LD%d       ", Alarm_LD[ALARM_LOWER] + 1);

  LCD_DisplayStringLine(Line2, (u8 *)"      Setting     ");

  for (size_t i = 0 , j = 0 , refcolumn = 319 - 1 * 16; i < 4; i++)
  {
    if(i == Setting_Item)
    {
      LCD_SetBackColor(Black);
      LCD_SetTextColor(White);
      char * ptr = str[i];
      ptr += 1;
      while ((*ptr != 0) && (j < 18))	 //	20
      {
        LCD_DisplayChar(Line4 + i * 24, refcolumn, *ptr);
        refcolumn -= 16;
        ptr++;
        j++;
      }
      LCD_SetBackColor(White);
      LCD_SetTextColor(Black);
      continue;        
    }

    LCD_DisplayStringLine(Line4 + i * 24, (u8 *)str[i]);
  }

}

void LED_Show(void)
{
  if(Status == STATUS_NORMAL)
  {
    M4_LED_Reset(0xff);
    return;
  }

  if(Led_tim / 200 == 1)
  {
    M4_LED_Reset(0xff);
  }
  else if(Led_tim > 400)
  {
    Led_tim = 0;
  }
  else
  {
    M4_LED_Set(0x01 << Alarm_LD[Status - 1]);
  }
}

void B1_short(void)
{
  UI = (UI == UI_SETTING)? UI_MAIN : UI_SETTING;
}

void B2_short(void)
{
  if(UI == UI_MAIN)
    return;
  Setting_Item = (Setting_Item + 1) % 4;

}

void B3_short(void)
{
  if(UI == UI_MAIN)
    return;
  switch (Setting_Item)
  {
  case 0x00:
    {
      // Upper V > Lower V
      if(Alarm_V[ALARM_UPPER] + 0.3 > Alarm_V[ALARM_LOWER] && Alarm_V[ALARM_UPPER] + 0.3 < 3.4)
      {
        Alarm_V[ALARM_UPPER] += 0.3;
      }
    }
    break;
  case 0x01:
    {
      // Lower V < Upper V
      if(Alarm_V[ALARM_LOWER] + 0.3 < Alarm_V[ALARM_UPPER])
      {
        Alarm_V[ALARM_LOWER] += 0.3;
      }     
    }
    break;
  case 0x02:
    {
      // LD Upper != LD Lower
      if((Alarm_LD[ALARM_UPPER] + 1) % 8 == Alarm_LD[ALARM_LOWER])
      {
        Alarm_LD[ALARM_UPPER] = (Alarm_LD[ALARM_UPPER] + 2) % 8;
      }
      else
      {
        Alarm_LD[ALARM_UPPER] = (Alarm_LD[ALARM_UPPER] + 1) % 8;
      }
    }
    break;
  case 0x03:
    {
      // LD Lower != LD Upper
      if((Alarm_LD[ALARM_LOWER] + 1) % 8 == Alarm_LD[ALARM_UPPER])
      {
        Alarm_LD[ALARM_LOWER] = (Alarm_LD[ALARM_LOWER] + 2) % 8;
      }
      else
      {
        Alarm_LD[ALARM_LOWER] = (Alarm_LD[ALARM_LOWER] + 1) % 8;
      }
    }
    break;  
  default:
    break;
  }
  
}

void B4_short(void)
{
  if(UI == UI_MAIN)
    return;
  switch (Setting_Item)
  {
  case 0x00:
    {
      // Upper V > Lower V
      if(Alarm_V[ALARM_UPPER] - 0.3 > Alarm_V[ALARM_LOWER])
      {
        Alarm_V[ALARM_UPPER] -= 0.3;
      }
    }
    break;
  case 0x01:
    {
      // Lower V < Upper V
      if(Alarm_V[ALARM_LOWER] - 0.3 < Alarm_V[ALARM_UPPER] && Alarm_V[ALARM_LOWER] - 0.3 > 0)
      {
        Alarm_V[ALARM_LOWER] -= 0.3;
      }     
    }
    break;
  case 0x02:
    {
      // LD Upper != LD Lower
      if((uint8_t)(Alarm_LD[ALARM_UPPER] - 1) % 8 == Alarm_LD[ALARM_LOWER])
      {
        Alarm_LD[ALARM_UPPER] = (uint8_t)(Alarm_LD[ALARM_UPPER] - 2) % 8;
      }
      else
      {
        Alarm_LD[ALARM_UPPER] = (uint8_t)(Alarm_LD[ALARM_UPPER] - 1) % 8;
      }
    }
    break;
  case 0x03:
    {
      // LD Lower != LD Upper
      if((uint8_t)(Alarm_LD[ALARM_LOWER] - 1) % 8 == Alarm_LD[ALARM_UPPER])
      {
        Alarm_LD[ALARM_LOWER] = (uint8_t)(Alarm_LD[ALARM_LOWER] - 2) % 8;
      }
      else
      {
        Alarm_LD[ALARM_LOWER] = (uint8_t)(Alarm_LD[ALARM_LOWER] - 1) % 8;
      }
    }
    break;  
  default:
    break;
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
