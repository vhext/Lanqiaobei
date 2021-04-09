#include "Features.h"

extern ADC_HandleTypeDef hadc2;

extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

volatile uint8_t Key_Click = 0x00;
volatile float R37V = 0.0;
volatile uint8_t Mode = MODE_AUTO;
volatile uint8_t UI = UI_DATA;

volatile uint8_t PA6V = 10;
volatile uint8_t PA7V = 20;

char ui_str[2][20] = {
    "      Data    ",
    "      Para    "};

char mode_str[2][20] = {
    "    Mode:AUTO    ",
    "    Mode:MANU    "};

struct KeyInfo KeyList[KEY_MAX_SUM] = {
    {GPIOB, GPIO_PIN_0, 0xff},
    {GPIOB, GPIO_PIN_1, 0xff},
    {GPIOB, GPIO_PIN_2, 0xff},
    {GPIOA, GPIO_PIN_0, 0xff}
};

KeyCallback KeyHadnle[KEY_MAX_SUM] = {
    B1_Short,
    B2_Short,
    B3_Short,
    B4_Short
};

void LED_Set(uint8_t led)
{
    HAL_GPIO_WritePin(GPIOC, LED_ALL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, (uint16_t)led << 8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

void Status_Init(void)
{
    LCD_Init();
    LCD_Clear(Black);
    LCD_SetBackColor(Black);
    LCD_SetTextColor(White);

    PWM_Start();
}

void LCD_Show(void)
{
    char str[20] = {};
    LCD_DisplayStringLine(Line0, (u8*)(&ui_str[UI]));
    if (UI == UI_DATA)
    {
        sprintf(str, "    V:%.2fV     ", R37V);
        LCD_DisplayStringLine(Line2, (u8*)str);
        LCD_DisplayStringLine(Line4, (u8*)(&mode_str[Mode]));
    }
    else
    {
        sprintf(str, "    PA6:%d%%    ", PA6V);
        LCD_DisplayStringLine(Line2, (u8*)str);
        sprintf(str, "    PA7:%d%%    ", PA7V);
        LCD_DisplayStringLine(Line4, (u8*)str);
    }
}

void UI_Show(void)
{
    LCD_Show();
    LED_Set((Mode | (UI << 0x01)) ^ 0x03);
}

void Status_Update(void)
{
    // 电压采集
    HAL_ADC_Start(&hadc2);
    R37V = (HAL_ADC_GetValue(&hadc2)  / 4095.0) * 3.3;
    HAL_ADC_Stop(&hadc2);

    // 是否在AUTO模式
    if(Mode == MODE_AUTO)
    {
        PA6V = (uint8_t)((R37V * 100.0) / 3.3);
        PA7V = (uint8_t)((R37V * 100.0) / 3.3);
        // 更新PWM
        PWM_Refresh();
    }
}

void PWM_Start(void)
{
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
}

void PWM_Refresh()
{
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, PA6V * 100);
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, PA7V * 50);
}

void Key_Scan(void)
{
    for (size_t i = 0; i < KEY_MAX_SUM; i++)
    {
        if(HAL_GPIO_ReadPin(KeyList[i].port, KeyList[i].pin) == GPIO_PIN_SET)
        {
            if((KeyList[i].status & 0x01) == 0x00)
            {
                // 短按
                Key_Click = 0x01<<i;
                KeyList[i].status = 0xff;
            }
        }
        else
        {
            KeyList[i].status <<= 1;
        }
    }
    
}

void Key_Handle(void)
{
    for (size_t i = 0; i < KEY_MAX_SUM; i++)
    {
        if(Key_Click == 0x01 << i)
        {
            KeyHadnle[i]();
            Key_Click = 0x00;
        }
    }
    
}

void B1_Short(void)
{
    UI = (UI == UI_DATA)? UI_PARA : UI_DATA;
}

void B2_Short(void)
{
    if(UI == UI_PARA)
    {
        PA6V = (PA6V + 10 > 90)? 10 : PA6V + 10;
        PWM_Refresh();
    }
}

void B3_Short(void)
{
    if(UI == UI_PARA)
    {
        PA7V = (PA7V + 10 > 90)? 10 : PA7V + 10;
        PWM_Refresh();
    }    
}

void B4_Short(void)
{
    Mode = (Mode == MODE_AUTO)? MODE_MANU : MODE_AUTO;
}


