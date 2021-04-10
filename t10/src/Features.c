#include "Features.h"

volatile uint8_t UI = UI_MAIN;
volatile uint8_t Status = STATUS_NORMAL;

volatile uint8_t KeyClick = 0x00;

volatile float R37V = 0.0;

volatile uint8_t ChooseItem = 0x00;

volatile uint8_t AlarmFlag = 0x00;

extern ADC_HandleTypeDef hadc2;

volatile uint32_t LEDCount = 0;

volatile float SettingItem[SETTING_ITEM_SUM] = {
    2.4, // Max Volt
    1.2, // Min Volt
    1.0, // Upper LED
    2.0  // Lower LED
};

struct KeyInfo KeyList[KEY_MAX_SUM] = {
    {GPIOB, GPIO_PIN_0, KEY_UP},
    {GPIOB, GPIO_PIN_1, KEY_UP},
    {GPIOB, GPIO_PIN_2, KEY_UP},
    {GPIOA, GPIO_PIN_0, KEY_UP}};

Callback KeyCallbacks[KEY_MAX_SUM] = {
    B1_Short,
    B2_Short,
    B3_Short,
    B4_Short};

char ui_str[2][20] = {
    "        Main      ",
    "       Setting    "};

const char status_str[3][8] = {
    "Normal",
    "Upper",
    "Lower"};

const char temp_str[SETTING_ITEM_SUM][22] = {
    "    Max Volt:%.2fV   ",
    "    Min Volt:%.2fV   ",
    "    Upper:LD%.0f     ",
    "    Lower:LD%.0f     "};

void LCD_Show(void)
{
    char str[20] = "                    ";

    LCD_DisplayStringLine(Line1, (u8 *)ui_str[UI]);
    if (UI == UI_MAIN)
    {
        LCD_DisplayStringLine(Line4, (u8*)str);
        LCD_DisplayStringLine(Line6, (u8*)str);
        sprintf(str, "    Volt: %.2fV    ", R37V);
        LCD_DisplayStringLine(Line3, (u8 *)str);
        sprintf(str, "    Status: %s     ", status_str[Status]);
        LCD_DisplayStringLine(Line5, (u8 *)str);

        return;
    }
    // Setting User Interface
    for (size_t i = 0; i < SETTING_ITEM_SUM; i++)
    {
        sprintf(str, temp_str[i], SettingItem[i]);

        u32 j = 0;
        u16 refcolumn = 319; //319;
        char *ptr = str;
        while ((*ptr != 0) && (j < 20)) //	20
        {
            if (i == ChooseItem && j == 4)
            {
                LCD_SetBackColor(White);
                LCD_SetTextColor(Black);
            }

            LCD_DisplayChar(Line3 + Line1 * i, refcolumn, *ptr);

            if (i == ChooseItem && j == 17)
            {
                LCD_SetBackColor(Black);
                LCD_SetTextColor(White);
            }

            refcolumn -= 16;
            ptr++;
            j++;
        }
    }
}

void LED_Refresh(void)
{
    if(LEDCount > 2 * LED_REFRESH_DELAY)
    {
        LEDCount = 0;
        return;
    }
    
    uint8_t status = (LEDCount < LED_REFRESH_DELAY)? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(GPIOC, ALL_LED_PIN, GPIO_PIN_SET);
    if (R37V > SettingItem[MAX_VOLT])
    {
        HAL_GPIO_WritePin(GPIOC, 0x0100 << ((uint8_t)SettingItem[UPPER_LD] - 1), status);
    }
    else if (R37V < SettingItem[MIN_VOLT])
    {
        HAL_GPIO_WritePin(GPIOC, 0x0100 << ((uint8_t)SettingItem[LOWER_LD] - 1), status);
    }

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

void UI_Init(void)
{
    LCD_Init();
    LCD_Clear(Black);
    LCD_SetBackColor(Black);
    LCD_SetTextColor(White);
}

void UI_Show(void)
{
    LCD_Show();
}

void Status_Update(void)
{
    HAL_ADC_Start(&hadc2);
    R37V = HAL_ADC_GetValue(&hadc2) * 3.3 / 4095.0;
    HAL_ADC_Stop(&hadc2);

    if (R37V >= SettingItem[MAX_VOLT])
        Status = STATUS_UPPER;
    else if (R37V <= SettingItem[MIN_VOLT])
        Status = STATUS_LOWER;
    else
        Status = STATUS_NORMAL;
}

void Key_Scan(void)
{
    for (size_t i = 0; i < KEY_MAX_SUM; i++)
    {
        if (HAL_GPIO_ReadPin(KeyList[i].port, KeyList[i].pin) == GPIO_PIN_SET)
        {
            if (KeyList[i].status == KEY_DOWN)
            {
                KeyClick = 0x01 << i;
            }
            KeyList[i].status = KEY_UP;
        }
        else
        {
            KeyList[i].status = KEY_DOWN;
        }
    }
}

void Key_Handle(void)
{
    for (size_t i = 0; i < KEY_MAX_SUM; i++)
    {
        if (KeyClick == 0x01 << i)
        {
            KeyCallbacks[i]();
            KeyClick = 0x00;
        }
    }
}

void B1_Short(void)
{
    UI = (UI == UI_MAIN) ? UI_SETTING : UI_MAIN;
    ChooseItem = 0;
}

void B2_Short(void)
{
    if (UI == UI_MAIN)
        return;
    ChooseItem = (ChooseItem + 1) % 4;
}

void B3_Short(void)
{
    if (UI == UI_MAIN)
        return;    
    if (ChooseItem / 2)
    {
        // Upper LD and Lower LD
        SettingItem[ChooseItem] = (SettingItem[ChooseItem] >= 8.0f) ? 1.0f : SettingItem[ChooseItem] + 1.0f;

        if (SettingItem[UPPER_LD] == SettingItem[LOWER_LD])
        {
            SettingItem[ChooseItem] = (SettingItem[ChooseItem] >= 8.0f) ? 1.0f : SettingItem[ChooseItem] + 1.0f;
        }
    }
    else
    {
        // Max Volt and Min Volt
        if (SettingItem[ChooseItem] < 3.30f)
        {
            if (ChooseItem == MIN_VOLT)
            {
                if (SettingItem[MIN_VOLT] + 0.3f >= SettingItem[MAX_VOLT])
                {
                    return;
                }
            }
            SettingItem[ChooseItem] += 0.3f;
        }
    }
}

void B4_Short(void)
{
    if (UI == UI_MAIN)
        return;    
    if (ChooseItem / 2)
    {
        // Upper LD and Lower LD
        SettingItem[ChooseItem] = (SettingItem[ChooseItem] <= 1.0f) ? 8.0f : SettingItem[ChooseItem] - 1.0f;

        if (SettingItem[UPPER_LD] == SettingItem[LOWER_LD])
        {
            SettingItem[ChooseItem] = (SettingItem[ChooseItem] <= 1.0f) ? 8.0f : SettingItem[ChooseItem] - 1.0f;
        }
    }
    else
    {
        // Max Volt and Min Volt
        if (SettingItem[ChooseItem] > 0.0f)
        {
            if (ChooseItem == MAX_VOLT)
            {
                if (SettingItem[MAX_VOLT] - 0.3f <= SettingItem[MIN_VOLT])
                {
                    return;
                }
            }
            SettingItem[ChooseItem] -= 0.3f;
        }
    }
}
