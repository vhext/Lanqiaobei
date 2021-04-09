#ifndef FEATURES_H
#define FEATURES_H

#include "lcd.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>

#define MODE_AUTO 0x00
#define MODE_MANU 0x01

#define UI_DATA 0x00 
#define UI_PARA 0x01

#define KEY_MAX_SUM 4
#define KEY_LONG_DOWN 60

#define LED_ALL_PIN (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)

typedef void (*KeyCallback)(void);

struct KeyInfo
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t status;
};


void LED_Set(uint8_t led);

void Status_Init(void);

void LCD_Show(void);

void UI_Show(void);

void Status_Update(void);

void PWM_Start(void);

void PWM_Refresh(void);

void Key_Scan(void);

void Key_Handle(void);

void B1_Short(void);

void B2_Short(void);

void B3_Short(void);

void B4_Short(void);

#endif
