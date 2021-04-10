#ifndef FEATURES_H
#define FEATURES_H

#include "stm32g4xx_hal.h"
#include "lcd.h"
#include <stdio.h>

#define UI_MAIN       0x00 
#define UI_SETTING    0x01

#define SETTING_ITEM_SUM 0x04

#define STATUS_NORMAL 0x00 
#define STATUS_UPPER  0x01
#define STATUS_LOWER  0x02

#define KEY_DOWN    0xff
#define KEY_UP      0x00
#define KEY_MAX_SUM 0x04

#define MAX_VOLT 0x00
#define MIN_VOLT 0x01
#define UPPER_LD 0x02
#define LOWER_LD 0x03

#define LED_REFRESH_DELAY 190

#define ALL_LED_PIN (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)

typedef void (*Callback)(void);

struct KeyInfo
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t status;
};

void LCD_Show(void);

void LED_Refresh();

void UI_Init(void);

void UI_Show(void);

void Status_Update(void);

void Key_Scan(void);

void Key_Handle(void);

void B1_Short(void);
void B2_Short(void);
void B3_Short(void);
void B4_Short(void);

#endif