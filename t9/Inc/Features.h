#ifndef FEATURES_H
#define FEATURES_H

#include "stm32g431xx.h"
#include "lcd.h"
#include "i2c.h"
#include <stdio.h>

#define TIME_SIZE 3
#define TIME_H 0
#define TIME_M 1
#define TIME_S 2

#define STATUS_RUNNING 0x00
#define STATUS_STANDBY 0x01
#define STATUS_PAUSE 0x02
#define STATUS_SETTING 0x03

#define STATUS_SUM_MAX 4

#define TIME_LIST_MAX 5

#define KEY_LONG_DOWN_MAX 40
#define KEY_SUM_MAX 4

#define KEY_DOWN 0x00
#define KEY_UP 0xff

#define LED_FLASH_INTERVAL 500
#define LED_ALL_PIN (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)

struct KeyInfo
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t status;
    uint8_t count;
};

typedef void (*Callback)(void);

void LED_Set(uint8_t led);

void LED_Flash(void);

void LCD_Show(void);

void LCD_UInit(void);

void Tim_Update(void);

uint8_t M24c02_Read(uint8_t addr);

void M24c02_Write(uint8_t addr, uint8_t val);

void Get_TimeList(uint8_t item);

void Set_TImeList(uint8_t item);

void PWM_Output(void);

void Key_Scan(void);

void Key_Handle(void);

void B1_short(void);
void B2_short(void);
void B3_short(void);
void B4_short(void);
void B1_long(void);
void B2_long(void);
void B3_long(void);
void B4_long(void);

#endif
