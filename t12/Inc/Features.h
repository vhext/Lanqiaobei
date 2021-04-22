#pragma once

#include "stm32g4xx_hal.h"
#include "lcd.h"
#include "stdio.h"
#include "string.h"

#define UI_DATA 0x00
#define UI_PARA 0x01
#define LED_ALL_PIN (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)

#define DATA_CNBR 0x00
#define DATA_VNBR 0x01
#define DATA_IDLE 0x02

#define FEE_CNBR 0x00
#define FEE_VNBR 0x01

#define RECV_WAIT 0x00
#define RECV_FINISH 0x01

#define RECV_BUF_MAX 32
#define CMD_LEN_MAX 22

#define MODE_NONE 0x00
#define MODE_CNBR 0x01
#define MODE_VNBR 0x02

#define CAR_LEN_MAX 8

#define HANDLE_ERR 0xff
#define HANDLE_OK 0x00

#define T_1m (60)
#define T_1H (60 * 60)
#define T_1D (24 * 60 * 60)

#define KEY_SUM 4
#define KEY_SCAN_DELAY 10
#define KEY_UP 0x00
#define KEY_DOWN 0x01

struct Time
{
    int YY;
    int MM;
    int DD;
    int HH;
    int mm;
    int SS;
};

struct CarInfo
{
    uint8_t mode;
    uint32_t id;
    struct Time time;
};

struct KeyInfo
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t status;
};

typedef void (*Callback)(void);

void App_Init(void);

void App_Handle(void);

void Lcd_Task(void);

void Led_Task(void);

void Uart_Task(void);

void key_Task(void);

void _Led_Set(uint8_t led);

uint8_t CalTime(const struct Time *t0, const struct Time *t1, uint32_t *ret);

uint8_t TimeCheck(const struct Time *time);

uint8_t RecvFormat(const char *s, struct CarInfo *d);

uint8_t CarInfoFind(uint32_t id);

uint8_t CarInfoAdd(const struct CarInfo *info);

uint8_t CarInfoDel(uint8_t id);

void KeyScan(void);

void KeyHandle(void);

void B1_Short(void);
void B2_Short(void);
void B3_Short(void);
void B4_Short(void);

