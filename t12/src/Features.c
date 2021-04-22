#include "Features.h"

uint8_t UINumb = UI_DATA;
char RecvBuf[RECV_BUF_MAX] = {0};
struct CarInfo CarList[CAR_LEN_MAX] = {0};

uint8_t CarData[3] = {0, 0, 8};
double FeeData[2] = {3.5, 2.0};

volatile uint8_t RecvFlag = RECV_WAIT;

uint32_t TimKeyScan = 0;
uint8_t KeyClick = 0x00;

uint8_t PWMOutputFlag = 0;

const char ui_str[2][20] = {
    "     Data     ",
    "     Para     "};

struct KeyInfo KeyList[KEY_SUM] = {
    {GPIOB, GPIO_PIN_0, KEY_UP},
    {GPIOB, GPIO_PIN_1, KEY_UP},
    {GPIOB, GPIO_PIN_2, KEY_UP},
    {GPIOA, GPIO_PIN_0, KEY_UP}};

Callback KeyHandles[KEY_SUM] = {
    B1_Short,
    B2_Short,
    B3_Short,
    B4_Short};

extern TIM_HandleTypeDef htim17;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

void App_Init(void)
{
    LCD_Init();
    LCD_Clear(Black);
    LCD_SetBackColor(Black);
    LCD_SetTextColor(White);

    _Led_Set(0x00);

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, (uint8_t *)RecvBuf, RECV_BUF_MAX);
    RecvFlag = RECV_WAIT;

    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
}

void App_Handle(void)
{
    Lcd_Task();
    Led_Task();
    Uart_Task();
    key_Task();
}

void Lcd_Task(void)
{
    char str[20] = {0};
    LCD_DisplayStringLine(Line1, (u8 *)ui_str[UINumb & 0x03]);

    if (UINumb == UI_DATA)
    {
        sprintf(str, "    CNBR:%d     ", CarData[DATA_CNBR]);
        LCD_DisplayStringLine(Line3, (u8 *)str);
        sprintf(str, "    VNBR:%d     ", CarData[DATA_VNBR]);
        LCD_DisplayStringLine(Line5, (u8 *)str);
        sprintf(str, "    IDLE:%d     ", CarData[DATA_IDLE]);
        LCD_DisplayStringLine(Line7, (u8 *)str);
    }
    else
    {
        sprintf(str, "    CNBR:%.2f     ", FeeData[FEE_CNBR]);
        LCD_DisplayStringLine(Line3, (u8 *)str);
        sprintf(str, "    VNBR:%.2f     ", FeeData[FEE_VNBR]);
        LCD_DisplayStringLine(Line5, (u8 *)str);
        LCD_ClearLine(Line7);
    }
}

void Led_Task(void)
{
    uint8_t led = ((CarData[DATA_IDLE] > 0) ? 0x01 : 0x00);
    led |= ((PWMOutputFlag == 1)? 0x02 : 0x00);
    _Led_Set(led);
}

void key_Task(void)
{
    KeyScan();
    KeyHandle();
}

void _Led_Set(uint8_t led)
{
    HAL_GPIO_WritePin(GPIOC, LED_ALL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, (uint16_t)led << 8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

// VNBR:A000:201012120000
void Uart_Task(void)
{
    if (RecvFlag == RECV_WAIT)
        return;

    RecvFlag = RECV_WAIT;

    char str[30] = {"Error"};
    do
    {
        if (strlen(RecvBuf) != CMD_LEN_MAX)
            // 数据长度不符合预期
            break;
        struct CarInfo Info = {0};
        if (RecvFormat(RecvBuf, &Info) == HANDLE_ERR)
            break;
        uint8_t id = CarInfoFind(Info.id);
        if (id == HANDLE_ERR)
        {
            if (CarInfoAdd(&Info) == HANDLE_ERR)
                break;
            str[0] = '\0';
            break;
        }
        // 车辆停车类型不符
        if (Info.mode != CarList[id].mode)
            break;
        // 车辆离开
        uint32_t dtime = 0;
        if (CalTime(&CarList[id].time, &Info.time, &dtime) == HANDLE_ERR)
            break;
        sprintf(str, "%.4s:%.4s:%lu:%.2f",
                ((Info.mode == MODE_CNBR) ? "CNBR" : "VNBR"),
                (char *)(&Info.id),
                dtime,
                dtime * FeeData[Info.mode - 1]);
        CarInfoDel(id);

    } while (0);

    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 20);
    memset((void *)RecvBuf, 0, RECV_BUF_MAX);
    HAL_UART_Receive_DMA(&huart1, (uint8_t *)RecvBuf, RECV_BUF_MAX);
}

uint8_t CalTime(const struct Time *t0, const struct Time *t1, uint32_t *ret)
{
    // 计算秒
    uint32_t tt0 = t0->SS + t0->mm * T_1m + t0->HH * T_1H + t0->DD * T_1D;
    uint32_t tt1 = t1->SS + t1->mm * T_1m + t1->HH * T_1H + t1->DD * T_1D;
    long dt = 0;

    // 计算月
    for (size_t i = 0; i < t0->MM; i++)
        tt0 += ((i % 2 == ((i < 8) ? 1 : 0)) ? 31 : 30) * T_1D;
    for (size_t i = 0; i < t1->MM; i++)
        tt1 += ((i % 2 == ((i < 8) ? 1 : 0)) ? 31 : 30) * T_1D;

    // 计算年
    for (size_t i = 0; i < t0->YY; i++)
        tt0 += (((t0->YY % 4) ? 355 : 356) * T_1D);
    for (size_t i = 0; i < t1->YY; i++)
        tt1 += (((t1->YY % 4) ? 355 : 356) * T_1D);

    dt = tt1 - tt0;
    if (dt < 0)
        return HANDLE_ERR;

    // 把秒换算成时
    *ret = (dt / T_1H) + ((dt % T_1H == 0) ? 0 : 1);

    return HANDLE_OK;
}

uint8_t TimeCheck(const struct Time *time)
{
    // 1 3 5 7 8 10 12 => 31
    // 2 4 6 9 11 => 30
    // 时间信息验证
    // 闰年
    if (time->MM == 2)
        if (time->DD > ((time->YY % 4) ? 28 : 29))
            return 1;

    if (time->MM > 12 || time->MM < 1)
        return 1;
    // 大小月
    if (time->DD > ((time->MM % 2 == ((time->MM < 8) ? 1 : 0)) ? 31 : 30) || time->DD < 1)
        return 1;

    if (time->HH > 23 || time->HH < 0)
        return 1;

    if (time->mm > 59 || time->mm < 0)
        return 1;

    if (time->SS > 59 || time->SS < 0)
        return 1;

    return 0;
}

uint8_t RecvFormat(const char *s, struct CarInfo *d)
{
    struct CarInfo info = {};
    char mode[5] = {}, id[5] = {};
    uint8_t ret = 0;

    ret = sscanf(s, "%4s:%4s:%02d%02d%02d%02d%02d%02d",
                 mode, id, &info.time.YY, &info.time.MM, &info.time.DD, &info.time.HH, &info.time.mm, &info.time.SS);
    // 数据格式验证
    if (ret != 8)
        return HANDLE_ERR;

    // 停车模式验证
    if (strcmp(mode, "CNBR") == 0)
        info.mode = MODE_CNBR;
    else if (strcmp(mode, "VNBR") == 0)
        info.mode = MODE_VNBR;
    else
        return HANDLE_ERR;

    // 时间格式验证
    if (TimeCheck(&info.time))
        return HANDLE_ERR;

    info.id = *((uint32_t *)id);
    *d = info;

    return HANDLE_OK;
}

uint8_t CarInfoFind(uint32_t id)
{
    for (size_t i = 0; i < CAR_LEN_MAX; i++)
    {
        if (id == CarList[i].id)
            return i; // 匹配成功
    }

    // Not Find
    return HANDLE_ERR;
}

uint8_t CarInfoAdd(const struct CarInfo *info)
{
    for (size_t i = 0; i < CAR_LEN_MAX; i++)
    {
        if (CarList[i].mode == MODE_NONE)
        {
            CarList[i] = *info;
            --CarData[DATA_IDLE];
            ++CarData[(info->mode == MODE_CNBR) ? DATA_CNBR : DATA_VNBR];

            return HANDLE_OK;
        }
    }

    return HANDLE_ERR;
}

uint8_t CarInfoDel(uint8_t id)
{
    ++CarData[DATA_IDLE];
    --CarData[(CarList[id & 0x07].mode == MODE_CNBR) ? DATA_CNBR : DATA_VNBR];
    memset((void *)(&CarList[id & 0x07]), 0, sizeof(struct CarInfo));
    return HANDLE_OK;
}

void KeyScan(void)
{
    if (HAL_GetTick() - TimKeyScan < KEY_SCAN_DELAY)
        return;
    TimKeyScan = HAL_GetTick();

    for (size_t i = 0; i < KEY_SUM; i++)
    {
        if (HAL_GPIO_ReadPin(KeyList[i].port, KeyList[i].pin) == GPIO_PIN_SET)
        {
            if (KeyList[i].status == KEY_DOWN)
                KeyClick = 0x01 << i;
            KeyList[i].status = KEY_UP;
        }
        else
        {
            KeyList[i].status = KEY_DOWN;
        }
    }
}

void KeyHandle(void)
{
    for (size_t i = 0; i < KEY_SUM; i++)
    {
        if (KeyClick == (0x01 << i))
            KeyHandles[i]();
    }
    KeyClick = 0x00;
}

void B1_Short(void)
{
    UINumb = (UINumb == UI_DATA) ? UI_PARA : UI_DATA;
}

void B2_Short(void)
{
    if(UINumb != UI_PARA)
        return;
    FeeData[FEE_CNBR] += 0.5;
    FeeData[FEE_VNBR] += 0.5;
}

void B3_Short(void)
{
    if(UINumb != UI_PARA)
        return;
    
    if (FeeData[FEE_VNBR] > 0)
    {
        FeeData[FEE_CNBR] -= 0.5;
        FeeData[FEE_VNBR] -= 0.5;
    }
}

void B4_Short(void)
{
    if (PWMOutputFlag == 0)
    {
        __HAL_TIM_SetCompare(&htim17, TIM_CHANNEL_1, 400);
        PWMOutputFlag = 1;
    }
    else
    {
        __HAL_TIM_SetCompare(&htim17, TIM_CHANNEL_1, 0);
        PWMOutputFlag = 0;
    }
}
