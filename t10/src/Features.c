
#include "Features.h"

inline void M4_LED_Set(uint8_t led_bit)
{
    // 熄灭所有LED
    M4_LED_Reset(0xff);
    GPIOC->BRR = (uint32_t)led_bit << 8;
    // 更新锁存器
    GPIOD->BSRR |= GPIO_PIN_2;
    __NOP();
    __NOP();
    __NOP();
    GPIOD->BRR |= GPIO_PIN_2;
    __NOP();
    __NOP();
    __NOP();
}

inline void M4_LED_Reset(uint8_t led_bit)
{
    GPIOC->BSRR = (uint16_t)led_bit << 8;
    // 更新锁存器
    GPIOD->BSRR |= GPIO_PIN_2;
    __NOP();
    __NOP();
    __NOP();
    GPIOD->BRR |= GPIO_PIN_2;
    __NOP();
    __NOP();
    __NOP();
    // 还原原始值
}

extern ADC_HandleTypeDef hadc2;

uint32_t M4_ADC_Start(void)
{
    uint16_t ADCValue = 0;
    HAL_ADC_Start(&hadc2);
    
    ADCValue = HAL_ADC_GetValue(&hadc2);

    return ADCValue;
}
