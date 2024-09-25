#pragma once

#include "stm32f4xx_hal.h"

/*
Motors will be 80kHZ

TIM_5 APB1 TIM CLOCK = 84MHz
TIM_CLOCK = (APB TIM CLOCK) / PRESCALER
Frequency = TIM_CLOCK / ARR
Duty Cycle = (CCR / ARR) * 100

25.2V => 100% Duty Cycle
0V => 0% Duty Cycle
25.2V / 1000 = 0.025V per 0.1% Duty Cycle is precise
Lowest 
ARR = 1000
Frequency => 42kHz = TIM_CLOCK / 1000,
TIM_CLOCK = 42kHz * 1000 = 42 000 000
TIM_CLOCK => 42 000 000 = 84 000 000 / PRESCALER
PRESCALER = 2

*/


class MotorPWMManager
{
public:
    MotorPWMManager(TIM_HandleTypeDef *htim, uint32_t Channel);
    virtual ~MotorPWMManager();

    void _init();

private:
    TIM_HandleTypeDef *_pTIM;
    uint32_t _channel;
    volatile uint32_t* _ccr;
    uint32_t _duty;

    uint32_t TIM_CLOCK;
    uint32_t PRESCALER;
    uint32_t ARR;
    uint32_t CCR;
    uint32_t Frequency;
    uint32_t DutyCycle;
};