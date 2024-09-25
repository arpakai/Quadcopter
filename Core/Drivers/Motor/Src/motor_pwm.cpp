#include "motor_pwm.h"

MotorPWMManager::MotorPWMManager(TIM_HandleTypeDef *htim, uint32_t Channel) :_pTIM{htim}, _channel{Channel},
                                                                            _ccr{nullptr}, _duty{0}
{
    _ccr =  (_channel == TIM_CHANNEL_1) ? &htim->Instance->CCR1 :
            (_channel == TIM_CHANNEL_2) ? &htim->Instance->CCR2 : 
            (_channel == TIM_CHANNEL_3) ? &htim->Instance->CCR3 : 
            (_channel == TIM_CHANNEL_4) ? &htim->Instance->CCR4 : 
            &htim->Instance->CCR1;

    *_ccr = 500;   // 50% Duty Cycle
}

MotorPWMManager::~MotorPWMManager()
{

}

void MotorPWMManager::_init()
{
    //HAL_TIM_PWM_Start(_pTIM, _channel);
    _duty = 500;
    HAL_TIM_PWM_Start_DMA(_pTIM, _channel, &_duty, 1);
}

// _ccr ranges from 0 to 1000