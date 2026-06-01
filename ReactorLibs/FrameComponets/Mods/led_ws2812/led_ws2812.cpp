#include "led_ws2812.hpp"

TIM_HandleTypeDef* g_ws2812_callback_htim = nullptr;
uint32_t g_ws2812_callback_channel = 0;

/**
 * @brief 覆写PWM + DMA发送完成中断
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == g_ws2812_callback_htim)
    {
        HAL_TIM_PWM_Stop_DMA(g_ws2812_callback_htim, g_ws2812_callback_channel);
    }
}
