#include "bsp_tim_pwm.hpp"
#include "bsp_halport.hpp"

namespace
{
    struct CallbackSlot
    {
        BSP::TIM::TimID tim = nullptr;
        uint32_t channel = 0;
        BSP::TIM::PulseFinishedCallback callback = nullptr;
        void* user_ctx = nullptr;
    };

    constexpr size_t kMaxCallbackSlots = 16;
    CallbackSlot g_callback_slots[kMaxCallbackSlots];

    static inline TIM_HandleTypeDef* ToHal(BSP::TIM::TimID tim)
    {
        return reinterpret_cast<TIM_HandleTypeDef*>(tim);
    }

    static uint32_t ExtractActiveChannel(TIM_HandleTypeDef* htim)
    {
#ifdef USE_REAL_HAL
        switch (htim->Channel)
        {
            case HAL_TIM_ACTIVE_CHANNEL_1: return TIM_CHANNEL_1;
            case HAL_TIM_ACTIVE_CHANNEL_2: return TIM_CHANNEL_2;
            case HAL_TIM_ACTIVE_CHANNEL_3: return TIM_CHANNEL_3;
            case HAL_TIM_ACTIVE_CHANNEL_4: return TIM_CHANNEL_4;
            default: return 0;
        }
#else
        (void)htim;
        return 0;
#endif
    }

    static uint32_t ResolveTimerClockFreq(TIM_HandleTypeDef* htim)
    {
#ifdef USE_REAL_HAL
        uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
        uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();

        if (htim->Instance == TIM1 || htim->Instance == TIM8
#ifdef TIM9
            || htim->Instance == TIM9
#endif
#ifdef TIM10
            || htim->Instance == TIM10
#endif
#ifdef TIM11
            || htim->Instance == TIM11
#endif
        )
        {
            return pclk2 * 2;
        }
        return pclk1 * 2;
#else
        (void)htim;
        return 0;
#endif
    }

    static CallbackSlot* FindCallbackSlot(BSP::TIM::TimID tim, uint32_t channel)
    {
        for (size_t i = 0; i < kMaxCallbackSlots; ++i)
        {
            if (g_callback_slots[i].tim == tim && g_callback_slots[i].channel == channel)
            {
                return &g_callback_slots[i];
            }
        }
        return nullptr;
    }
}

namespace BSP
{
namespace TIM
{
    void StartPWM(TimID tim, uint32_t channel)
    {
        if (tim == nullptr || channel == 0) return;
#ifdef USE_REAL_HAL
        HAL_TIM_PWM_Start(ToHal(tim), channel);
#endif
    }

    void StopPWM(TimID tim, uint32_t channel)
    {
        if (tim == nullptr || channel == 0) return;
#ifdef USE_REAL_HAL
        HAL_TIM_PWM_Stop(ToHal(tim), channel);
#endif
    }

    void SetDuty(TimID tim, uint32_t channel, float duty)
    {
        if (tim == nullptr || channel == 0) return;

        if (duty < 0.0f) duty = 0.0f;
        if (duty > 1.0f) duty = 1.0f;

#ifdef USE_REAL_HAL
        TIM_HandleTypeDef* htim = ToHal(tim);
        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
        uint32_t ccr = (uint32_t)(arr * duty);
        __HAL_TIM_SET_COMPARE(htim, channel, ccr);
#endif
    }

    float GetFreq(TimID tim)
    {
        if (tim == nullptr) return 0.0f;

#ifdef USE_REAL_HAL
        TIM_HandleTypeDef* htim = ToHal(tim);
        uint32_t timer_clock = ResolveTimerClockFreq(htim);
        uint32_t psc = htim->Instance->PSC;
        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
        return (float)timer_clock / (float)(psc + 1u) / (float)(arr + 1u);
#else
        return 0.0f;
#endif
    }

    uint32_t GetAutoReload(TimID tim)
    {
        if (tim == nullptr) return 0;
#ifdef USE_REAL_HAL
        return __HAL_TIM_GET_AUTORELOAD(ToHal(tim));
#else
        return 0;
#endif
    }

    uint32_t GetCCR(TimID tim, uint32_t channel)
    {
        if (tim == nullptr || channel == 0) return 0;
#ifdef USE_REAL_HAL
        return __HAL_TIM_GET_COMPARE(ToHal(tim), channel);
#else
        return 0;
#endif
    }

    bool StartDma(TimID tim, uint32_t channel, const uint32_t* buf, size_t len)
    {
        if (tim == nullptr || channel == 0 || buf == nullptr || len == 0) return false;

#ifdef USE_REAL_HAL
        TIM_HandleTypeDef* htim = ToHal(tim);
        htim->Instance->CNT = htim->Instance->ARR - 1;
        return HAL_TIM_PWM_Start_DMA(htim, channel, (uint32_t*)buf, len) == HAL_OK;
#else
        return true;
#endif
    }

    void StopDma(TimID tim, uint32_t channel)
    {
        if (tim == nullptr || channel == 0) return;
#ifdef USE_REAL_HAL
        HAL_TIM_PWM_Stop_DMA(ToHal(tim), channel);
#endif
    }

    bool RegistPFCb(
        TimID tim,
        uint32_t channel,
        PulseFinishedCallback cb,
        void* user_ctx
    )
    {
        if (tim == nullptr || channel == 0) return false;

        CallbackSlot* existed = FindCallbackSlot(tim, channel);
        if (existed != nullptr)
        {
            existed->callback = cb;
            existed->user_ctx = user_ctx;
            return true;
        }

        for (size_t i = 0; i < kMaxCallbackSlots; ++i)
        {
            if (g_callback_slots[i].tim == nullptr)
            {
                g_callback_slots[i].tim = tim;
                g_callback_slots[i].channel = channel;
                g_callback_slots[i].callback = cb;
                g_callback_slots[i].user_ctx = user_ctx;
                return true;
            }
        }
        return false;
    }

    void UnRegistPFCb(TimID tim, uint32_t channel)
    {
        CallbackSlot* slot = FindCallbackSlot(tim, channel);
        if (slot == nullptr) return;

        slot->tim = nullptr;
        slot->channel = 0;
        slot->callback = nullptr;
        slot->user_ctx = nullptr;
    }
}
}

extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
    uint32_t channel = ExtractActiveChannel(htim);
    if (channel == 0) return;

    BSP::TIM::TimID tim = reinterpret_cast<BSP::TIM::TimID>(htim);
    CallbackSlot* slot = FindCallbackSlot(tim, channel);
    if (slot == nullptr || slot->callback == nullptr) return;

    slot->callback(tim, channel, slot->user_ctx);
}
