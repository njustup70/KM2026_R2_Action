#include "bsp_tim_pwm.hpp"
#include "bsp_halport.hpp"

namespace
{
    struct CallbackSlot
    {
        BSP::TIM::TimID tim = nullptr;
        BSP::TIM::Channel channel = BSP::TIM::CH1;
        BSP::TIM::PulseFinishedCallback callback = nullptr;
        void* user_ctx = nullptr;
    };

    constexpr size_t kMaxCallbackSlots = 16;
    CallbackSlot g_callback_slots[kMaxCallbackSlots];

    static inline TIM_HandleTypeDef* ToHal(BSP::TIM::TimID tim)
    {
        return reinterpret_cast<TIM_HandleTypeDef*>(tim);
    }

    static BSP::TIM::Channel ExtractActiveChannel(TIM_HandleTypeDef* htim)
    {
#ifdef USE_REAL_HAL
        switch (htim->Channel)
        {
            case HAL_TIM_ACTIVE_CHANNEL_1: return BSP::TIM::CH1;
            case HAL_TIM_ACTIVE_CHANNEL_2: return BSP::TIM::CH2;
            case HAL_TIM_ACTIVE_CHANNEL_3: return BSP::TIM::CH3;
            case HAL_TIM_ACTIVE_CHANNEL_4: return BSP::TIM::CH4;
            default: return static_cast<BSP::TIM::Channel>(0);
        }
#else
        (void)htim;
        return static_cast<BSP::TIM::Channel>(0);
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

    static CallbackSlot* FindCallbackSlot(BSP::TIM::TimID tim, BSP::TIM::Channel channel)
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
    void StartPWM(TimID tim, Channel channel)
    {
        if (tim == nullptr || channel == 0) return;
#ifdef USE_REAL_HAL
        HAL_TIM_PWM_Start(ToHal(tim), channel);
#endif
    }

    void StopPWM(TimID tim, Channel channel)
    {
        if (tim == nullptr || channel == 0) return;
#ifdef USE_REAL_HAL
        HAL_TIM_PWM_Stop(ToHal(tim), channel);
#endif
    }

    void SetDuty(TimID tim, Channel channel, float duty)
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

    uint32_t GetCCR(TimID tim, Channel channel)
    {
        if (tim == nullptr || channel == 0) return 0;
#ifdef USE_REAL_HAL
        return __HAL_TIM_GET_COMPARE(ToHal(tim), channel);
#else
        return 0;
#endif
    }

    bool StartDma(TimID tim, Channel channel, const uint32_t* buf, size_t len)
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

    void StopDma(TimID tim, Channel channel)
    {
        if (tim == nullptr || channel == 0) return;
#ifdef USE_REAL_HAL
        HAL_TIM_PWM_Stop_DMA(ToHal(tim), channel);
#endif
    }

    bool RegistPFCb(
        TimID tim,
        Channel channel,
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

    void UnRegistPFCb(TimID tim, Channel channel)
    {
        CallbackSlot* slot = FindCallbackSlot(tim, channel);
        if (slot == nullptr) return;

        slot->tim = nullptr;
        slot->channel = BSP::TIM::CH1;
        slot->callback = nullptr;
        slot->user_ctx = nullptr;
    }

    // ==================== Update DMA 回调槽 ====================
    struct UpdateDmaCallbackSlot
    {
        BSP::TIM::TimID tim = nullptr;
        BSP::TIM::Channel channel = BSP::TIM::CH1;
        BSP::TIM::PulseFinishedCallback callback = nullptr;
        void* user_ctx = nullptr;
    };

    constexpr size_t kMaxUpdateDmaSlots = 8;
    UpdateDmaCallbackSlot g_update_dma_slots[kMaxUpdateDmaSlots];

    static UpdateDmaCallbackSlot* FindUpdateDmaSlot(BSP::TIM::TimID tim, BSP::TIM::Channel channel)
    {
        for (size_t i = 0; i < kMaxUpdateDmaSlots; ++i)
        {
            if (g_update_dma_slots[i].tim == tim &&
                g_update_dma_slots[i].channel == channel)
            {
                return &g_update_dma_slots[i];
            }
        }
        return nullptr;
    }

    /// @brief DMA 完成中断路由器：遍历槽找到对应的 TIM，调用注册的回调
    static void _UpdateDmaCbRouter(DMA_HandleTypeDef* hdma)
    {
        for (size_t i = 0; i < kMaxUpdateDmaSlots; ++i)
        {
            if (g_update_dma_slots[i].tim == nullptr) continue;
            TIM_HandleTypeDef* htim = reinterpret_cast<TIM_HandleTypeDef*>(g_update_dma_slots[i].tim);
            if (htim->hdma[TIM_DMA_ID_UPDATE] == hdma)
            {
                if (g_update_dma_slots[i].callback)
                    g_update_dma_slots[i].callback(
                        g_update_dma_slots[i].tim,
                        g_update_dma_slots[i].channel,
                        g_update_dma_slots[i].user_ctx);
                return;
            }
        }
    }


bool StartUpdateDma(TimID tim, Channel channel, const uint16_t* buf, size_t len)
{
    if (tim == nullptr || buf == nullptr || len == 0) return false;

#ifdef USE_REAL_HAL
    TIM_HandleTypeDef* htim = ToHal(tim);
    DMA_HandleTypeDef* hdma = htim->hdma[TIM_DMA_ID_UPDATE];
    if (hdma == nullptr) return false;

    // 1. 根据通道解析 CCR 寄存器地址
    volatile uint32_t* ccr = nullptr;
    switch (channel)
    {
        case TIM_CHANNEL_1: ccr = &htim->Instance->CCR1; break;
        case TIM_CHANNEL_2: ccr = &htim->Instance->CCR2; break;
        case TIM_CHANNEL_3: ccr = &htim->Instance->CCR3; break;
        case TIM_CHANNEL_4: ccr = &htim->Instance->CCR4; break;
        default: return false;
    }

    // 2. DMA 先于定时器启动，确保首次 Update 立即触发有效 DMA 写
    hdma->XferErrorCallback = _UpdateDmaCbRouter;
    hdma->State = HAL_DMA_STATE_READY;
    HAL_DMA_Start(hdma, (uint32_t)buf, (uint32_t)ccr, len);
    htim->Instance->DIER |= TIM_DIER_UDE;

    // 3. 启动 PWM（首轮 Update 即由 DMA 写入正确 CCR）
    HAL_TIM_PWM_Start(htim, channel);
    if (htim->Instance == TIM1 || htim->Instance == TIM8)
        htim->Instance->BDTR |= TIM_BDTR_MOE;

    return true;
#else
    (void)tim; (void)channel; (void)buf; (void)len;
    return true;
#endif
}

void StopUpdateDma(TimID tim, Channel channel)
{
    if (tim == nullptr) return;

#ifdef USE_REAL_HAL
    TIM_HandleTypeDef* htim = ToHal(tim);
    DMA_HandleTypeDef* hdma = htim->hdma[TIM_DMA_ID_UPDATE];

    // 1. 关闭 Update DMA 请求
    htim->Instance->DIER &= ~TIM_DIER_UDE;

    // 2. 停止 DMA
    if (hdma)
    {
        HAL_DMA_Abort(hdma);
        hdma->XferCpltCallback = nullptr;
    }

    // 3. 停止 PWM 输出
    HAL_TIM_PWM_Stop(htim, channel);
#endif
}

bool RegistUpdateDmaCb(TimID tim, Channel channel,
                       PulseFinishedCallback cb, void* user_ctx)
{
    if (tim == nullptr || channel == 0) return false;

    UpdateDmaCallbackSlot* existed = FindUpdateDmaSlot(tim, channel);
    if (existed != nullptr)
    {
        existed->callback = cb;
        existed->user_ctx = user_ctx;
        return true;
    }

    for (size_t i = 0; i < kMaxUpdateDmaSlots; ++i)
    {
        if (g_update_dma_slots[i].tim == nullptr)
        {
            g_update_dma_slots[i].tim = tim;
            g_update_dma_slots[i].channel = channel;
            g_update_dma_slots[i].callback = cb;
            g_update_dma_slots[i].user_ctx = user_ctx;
            return true;
        }
    }
    return false;
}

void UnRegistUpdateDmaCb(TimID tim, Channel channel)
{
    UpdateDmaCallbackSlot* slot = FindUpdateDmaSlot(tim, channel);
    if (slot == nullptr) return;

    slot->tim = nullptr;
    slot->channel = BSP::TIM::CH1;
    slot->callback = nullptr;
    slot->user_ctx = nullptr;
}

} // namespace TIM
} // namespace BSP

extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
    BSP::TIM::Channel channel = ExtractActiveChannel(htim);
    if (channel == 0) return;

    BSP::TIM::TimID tim = reinterpret_cast<BSP::TIM::TimID>(htim);
    CallbackSlot* slot = FindCallbackSlot(tim, channel);
    if (slot == nullptr || slot->callback == nullptr) return;

    slot->callback(tim, channel, slot->user_ctx);
}
