#ifndef BSP_TIM_PWM_H
#define BSP_TIM_PWM_H

#include <stddef.h>
#include <stdint.h>

namespace BSP
{
namespace TIM
{
    /// @brief 不透明指针，等价于框架内部的 TIM 句柄
    struct OpaqueTim;
    using TimID = OpaqueTim*;

    /// @brief TIM 通道枚举，值与 STM32 HAL TIM_CHANNEL_x 对齐
    enum Channel : uint32_t
    {
        CH1 = 0x00000000U,  // TIM_CHANNEL_1
        CH2 = 0x00000004U,  // TIM_CHANNEL_2
        CH3 = 0x00000008U,  // TIM_CHANNEL_3
        CH4 = 0x0000000CU,  // TIM_CHANNEL_4
    };
    
    /**
     * @brief PWM DMA发送完成回调 
     * @param user_ctx 用户上下文
     */
    using PulseFinishedCallback = void(*)(TimID tim, Channel channel, void* user_ctx);

    /// @brief 使能某个 TIM 通道的 PWM 输出
    void StartPWM(TimID tim, Channel channel);

    /// @brief 关闭某个 TIM 通道的 PWM 输出
    void StopPWM(TimID tim, Channel channel);

    /// @brief 设置某个 TIM 通道的占空比，范围会被钳制到 0.0~1.0
    void SetDuty(TimID tim, Channel channel, float duty);
    
    /// @brief 获取该定时器当前 PWM 基频
    float GetFreq(TimID tim);

    /// @brief 获取该定时器的自动重装载值 ARR
    uint32_t GetAutoReload(TimID tim);

    /// @brief 获取某个 TIM 通道当前比较寄存器值 CCR
    uint32_t GetCCR(TimID tim, Channel channel);

    /**
     * @brief 启动某个 TIM 通道的 PWM DMA 发送
     * @return 是否成功启动
     */
    bool StartDma(TimID tim, Channel channel, const uint32_t* buf, size_t len);

    /// @brief 停止某个 TIM 通道的 PWM DMA 发送
    void StopDma(TimID tim, Channel channel);
    
    /**
     * @brief 注册某个 TIM 通道的 PWM DMA完成回调
     * @note 全称 Pulse Finished Callback
     * @return 是否注册成功 
     */
    bool RegistPFCb(
        TimID tim,
        Channel channel,
        PulseFinishedCallback cb,
        void* user_ctx
    );

    /// @brief 注销某个 TIM 通道的 PWM DMA完成回调
    void UnRegistPFCb(TimID tim, Channel channel);

    // ==================== 路线 A: Update DMA ====================

    /// @brief 使用 TIM Update 事件触发 DMA 写 CCR (适合 IR_TX 等场景)
    /// @param buf  16-bit 半字缓冲区，内容即每次更新时写入 CCR 的值
    /// @param len  半字数量
    bool StartUpdateDma(TimID tim, Channel channel, const uint16_t* buf, size_t len);

    /// @brief 停止 Update DMA
    void StopUpdateDma(TimID tim, Channel channel);

    /// @brief 注册 Update DMA 完成回调 (与 RegistPFCb 独立)
    bool RegistUpdateDmaCb(TimID tim, Channel channel,
                           PulseFinishedCallback cb, void* user_ctx);

    /// @brief 注销 Update DMA 完成回调
    void UnRegistUpdateDmaCb(TimID tim, Channel channel);
}
}

#endif
