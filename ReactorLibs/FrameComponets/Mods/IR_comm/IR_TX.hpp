/**
 * @file IR_TX.hpp
 * @brief 红外发射模块 (NEC 协议)
 * @note PWM + DMA 完全非阻塞发送，类似 WS2812 驱动
 */

#pragma once

#include "bsp_tim_pwm.hpp"
#include "bsp_dwt.hpp"
#include <stdint.h>
#include "string.h"

/// @brief DMA 缓冲区大小 (足够容纳一帧 NEC 数据 @ 38kHz)
static constexpr uint16_t IRTX_DMA_BUF_SIZE = 8192;

/// @brief 发送完成回调 (timer, channel, user_ctx)
using IRTxDoneCallback = void (*)(BSP::TIM::TimID tim, BSP::TIM::Channel ch, void *user_ctx);

/**
 * @class IR_TX
 * @brief 红外发射模块 (NEC 协议)，PWM + DMA 非阻塞发送
 *
 * 使用方式:
 * @code
 * IR_TX ir_tx;
 * ir_tx.Init(TIM2, BSP::TIM::CH1);
 * ir_tx.Send(0x01, 0xA5);       // DMA 非阻塞，立即返回
 * // 等待完成轮询:
 * while (ir_tx.IsBusy());
 * // 或注册完成回调:
 * ir_tx.RegisterDoneCallback(MyCallback, nullptr);
 * @endcode
 */
class IR_TX
{
public:
    IR_TX() {};
    ~IR_TX() {};

    IR_TX(const IR_TX &) = delete;
    IR_TX &operator=(const IR_TX &) = delete;

    /**
     * @brief 初始化
     * @param tim     PWM 定时器（需外部预配置为 38kHz, 50% 占空比）
     * @param channel PWM 通道
     */
    void Init(BSP::TIM::TimID tim, BSP::TIM::Channel channel);

    /**
     * @brief 发送一帧 NEC 数据（非阻塞，DMA 方式）
     * @param address 8-bit 地址
     * @param command 8-bit 命令
     * @return true  启动成功
     * @return false 上一帧还在发送中
     */
    bool Send(uint8_t address, uint8_t command);

    /**
     * @brief 发送 NEC 重复码（非阻塞，DMA 方式）
     * @return true  启动成功
     * @return false 上一帧还在发送中
     */
    bool SendRepeat();

    /**
     * @brief DMA 是否正在发送中
     */
    bool IsBusy() const { return _dma_busy; }

    /**
     * @brief 注册发送完成回调
     */
    void RegisterDoneCallback(IRTxDoneCallback cb, void *user_ctx = nullptr);

    // ==================== 公共可配置参数 ====================

    BSP::TIM::TimID    tim_id  = nullptr;
    BSP::TIM::Channel  channel = BSP::TIM::CH1;

    // ---- NEC 协议时序参数 (单位: us) ----
    uint16_t leader_on_us  = 9000;
    uint16_t leader_off_us = 4500;
    uint16_t repeat_off_us = 2250;
    uint16_t bit_on_us     = 563;
    uint16_t bit_0_off_us  = 563;
    uint16_t bit_1_off_us  = 1687;

private:
    void _FillPulse(uint32_t time_us, bool high);
    void _BuildFrame(uint8_t address, uint8_t command);
    void _BuildRepeat();

    static void _UpdateDmaDoneCb(BSP::TIM::TimID tim, BSP::TIM::Channel ch, void *ctx);

    uint16_t _dma_buf[IRTX_DMA_BUF_SIZE];
    uint16_t _dma_len            = 0;
    bool     _dma_busy           = false;
    bool     _initialized        = false;

    uint32_t _ccr_high           = 0;      ///< 载波 ON 的 CCR (50% duty)
    uint32_t _ccr_low            = 0;      ///< 载波 OFF 的 CCR (0% duty)
    float    _pwm_period_us      = 0.0f;   ///< 单个 PWM 周期对应的微秒数

    struct
    {
        IRTxDoneCallback cb       = nullptr;
        void            *user_ctx = nullptr;
    } _done_handler;
};
