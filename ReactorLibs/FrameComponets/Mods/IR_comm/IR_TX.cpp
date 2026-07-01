/**
 * @file IR_TX.cpp
 * @brief 红外发射模块实现 (PWM + DMA 非阻塞)
 */

#include "IR_TX.hpp"
#include "bsp_log.hpp"

void IR_TX::Init(BSP::TIM::TimID tim, BSP::TIM::Channel ch)
{
    tim_id  = tim;
    channel = ch;

    if (tim_id == nullptr || channel == 0)
    {
        BspLog_LogError("[IR_TX] Init failed: null timer or channel");
        return;
    }

    // 获取 PWM 参数，用于计算 DMA 缓冲区
    uint32_t arr = BSP::TIM::GetAutoReload(tim_id);
    _ccr_high = (uint32_t)(arr * 0.5f);
    _ccr_low  = 0;

    float pwm_freq = BSP::TIM::GetFreq(tim_id);
    _pwm_period_us = (pwm_freq > 0.0f) ? (1000000.0f / pwm_freq) : 26.3f;

    // 注册 PWM DMA 完成回调
    BSP::TIM::RegistPFCb(tim_id, channel, _PulseFinishedCb, this);

    _initialized = true;
    BspLog_LogInfo("[IR_TX] Init OK, freq=%.0fHz, period=%.2fus",
                   (double)pwm_freq, (double)_pwm_period_us);
}

// ==================== DMA 缓冲区填充 ====================

void IR_TX::_FillPulse(uint32_t time_us, bool high)
{
    uint16_t cycles = (uint16_t)((float)time_us / _pwm_period_us + 0.5f);
    if (cycles == 0) cycles = 1;

    uint32_t ccr = high ? _ccr_high : _ccr_low;
    for (uint16_t i = 0; i < cycles; i++)
    {
        if (_dma_len >= IRTX_DMA_BUF_SIZE)
        {
            BspLog_LogError("[IR_TX] DMA buffer overflow!");
            return;
        }
        _dma_buf[_dma_len++] = ccr;
    }
}

void IR_TX::_BuildFrame(uint8_t address, uint8_t command)
{
    _dma_len = 0;

    // Leader
    _FillPulse(leader_on_us, true);
    _FillPulse(leader_off_us, false);

    // Address (LSB first) + ~Address
    for (uint8_t i = 0; i < 8; i++)
    {
        _FillPulse(bit_on_us, true);
        _FillPulse(((address >> i) & 0x01) ? bit_1_off_us : bit_0_off_us, false);
    }
    for (uint8_t i = 0; i < 8; i++)
    {
        _FillPulse(bit_on_us, true);
        _FillPulse(((~address >> i) & 0x01) ? bit_1_off_us : bit_0_off_us, false);
    }

    // Command (LSB first) + ~Command
    for (uint8_t i = 0; i < 8; i++)
    {
        _FillPulse(bit_on_us, true);
        _FillPulse(((command >> i) & 0x01) ? bit_1_off_us : bit_0_off_us, false);
    }
    for (uint8_t i = 0; i < 8; i++)
    {
        _FillPulse(bit_on_us, true);
        _FillPulse(((~command >> i) & 0x01) ? bit_1_off_us : bit_0_off_us, false);
    }

    // Stop bit
    _FillPulse(bit_on_us, true);
}

void IR_TX::_BuildRepeat()
{
    _dma_len = 0;
    _FillPulse(leader_on_us, true);
    _FillPulse(repeat_off_us, false);
    _FillPulse(bit_on_us, true);
}

// ==================== 发送 ====================

bool IR_TX::Send(uint8_t address, uint8_t command)
{
    if (!_initialized)
    {
        BspLog_LogError("[IR_TX] Send failed: not initialized");
        return false;
    }
    if (_dma_busy)
    {
        BspLog_LogWarning("[IR_TX] Send skipped: DMA busy");
        return false;
    }

    _BuildFrame(address, command);
    _dma_busy = true;
    BSP::TIM::StartDma(tim_id, channel, _dma_buf, _dma_len);
    return true;
}

bool IR_TX::SendRepeat()
{
    if (!_initialized) return false;
    if (_dma_busy) return false;

    _BuildRepeat();
    _dma_busy = true;
    BSP::TIM::StartDma(tim_id, channel, _dma_buf, _dma_len);
    return true;
}

void IR_TX::RegisterDoneCallback(IRTxDoneCallback cb, void *user_ctx)
{
    _done_handler.cb       = cb;
    _done_handler.user_ctx = user_ctx;
}

// ==================== DMA 完成回调 ====================

void IR_TX::_PulseFinishedCb(BSP::TIM::TimID tim, BSP::TIM::Channel ch, void *ctx)
{
    BSP::TIM::StopDma(tim, ch);
    BSP::TIM::StopPWM(tim, ch);

    IR_TX *self = static_cast<IR_TX *>(ctx);
    if (self)
    {
        self->_dma_busy = false;
        if (self->_done_handler.cb)
            self->_done_handler.cb(tim, ch, self->_done_handler.user_ctx);
    }
}
