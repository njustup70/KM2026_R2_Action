/**
 * @author Huangney
 * @note 相较于先前的C版本，本库改为使用PWM驱动（这样不受MCU主频限制）
 */
#ifndef _LED_WS2812_HPP_
#define _LED_WS2812_HPP_

#include <cstddef>
#include "std_math.hpp"
#include "bsp_tim_pwm.hpp"
#include "arm_math.h"
#include "bsp_dwt.hpp"
#include "bsp_log.hpp"

/**
 * @brief WS2812 灯带驱动（PWM + DMA）
 * @tparam MaxLedNumsT 最大灯珠数，静态缓冲区
 */
template <size_t MaxLedNumsT = 16>
class LedWs2812
{
private:
    static constexpr size_t kBitsPerLed = 24;
    static constexpr size_t kResetSlots = 64; // >50us reset low at 800kHz

    enum class EffectMode : uint8_t
    {
        Static = 0,
        Breath,
        GradientFlow,
        Running,
        Expand,
        Blink,
        ProgBar,
    };

    uint32_t PwmMaxValue = 0;       // PWM最大值
    uint32_t HIGH_WS2812 = 0;       // PWM高电平数值
    uint32_t LOW_WS2812 = 0;        // PWM低电平数值

    // 这俩变量是给灯带动画用的，就是如果要渐变效果的话，循环调用就用这个内部计时器
    uint32_t dwt_tick = 0;
    float RuntimeCnt = 0;
    bool initialized_ = false;
    bool dirty_ = false;
    uint8_t handle_prescaler_ = 6;
    uint8_t handle_div_cnt_ = 0;

    EffectMode current_mode_ = EffectMode::Static;
    Color effect_color_0_;
    Color effect_color_1_;
    float effect_period_ = 1.0f;
    float effect_width_ = 0.2f;
    uint8_t effect_blink_times_ = 1;
    bool blink_state_on_ = false;
    Color prog_bar_color_;
    float* prog_bar_value_ = nullptr;
    float prog_bar_last_value_ = -1.0f;

    // 灯带的写入缓冲区，每颗灯24bit，颜色顺序是GRB，末尾补一段低电平用于锁存
    uint32_t WS2812buf2send[MaxLedNumsT * kBitsPerLed + kResetSlots] = {0};

public:
    LedWs2812() {};
    ~LedWs2812() {};

    BSP::TIM::TimID tim_id = nullptr;
    BSP::TIM::Channel Channel = BSP::TIM::CH1;

    uint8_t LedNums = 0;                // LED灯珠数量
    Vec3 BiasFactor;                    // 颜色偏置因子（用于校正颜色）

    /**
     * @brief 设置单颗灯颜色（内部自动编码为24bit PWM波形）
     * @note Led_id 会按 LedNums 做环绕映射，允许越界索引输入
     */
    void SetColor(int8_t Led_id, uint8_t R, uint8_t G, uint8_t B);

    /**
     * @brief 把颜色从缓冲区上传到灯带
     */
    void Upload();

    /**
     * @brief 在外部循环中持续调用，自动维护当前效果并按需上传
     */
    void Handle();

    /**
     * @brief 初始化灯带驱动
     * @param tim WS2812对应的定时器ID
     * @param Channel PWM通道
     * @note 实际灯珠数固定为模板参数 MaxLedNumsT
     */
    void Init(BSP::TIM::TimID tim, BSP::TIM::Channel Channel);

    /**
     * @brief 设置 Handle 的分频系数，默认 6
     * @note 传入 0 时会按 1 处理
     */
    void SetHandlePrescaler(uint8_t prescaler);

    private:
    /**--- 预设动画的内部持续调用，维护灯带颜色 ---**/
    void _ApplyBreathFrame();
    void _ApplyGradientFlowFrame();
    void _ApplyRunningFrame();
    void _ApplyExpandFrame();
    void _ApplyBlinkFrame();
    void _ApplyProgBarFrame();
    void _ResetEffectRuntime();
    void _ActivateStaticMode();
    void _SetColorRaw(int8_t Led_id, uint8_t R, uint8_t G, uint8_t B);
    void _FillFrame(uint8_t R, uint8_t G, uint8_t B);
    float _NormalizePeriod(float period) const;
    bool _IsSameColor(const Color& lhs, const Color& rhs) const;
    float _Clamp01(float value) const;
    float _SmoothStep(float edge_0, float edge_1, float value) const;

    public:
    /*---   一些预设的动画  ---*/
    void Fill(uint8_t R, uint8_t G, uint8_t B);
    void Breath(Color color_0, float period);
    void GradientFlow(Color color_0, Color color_1, float period);
    void Running(Color color, float width, float period);
    void Blink(Color color, uint8_t times, float period);
    void Lit(Color color); // 供外部调用的Fill，省算力
    void Expand(Color color_0, float period);
    void ProgBar(Color color, float* progress);

    static void _PulseFinishedCb(BSP::TIM::TimID tim, BSP::TIM::Channel channel, void* user_ctx);
};

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::Init(BSP::TIM::TimID tim, BSP::TIM::Channel Channel)
{
    // 检查输入参数合法性
    if (tim == nullptr || Channel == 0)
    {
        BspLog_LogError("[WS2812]: Init at null tim id or channel!\r\n");
        return;
    }

    // 存储基本信息
    this->tim_id = tim;
    this->Channel = Channel;
    this->LedNums = static_cast<uint8_t>(MaxLedNumsT);
    BiasFactor = Vec3(1.0f, 1.0f, 1.0f);

    // 初始化PWM
    PwmMaxValue = BSP::TIM::GetAutoReload(this->tim_id);
    HIGH_WS2812 = PwmMaxValue * 0.7f;
    LOW_WS2812 = PwmMaxValue * 0.3f;

    // 灯带颜色默认全灭
    for (size_t i = 0; i < MaxLedNumsT; i++)
    {
        _SetColorRaw((int8_t)i, 0, 0, 0);
    }

    // 注册 PWM 完成回调
    BSP::TIM::RegistPFCb(this->tim_id, this->Channel, _PulseFinishedCb, this);
    initialized_ = true;
    dirty_ = true;
    handle_div_cnt_ = 0;
    _ActivateStaticMode();
    _ResetEffectRuntime();
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::SetColor(int8_t Led_id, uint8_t R, uint8_t G, uint8_t B)
{
    _ActivateStaticMode();
    _SetColorRaw(Led_id, R, G, B);
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::_SetColorRaw(int8_t Led_id, uint8_t R, uint8_t G, uint8_t B)
{
    if (LedNums == 0) return;

    while (Led_id > (LedNums - 1)) Led_id -= LedNums;
    while (Led_id < 0) Led_id += LedNums;

    size_t base = static_cast<size_t>(Led_id) * kBitsPerLed;

    for (int i = 0; i < 8; i++) WS2812buf2send[base + i] = (G & (1 << (7 - i)) ? (HIGH_WS2812) : LOW_WS2812);
    for (int i = 8; i < 16; i++) WS2812buf2send[base + i] = (R & (1 << (15 - i)) ? (HIGH_WS2812) : LOW_WS2812);
    for (int i = 16; i < 24; i++) WS2812buf2send[base + i] = (B & (1 << (23 - i)) ? (HIGH_WS2812) : LOW_WS2812);
    dirty_ = true;
}

template <size_t MaxLedNumsT>
float LedWs2812<MaxLedNumsT>::_NormalizePeriod(float period) const
{
    return (period > 0.0f) ? period : 0.001f;
}

template <size_t MaxLedNumsT>
float LedWs2812<MaxLedNumsT>::_Clamp01(float value) const
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

template <size_t MaxLedNumsT>
float LedWs2812<MaxLedNumsT>::_SmoothStep(float edge_0, float edge_1, float value) const
{
    if (edge_0 == edge_1) return value < edge_0 ? 0.0f : 1.0f;

    float t = (value - edge_0) / (edge_1 - edge_0);
    t = _Clamp01(t);
    return t * t * (3.0f - 2.0f * t);
}

template <size_t MaxLedNumsT>
bool LedWs2812<MaxLedNumsT>::_IsSameColor(const Color& lhs, const Color& rhs) const
{
    return lhs.r == rhs.r && lhs.g == rhs.g && lhs.b == rhs.b;
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::_ResetEffectRuntime()
{
    RuntimeCnt = 0.0f;
    dwt_tick = DWT->CYCCNT;
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::_ActivateStaticMode()
{
    current_mode_ = EffectMode::Static;
    blink_state_on_ = false;
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::_FillFrame(uint8_t R, uint8_t G, uint8_t B)
{
    for (int i = 0; i < LedNums; i++)
    {
        _SetColorRaw(i, R, G, B);
    }
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::_ApplyGradientFlowFrame()
{
    if (LedNums == 0) return;

    float delta_t = DWT_GetDeltaTime(&dwt_tick);
    RuntimeCnt += delta_t;
    if (RuntimeCnt >= effect_period_) {
        RuntimeCnt = fmod(RuntimeCnt, effect_period_);
    }

    Color color_0 = effect_color_0_ * BiasFactor;
    Color color_1 = effect_color_1_ * BiasFactor;
    float phaseStep = effect_period_ / LedNums;

    for (int i = 0; i < LedNums; i++)
    {
        float phase = fmod(RuntimeCnt + (i * phaseStep), effect_period_);
        float ratio = 0.5f * (1.0f + sinf(2 * 3.1415f * phase / effect_period_ - 1.5708f));
        Color color_targ = (color_0 + (color_1 - color_0) * ratio);
        _SetColorRaw(i, color_targ.r, color_targ.g, color_targ.b);
    }
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::_ApplyBreathFrame()
{
    if (LedNums == 0) return;

    float delta_t = DWT_GetDeltaTime(&dwt_tick);
    RuntimeCnt += delta_t;
    if (RuntimeCnt >= effect_period_) {
        RuntimeCnt = fmod(RuntimeCnt, effect_period_);
    }

    Color color_0 = effect_color_0_ * BiasFactor;
    float ratio = 0.5f * (1.0f + sinf((2 * PI) * RuntimeCnt / effect_period_ - (PI / 2)));
    Color color_targ = color_0 * ratio;
    _FillFrame(color_targ.r, color_targ.g, color_targ.b);
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::_ApplyRunningFrame()
{
    if (LedNums == 0) return;

    float delta_t = DWT_GetDeltaTime(&dwt_tick);
    RuntimeCnt += delta_t;
    if (RuntimeCnt >= effect_period_) RuntimeCnt = fmod(RuntimeCnt, effect_period_);

    Color color = effect_color_0_ * BiasFactor;
    float position = (RuntimeCnt / effect_period_) * LedNums;
    float width_in_leds = effect_width_ * LedNums;
    float half_width = width_in_leds * 0.5f;
    float feather = 0.75f;
    float inner_half = half_width - feather;
    if (inner_half < 0.0f) inner_half = 0.0f;
    float outer_half = half_width + feather;

    for (int i = 0; i < LedNums; i++)
    {
        float distance = fabsf(i - position);
        distance = fminf(distance, LedNums - distance);

        float intensity = 0.0f;
        if (distance <= inner_half)
        {
            intensity = 1.0f;
        }
        else if (distance < outer_half)
        {
            intensity = 1.0f - _SmoothStep(inner_half, outer_half, distance);
        }

        Color color_targ = color * intensity;
        _SetColorRaw(i, color_targ.r, color_targ.g, color_targ.b);
    }
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::_ApplyExpandFrame()
{
    if (LedNums == 0) return;

    float delta_t = DWT_GetDeltaTime(&dwt_tick);
    RuntimeCnt += delta_t;
    if (RuntimeCnt > effect_period_) RuntimeCnt = effect_period_;

    Color color_0 = effect_color_0_ * BiasFactor;
    float ratio = RuntimeCnt / effect_period_;
    float center = LedNums / 2.0f;

    for (int i = 0; i < LedNums; i++)
    {
        float litness = 0;
        if (i < center) litness = (i / center) + 2 * ratio - 1;
        else litness = 2 * ratio - ((float)i - center) / center;

        if (litness < 0) litness = 0;
        if (litness > 1) litness = 1;
        Color color_targ = color_0 * litness;
        _SetColorRaw(i, color_targ.r, color_targ.g, color_targ.b);
    }

    if (RuntimeCnt >= effect_period_)
    {
        _ActivateStaticMode();
    }
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::_ApplyBlinkFrame()
{
    if (LedNums == 0) return;

    uint8_t blink_times = (effect_blink_times_ == 0) ? 1 : effect_blink_times_;
    float slot_period = _NormalizePeriod(effect_period_);
    float cycle_period = blink_times * slot_period * 2.0f;

    float delta_t = DWT_GetDeltaTime(&dwt_tick);
    RuntimeCnt += delta_t;
    if (RuntimeCnt >= cycle_period) RuntimeCnt = fmod(RuntimeCnt, cycle_period);

    bool should_on = false;
    if (RuntimeCnt < (blink_times * slot_period))
    {
        should_on = fmod(RuntimeCnt, slot_period) < (slot_period * 0.5f);
    }

    if (should_on == blink_state_on_) return;

    blink_state_on_ = should_on;
    if (should_on)
    {
        Color color = effect_color_0_ * BiasFactor;
        _FillFrame(color.r, color.g, color.b);
    }
    else
    {
        _FillFrame(0, 0, 0);
    }
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::_ApplyProgBarFrame()
{
    if (LedNums == 0 || prog_bar_value_ == nullptr) return;

    float progress = *prog_bar_value_;
    if (!(progress >= 0.0f)) progress = 0.0f;
    if (progress > 1.0f) progress = 1.0f;

    if (prog_bar_last_value_ >= 0.0f && fabsf(progress - prog_bar_last_value_) < 1e-5f)
    {
        return;
    }

    prog_bar_last_value_ = progress;

    Color color = prog_bar_color_ * BiasFactor;
    float lit_position = progress * LedNums;

    for (int i = 0; i < LedNums; i++)
    {
        float intensity = _Clamp01(lit_position - static_cast<float>(i));
        if (intensity <= 0.0f)
        {
            _SetColorRaw(i, 0, 0, 0);
            continue;
        }

        Color color_targ = color * intensity;
        _SetColorRaw(i, color_targ.r, color_targ.g, color_targ.b);
    }
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::Fill(uint8_t R, uint8_t G, uint8_t B)
{
    _ActivateStaticMode();
    _FillFrame(R, G, B);
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::Breath(Color color_0, float period)
{
    float normalized_period = _NormalizePeriod(period);
    bool keep_runtime = (current_mode_ == EffectMode::Breath)
        && _IsSameColor(effect_color_0_, color_0)
        && (effect_period_ == normalized_period);

    effect_color_0_ = color_0;
    effect_period_ = normalized_period;
    current_mode_ = EffectMode::Breath;

    if (!keep_runtime) _ResetEffectRuntime();
    _ApplyBreathFrame();
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::GradientFlow(Color color_0, Color color_1, float period)
{
    float normalized_period = _NormalizePeriod(period);
    bool keep_runtime = (current_mode_ == EffectMode::GradientFlow)
        && _IsSameColor(effect_color_0_, color_0)
        && _IsSameColor(effect_color_1_, color_1)
        && (effect_period_ == normalized_period);

    effect_color_0_ = color_0;
    effect_color_1_ = color_1;
    effect_period_ = normalized_period;
    current_mode_ = EffectMode::GradientFlow;

    if (!keep_runtime) _ResetEffectRuntime();
    _ApplyGradientFlowFrame();
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::Running(Color color, float width, float period)
{
    float normalized_period = _NormalizePeriod(period);
    bool keep_runtime = (current_mode_ == EffectMode::Running)
        && _IsSameColor(effect_color_0_, color)
        && (effect_width_ == width)
        && (effect_period_ == normalized_period);

    effect_color_0_ = color;
    effect_width_ = width;
    effect_period_ = normalized_period;
    current_mode_ = EffectMode::Running;

    if (!keep_runtime) _ResetEffectRuntime();
    _ApplyRunningFrame();
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::Blink(Color color, uint8_t times, float period)
{
    uint8_t normalized_times = (times == 0) ? 1 : times;
    float normalized_period = _NormalizePeriod(period);
    bool keep_runtime = (current_mode_ == EffectMode::Blink)
        && _IsSameColor(effect_color_0_, color)
        && (effect_blink_times_ == normalized_times)
        && (effect_period_ == normalized_period);

    effect_color_0_ = color;
    effect_blink_times_ = normalized_times;
    effect_period_ = normalized_period;
    current_mode_ = EffectMode::Blink;

    if (!keep_runtime) _ResetEffectRuntime();
    _ApplyBlinkFrame();
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::Lit(Color color)
{
    color = color * BiasFactor;
    Fill(color.r, color.g, color.b);
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::Expand(Color color_0, float period)
{
    float normalized_period = _NormalizePeriod(period);
    bool keep_runtime = (current_mode_ == EffectMode::Expand)
        && _IsSameColor(effect_color_0_, color_0)
        && (effect_period_ == normalized_period);

    effect_color_0_ = color_0;
    effect_period_ = normalized_period;
    current_mode_ = EffectMode::Expand;

    if (!keep_runtime) _ResetEffectRuntime();
    _ApplyExpandFrame();
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::ProgBar(Color color, float* progress)
{
    if (progress == nullptr)
    {
        BspLog_LogError("[WS2812]: ProgBar at null progress pointer!\r\n");
        return;
    }

    prog_bar_color_ = color;
    prog_bar_value_ = progress;
    prog_bar_last_value_ = -1.0f;
    current_mode_ = EffectMode::ProgBar;

    _ApplyProgBarFrame();
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::SetHandlePrescaler(uint8_t prescaler)
{
    handle_prescaler_ = (prescaler == 0) ? 1 : prescaler;
    handle_div_cnt_ = 0;
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::Handle()
{
    if (!initialized_) return;

    if (++handle_div_cnt_ < handle_prescaler_) return;
    handle_div_cnt_ = 0;

    switch (current_mode_)
    {
        case EffectMode::Static:
            if (!dirty_) return;
            break;
        case EffectMode::Breath:
            _ApplyBreathFrame();
            break;
        case EffectMode::GradientFlow:
            _ApplyGradientFlowFrame();
            break;
        case EffectMode::Running:
            _ApplyRunningFrame();
            break;
        case EffectMode::Expand:
            _ApplyExpandFrame();
            break;
        case EffectMode::Blink:
            _ApplyBlinkFrame();
            break;
        case EffectMode::ProgBar:
            _ApplyProgBarFrame();
            break;
        default:
            return;
    }

    if (!dirty_) return;
    Upload();
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::Upload()
{
    BSP::TIM::StartDma(tim_id, Channel, WS2812buf2send, LedNums * kBitsPerLed + kResetSlots);
    dirty_ = false;
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::_PulseFinishedCb(BSP::TIM::TimID tim, BSP::TIM::Channel channel, void* user_ctx)
{
    BSP::TIM::StopDma(tim, channel);
}

#endif
