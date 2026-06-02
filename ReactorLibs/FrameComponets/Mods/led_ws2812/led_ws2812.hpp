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
    uint32_t PwmMaxValue = 0;       // PWM最大值
    uint32_t HIGH_WS2812 = 0;       // PWM高电平数值
    uint32_t LOW_WS2812 = 0;        // PWM低电平数值

    // 这俩变量是给灯带动画用的，就是如果要渐变效果的话，循环调用就用这个内部计时器
    uint32_t dwt_tick = 0;
    float RuntimeCnt = 0;

    // 灯带的写入缓冲区，每颗灯24bit，颜色顺序是GRB
    uint32_t WS2812buf2send[MaxLedNumsT][24] = {0};

public:
    LedWs2812() {};
    ~LedWs2812() {};

    BSP::TIM::TimID tim_id = nullptr;
    uint32_t Channel = 0;

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
     * @brief 初始化灯带驱动
     * @param tim WS2812对应的定时器ID
     * @param Channel PWM通道
     * @note 实际灯珠数固定为模板参数 MaxLedNumsT
     */
    void Init(BSP::TIM::TimID tim, uint32_t Channel);


    /*---   一些预设的动画  ---*/
    void Fill(uint8_t R, uint8_t G, uint8_t B);
    void Breath(Color color_0, float period);
    void GradientFlow(Color color_0, Color color_1, float period);
    void Running(Color color, float width, float period);
    void Lit(Color color); // 供外部调用的Fill，省算力
    void Expand(Color color_0, float period);

    static void _PulseFinishedCb(BSP::TIM::TimID tim, uint32_t channel, void* user_ctx);
};

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::Init(BSP::TIM::TimID tim, uint32_t Channel)
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

    // 灯带颜色默认全灭
    for (size_t i = 0; i < MaxLedNumsT; i++)
    {
        SetColor((int8_t)i, 0, 0, 0);
    }

    // 初始化PWM
    PwmMaxValue = BSP::TIM::GetAutoReload(this->tim_id);
    HIGH_WS2812 = PwmMaxValue * 0.67f;
    LOW_WS2812 = PwmMaxValue * 0.33f;

    // 注册 PWM 完成回调
    BSP::TIM::RegistPFCb(this->tim_id, this->Channel, _PulseFinishedCb, this);
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::SetColor(int8_t Led_id, uint8_t R, uint8_t G, uint8_t B)
{
    while (Led_id > (LedNums - 1)) Led_id -= LedNums;
    while (Led_id < 0) Led_id += LedNums;

    for (int i = 0; i < 8; i++) WS2812buf2send[Led_id][i] = (G & (1 << (7 - i)) ? (HIGH_WS2812) : LOW_WS2812);
    for (int i = 8; i < 16; i++) WS2812buf2send[Led_id][i] = (R & (1 << (15 - i)) ? (HIGH_WS2812) : LOW_WS2812);
    for (int i = 16; i < 24; i++) WS2812buf2send[Led_id][i] = (B & (1 << (23 - i)) ? (HIGH_WS2812) : LOW_WS2812);
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::GradientFlow(Color color_0, Color color_1, float period)
{
    color_0 = color_0 * BiasFactor;
    color_1 = color_1 * BiasFactor;

    float delta_t = DWT_GetDeltaTime(&dwt_tick);
    RuntimeCnt += delta_t;
    if (RuntimeCnt >= period) {
        RuntimeCnt = fmod(RuntimeCnt, period);
    }

    float phaseStep = (float)period / LedNums;

    for (int i = 0; i < LedNums; i++)
    {
        float phase = fmod(RuntimeCnt + (i * phaseStep), period);
        float ratio = 0.5f * (1.0f + sinf(2 * 3.1415f * phase / period - 1.5708f));
        Color color_targ = (color_0 + (color_1 - color_0) * ratio);
        SetColor(i, color_targ.r, color_targ.g, color_targ.b);
    }
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::Breath(Color color_0, float period)
{
    color_0 = color_0 * BiasFactor;

    float delta_t = DWT_GetDeltaTime(&dwt_tick);
    RuntimeCnt += delta_t;
    if (RuntimeCnt >= period) {
        RuntimeCnt = fmod(RuntimeCnt, period);
    }

    float ratio = 0.5f * (1.0f + sinf((2 * PI) * RuntimeCnt / period - (PI / 2)));
    Color color_targ = color_0 * ratio;
    Fill(color_targ.r, color_targ.g, color_targ.b);
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::Running(Color color, float width, float period)
{
    color = color * BiasFactor;

    float delta_t = DWT_GetDeltaTime(&dwt_tick);
    RuntimeCnt += delta_t;
    if (RuntimeCnt >= period) {
        RuntimeCnt = fmod(RuntimeCnt, period);
    }

    float position = (RuntimeCnt / period) * LedNums;
    int width_in_leds = width * LedNums;

    for (int i = 0; i < LedNums; i++)
    {
        float distance = fabsf(i - position);

        if (distance <= (width_in_leds / 2) || distance >= (LedNums - width_in_leds / 2))
        {
            SetColor(i, color.r, color.g, color.b);
        }
        else
        {
            SetColor(i, 0, 0, 0);
        }
    }
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::Lit(Color color)
{
    RuntimeCnt += DWT_GetDeltaTime(&dwt_tick);
    if (RuntimeCnt > 0.5f)
    {
        color = color * BiasFactor;
        Fill(color.r, color.g, color.b);
        RuntimeCnt = 0;
    }
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::Expand(Color color_0, float period)
{
    float delta_t = DWT_GetDeltaTime(&dwt_tick);

    if (RuntimeCnt < period)
    {
        RuntimeCnt += delta_t;
        float ratio = RuntimeCnt / period;
        float center = LedNums / 2.0f;

        for (int i = 0; i < LedNums; i++)
        {
            float litness = 0;
            if (i < center) litness = (i / center) + 2 * ratio - 1;
            else litness = 2 * ratio - ((float)i - center) / center;

            if (litness < 0) litness = 0;
            if (litness > 1) litness = 1;
            Color color_targ = color_0 * litness;
            SetColor(i, color_targ.r, color_targ.g, color_targ.b);
        }
    }
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::Fill(uint8_t R, uint8_t G, uint8_t B)
{
    for (int i = 0; i < LedNums; i++)
    {
        SetColor(i, R, G, B);
    }
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::Upload()
{
    BSP::TIM::StartDma(tim_id, Channel, (uint32_t*)WS2812buf2send, LedNums * 24 + 1);
}

template <size_t MaxLedNumsT>
void LedWs2812<MaxLedNumsT>::_PulseFinishedCb(BSP::TIM::TimID tim, uint32_t channel, void* user_ctx)
{
    (void)user_ctx;
    BSP::TIM::StopDma(tim, channel);
}

#endif

