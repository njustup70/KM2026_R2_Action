#include "servo.hpp"
#include "arm_math.h"

void Servo::Init(BSP::TIM::TimID tim, BSP::TIM::Channel channel)
{
    // 确认参数合法性
    if (tim == nullptr || channel == 0) return;
    
    // 存储信息
    tim_id = tim;
    this->channel = channel;
    initialized = true;
}

void Servo::Enable()
{
    if (!initialized) return;

    // 确认一下频率等于50Hz，舵机一般都是这个频率
    if (fabs(BSP::TIM::GetFreq(tim_id) - 50.0f) > 0.1f) return;
    
    // 启动PWM输出
    enabled = true;
    BSP::TIM::StartPWM(tim_id, channel);
}

void Servo::Disable()
{
    if (!initialized) return;
    
    enabled = false;
    BSP::TIM::StopPWM(tim_id, channel);
}


void Servo::SetAngle(float ang)
{
    if (!initialized) return;
    
    // 假设舵机的工作范围是-90到90度，对应的PWM占空比是2.5%到12.5%
    if (ang < -90.0f) ang = -90.0f;
    if (ang > 90.0f) ang = 90.0f;
    
    // 给类成员赋值
    angle = ang;

    // 通过线性映射计算占空比
    float duty = (angle + 90.0f) / 180.0f * (0.125f - 0.025f) + 0.025f;
    BSP::TIM::SetDuty(tim_id, channel, duty);
}








