#ifndef SERVO_HPP
#define SERVO_HPP

#include "bsp_tim_pwm.hpp"

class Servo
{
public:
    Servo() {};
    void Init(BSP::TIM::TimID tim, BSP::TIM::Channel channel);
    void Enable();              // 开始输出PWM信号
    void Disable();             // 停止输出PWM信号
    void SetAngle(float angle); // 角度制，单位：度DEG

    float angle;                // 当前舵机角度，单位：度DEG
    
private:
    BSP::TIM::TimID tim_id = nullptr;
    BSP::TIM::Channel channel = BSP::TIM::CH1;
    bool initialized = false;
    bool enabled = false;
};

#endif
