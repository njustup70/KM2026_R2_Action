#pragma once
#include "bsp_gpio.hpp"

// 若使用新版小黄（PUHUADA，小黑），对应GPIO要设置为Pull up模式

class InfraredDetector
{
public:
    InfraredDetector(){};

    void Init(Pin pin);
    bool Read() const;
private:
    BSP::GPIO::Inst inst;
};

