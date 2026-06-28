#pragma once
#include "System.hpp"

#include "led_ws2812.hpp"

class MainLoop : public Application
{
    SINGLETON(MainLoop) : Application("MainLoop") {};
    APPLICATION_OVERRIDE

    public:
    LedWs2812<13> test_led;
    int cnt = 0;
};

namespace APP
{
    extern MainLoop& main_loop;
}