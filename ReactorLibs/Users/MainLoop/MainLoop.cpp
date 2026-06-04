#include "MainLoop.hpp"

MainLoop& APP::main_loop = MainLoop::GetInstance();

void MainLoop::Start()
{
    test_led.Init(Hardware::tim_led, BSP::TIM::CH4);
    test_led.BiasFactor = Vec3(0.843f, 1.0f, 0.843f);
    // test_led.GradientFlow(Color(214,223,53) * 0.25f, Color(105,23,150) * 0.25f, 2.0f);
}

void MainLoop::Update()
{
    test_led.Handle();
}
