#include "MainLoop.hpp"
#include "HostPC.hpp"

MainLoop& APP::main_loop = MainLoop::GetInstance();

float debug_cnt = 0;

void MainLoop::Start()
{
    test_led.Init(Hardware::tim_led, BSP::TIM::CH4);
    test_led.BiasFactor = Vec3(0.843f, 1.0f, 0.843f);
    // test_led.GradientFlow(Color(214,223,53) * 0.25f, Color(105,23,150) * 0.25f, 2.0f);

    Pin spear_ir = {'E', 6};
    
    // 初始化小黄
    spear_detector.Init(spear_ir);
    
}

void MainLoop::Update()
{
    test_led.Handle();
    static float last_report_time = 0.0f;
    spear_detected = spear_detector.Read();
    cnt++;

    if (System.runtime_tick - last_report_time >= 15.0f)
    {
        last_report_time = System.runtime_tick;
        debug_cnt = MOD::host_pc.GetRecentRxFreqHz();
        APP::monit.LogInfo("[HostPC] RX_freq: %.1f Hz, cnt:%d", MOD::host_pc.GetRecentRxFreqHz(), cnt);
    }
}
