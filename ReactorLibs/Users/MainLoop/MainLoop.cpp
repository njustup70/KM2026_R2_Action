#include "MainLoop.hpp"
#include "HostPC.hpp"

MainLoop& APP::main_loop = MainLoop::GetInstance();

float debug_cnt = 0;

void MainLoop::Start()
{
    test_led.Init(Hardware::tim_led, BSP::TIM::CH4);
    test_led.BiasFactor = Vec3(0.843f, 1.0f, 0.843f);
    // test_led.GradientFlow(Color(214,223,53) * 0.25f, Color(105,23,150) * 0.25f, 2.0f);
}

void MainLoop::Update()
{
    test_led.Handle();
    static float last_report_time = 0.0f;
    cnt++;

    if (DWT_GetTimeline_Sec() - last_report_time >= 4.0f)
    {
        last_report_time = DWT_GetTimeline_Sec();
        debug_cnt = MOD::host_pc.GetRecentRxFreqHz();
        APP::monit.LogInfo("[HostPC] RX_freq: %.1f Hz, cnt:%d", MOD::host_pc.GetRecentRxFreqHz(), cnt);
    }
}
