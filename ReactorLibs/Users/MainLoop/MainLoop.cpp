#include "MainLoop.hpp"
#include "HostPC.hpp"
#include "farcon.hpp"

MainLoop& APP::main_loop = MainLoop::GetInstance();

float debug_cnt = 0;

void MainLoop::Start()
{
    test_led.Init(Hardware::tim_led, BSP::TIM::CH4);
    test_led.BiasFactor = Vec3(0.843f, 1.0f, 0.843f);

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

    // 在此 处理状态机的启动逻辑。原因是：MainFrame 无法阻塞等待判断，只能在这里

    // 从红切蓝
    if (MOD::farcon.toggle[3] == 1 && targ_area == AREA_RED)
    {
        targ_area = AREA_BLUE;
        APP::monit.LogRespond("Toggle Area: BLUE");
        test_led.Lit(Color::Blue);
    }
    else if (MOD::farcon.toggle[3] == 0 && targ_area == AREA_BLUE)
    {
        targ_area = AREA_RED;
        APP::monit.LogRespond("Toggle Area: RED");
        test_led.Lit(Color::Red);
    }

    // 如果状态机尚未启动（还在选图），就根据选择的图，启动对应的状态图
    if (!graph_started && MOD::farcon.button_second_half[0] == 1)
    {
        graph_started = true;
        if (targ_area == AREA_RED)
        {
            APP::monit.LogRespond("Start Challenge Graph: RED");
            APP::state_core.Enable(Challenge_Explore_RedGr_id);
        }
        else
        {
            APP::monit.LogRespond("Start Challenge Graph: BLUE");
            APP::state_core.Enable(Challenge_Explore_BlueGr_id);
        }
    }
}
