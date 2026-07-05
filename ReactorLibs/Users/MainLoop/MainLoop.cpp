#include "MainLoop.hpp"
#include "HostPC.hpp"
#include "farcon.hpp"
#include "CommCenter.hpp"

MainLoop& APP::main_loop = MainLoop::GetInstance();

float debug_cnt = 0;

void MainLoop::Start()
{
    test_led.Init(Hardware::tim_led, BSP::TIM::CH4);
    test_led.BiasFactor = Vec3(0.843f, 1.0f, 0.843f);

    Pin spear_ir = {'E', 6};
    
    // 初始化小黄
    spear_detector.Init(spear_ir);

    test_led.Lit(Color(0.0f, 0.0f, 0.0f));
    test_led.Segment(2);
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
    ToggleAreaLogics();
}


void MainLoop::RefreshFieldLed()
{
    test_led.SegLit(0, targ_area == AREA_BLUE ? Color::Blue : Color::Red);
    test_led.SegLit(1, APP::comm_center.AtBlueField() ? Color::Blue : Color::Red);
}


void MainLoop::ToggleAreaLogics()
{
    // 根据当前的红蓝区，4Hz (50分频) 持续发送给工控机，确保工控机知道当前的区域选择
    static int area_red_blue_cnt = 0;
    // 如果已经确定了分区，可以不用继续发了
    static bool confirmed_area = false;
    // 防止一直触发灯带的标志位
    static bool last_host_atblue = false;
    static bool field_led_initialized = false;

    if (!field_led_initialized)
    {
        RefreshFieldLed();
        last_host_atblue = APP::comm_center.AtBlueField();
        field_led_initialized = true;
    }

    // 发送红蓝区选择给工控机
    if (area_red_blue_cnt++ >= 50 && !confirmed_area)
    {
        area_red_blue_cnt = 0;
        if (targ_area == AREA_RED)
        {
            MOD::host_pc.SendFieldChoice(HostPC::RED);
        }
        else
        {
            MOD::host_pc.SendFieldChoice(HostPC::BLUE);
        }
    }

    // 红蓝区逻辑（STM32 维护）
    if (MOD::farcon.toggle[3] == 1 && targ_area == AREA_RED)
    {
        targ_area = AREA_BLUE;
        APP::monit.LogRespond("Toggle Area: BLUE");
        RefreshFieldLed();
    }
    else if (MOD::farcon.toggle[3] == 0 && targ_area == AREA_BLUE)
    {
        targ_area = AREA_RED;
        APP::monit.LogRespond("Toggle Area: RED");
        RefreshFieldLed();
    }

    // 红蓝区逻辑（工控机 维护）
    // 如果工控机反馈当前在蓝区，而上一次反馈在红区，则说明切换到了蓝区
    if (APP::comm_center.AtBlueField() && !last_host_atblue)
    {
        last_host_atblue = true;
        RefreshFieldLed();
    }
    // 如果工控机反馈当前在红区，而上一次反馈在蓝区，则说明切换到了红区
    else if (APP::comm_center.AtRedField() && last_host_atblue)
    {
        last_host_atblue = false;
        RefreshFieldLed();
    }

    // 如果状态机尚未启动（还在选图），就根据选择的图，启动对应的状态图
    if (!graph_started && MOD::farcon.button_second_half[0] == 1)
    {
        graph_started = true;
        if (targ_area == AREA_RED)
        {
            APP::monit.LogRespond("Start Challenge Graph: RED");
            APP::state_core.Enable(Challenge_Explore_RedGr_id);
            confirmed_area = true;
        }
        else
        {
            APP::monit.LogRespond("Start Challenge Graph: BLUE");
            APP::state_core.Enable(Challenge_Explore_BlueGr_id);
            confirmed_area = true;
        }
    }
}
