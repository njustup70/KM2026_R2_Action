#include "MainLoop.hpp"
#include "HostPC.hpp"
#include "farcon.hpp"
#include "CommCenter.hpp"

MainLoop& APP::main_loop = MainLoop::GetInstance();

#define Challenge_Area12_EXPLORE
// #define Challenge_Area3_ARENA
// #define Compete_KungfuMaster


float debug_cnt = 0;

void MainLoop::Start()
{
    test_led.Init(Hardware::tim_led, BSP::TIM::CH4);
    test_led.BiasFactor = Vec3(0.843f, 1.0f, 0.843f);

    // 取矛头用的小黄
    Pin spear_ir = {'E', 6};
    spear_detector.Init(spear_ir);

    /*****      四个按钮（当小黄处理了）      *****/
    // 第一个：红蓝区切换
    Pin button_0_pin = {'I', 9};
    button_0.Init(button_0_pin);

    // 第二个：一区/三区重试点
    Pin button_1_pin = {'F', 10};
    button_1.Init(button_1_pin);

    // 第三和第四个：重试码
    Pin button_2_pin = {'A', 4};
    Pin button_3_pin = {'A', 5};
    button_2.Init(button_2_pin);
    button_3.Init(button_3_pin);

    test_led.Lit(Color(0.0f, 0.0f, 0.0f));
    test_led.Segment(4);

    // 牢记修改：最大重试类型ID
    max_retry_id = 3;

    RefreshFieldLed();
    RefreshRetryLed();
}

void MainLoop::Update()
{
    test_led.Handle();
    static float last_report_time = 0.0f;
    spear_detected = spear_detector.Read();

    // 读取按键
    button_pressed[0] = !button_0.Read();
    button_pressed[1] = !button_1.Read();
    button_pressed[2] = !button_2.Read();
    button_pressed[3] = !button_3.Read();

    cnt++;

    if (System.runtime_tick - last_report_time >= 15.0f)
    {
        last_report_time = System.runtime_tick;
        debug_cnt = MOD::host_pc.GetRecentRxFreqHz();
        APP::monit.LogInfo("[HostPC] RX_freq: %.1f Hz, cnt:%d", MOD::host_pc.GetRecentRxFreqHz(), cnt);
    }

    // 在此 处理状态机的启动逻辑。原因是：MainFrame 无法阻塞等待判断，只能在这里
    ToggleAreaLogics();

    // 处理重试逻辑
    if (!graph_started) JudgeRetryFlags();

    static bool kfs_blinked = false;
    if (APP::comm_center.IsKFSSeqRecved() && !kfs_blinked)
    {
        test_led.TempBlink(Color::Pink, 3, 0.5f);
        kfs_blinked = true;
    }
}


void MainLoop::RefreshFieldLed()
{
    test_led.SegLit(0, targ_area == AREA_BLUE ? Color::Blue : Color::Red);
    test_led.SegLit(1, APP::comm_center.AtBlueField() ? Color::Blue : Color::Red);
    test_led.SegLit(2, APP::comm_center.AtAreaOne() ? Color::Pink : Color::Orange);
}


void MainLoop::RefreshRetryLed()
{
    Color retry_color = Color::White;

    switch (retry_id)
    {
    case 1:
        retry_color = Color::Orange;
        break;
    case 2:
        retry_color = Color::Yellow;
        break;
    case 3:
        retry_color = Color::Green;
        break;
    default:
        retry_color = Color::White;
        break;
    }

    test_led.SegLit(3, retry_color);
}

int area_red_blue_cnt = 0;
void MainLoop::ToggleAreaLogics()
{
    // 根据当前的红蓝区，4Hz (50分频) 持续发送给工控机，确保工控机知道当前的区域选择
    // static int area_red_blue_cnt = 0;
    // 如果已经确定了分区，可以不用继续发了
    static bool confirmed_area = false;
    // 防止一直触发灯带的标志位
    static bool last_host_atblue = false;
    static bool field_led_initialized = false;

    static bool last_host_start_at_one = false;

    if (!field_led_initialized)
    {
        RefreshFieldLed();
        last_host_atblue = APP::comm_center.AtBlueField();
        last_host_start_at_one = APP::comm_center.AtAreaOne();
        field_led_initialized = true;
    }

    // 发送红蓝区选择给工控机
    if (area_red_blue_cnt++ >= 50 && !confirmed_area)
    {
        area_red_blue_cnt = 0;
        if (targ_area == AREA_RED)
        {
            if(targ_start_area == AREA_ONE)
            {
                MOD::host_pc.SendFieldChoice(HostPC::RED, HostPC::AREA_ONE);
            }  
            else
            {
                MOD::host_pc.SendFieldChoice(HostPC::RED, HostPC::AREA_THREE);                
            }
        }
        else
        {
            if(targ_start_area == AREA_ONE)
            {
                MOD::host_pc.SendFieldChoice(HostPC::BLUE, HostPC::AREA_ONE);
            }  
            else
            {
                MOD::host_pc.SendFieldChoice(HostPC::BLUE, HostPC::AREA_THREE);                
            }        
        }
    }

    // 红蓝区逻辑（STM32 维护）
    if ((MOD::farcon.toggle[3] == 1 ) && targ_area == AREA_RED)
    {
        targ_area = AREA_BLUE;
        APP::monit.LogRespond("Toggle Area: BLUE");
        RefreshFieldLed();
    }
    else if ((MOD::farcon.toggle[3] == 0) && targ_area == AREA_BLUE)
    {
        targ_area = AREA_RED;
        APP::monit.LogRespond("Toggle Area: RED");
        RefreshFieldLed();
    }
    
    // 启动坐标切换逻辑（STM32 维护）
    if ((MOD::farcon.toggle[2] == 1 ) && targ_start_area == AREA_ONE)
    {
        targ_start_area = AREA_THREE;
        APP::monit.LogRespond("Start Area: Area Three");
        RefreshFieldLed();
    }
    else if ((MOD::farcon.toggle[2] == 0) && targ_start_area == AREA_THREE)
    {
        targ_start_area = AREA_ONE;
        APP::monit.LogRespond("Start Area: Area One");
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

    // 启动坐标切换逻辑（工控机 维护）
    if(APP::comm_center.AtAreaOne() && !last_host_start_at_one)
    {
        last_host_start_at_one = true;
        RefreshFieldLed();
    }
    else if(APP::comm_center.AtAreaThree() && last_host_start_at_one)
    {
        last_host_start_at_one = false;
        RefreshFieldLed();        
    }

    // 如果状态机尚未启动（还在选图），就根据选择的图，启动对应的状态图
    if (!graph_started && (MOD::farcon.button_second_half[0] == 1 || button_pressed[3] == 1))
    {
        graph_started = true;
        if (targ_area == AREA_RED)
        {
            APP::monit.LogRespond("Start Challenge Graph: RED");
            
            #if defined(Challenge_Area12_EXPLORE)
            APP::state_core.Enable(Challenge_Explore_RedGr_id);
            #elif defined(Challenge_Area3_ARENA)
            APP::state_core.Enable(Challenge_Treasure_RedGr_id);
            #elif defined(Compete_KungfuMaster)
            APP::state_core.Enable(Compete_KM_RedGr_id);
            #else
            APP::monit.LogError("Start Challenge Graph failed: no challenge macro defined.");
            #endif
            confirmed_area = true;
        }
        else
        {
            APP::monit.LogRespond("Start Challenge Graph: BLUE");

            #if defined(Challenge_Area12_EXPLORE)
            APP::state_core.Enable(Challenge_Explore_BlueGr_id);
            #elif defined(Challenge_Area3_ARENA)
            APP::state_core.Enable(Challenge_Treasure_BlueGr_id);
            #elif defined(Compete_KungfuMaster)
            APP::state_core.Enable(Compete_KM_BlueGr_id);
            #else
            APP::monit.LogError("Start Challenge Graph failed: no challenge macro defined.");
            #endif

            confirmed_area = true;
        }
    }
}

void MainLoop::JudgeRetryFlags()
{
    static bool last_retry_button_pressed = false;

    // 处理重试的逻辑：只在 button[2] 的上升沿触发一次
    if (button_pressed[2] && !last_retry_button_pressed)
    {
        APP::monit.LogInfo("Toggle Retry");
        retry_id++;
        if (retry_id > max_retry_id)
        {
            retry_id = 0;
        }

        RefreshRetryLed();
    }

    last_retry_button_pressed = button_pressed[2];
}

///@brief 遥控器按键控制逻辑
void ButtonLogics()
{
    // 第二半，7号按键，重启SLAM
    static uint8_t last_restart_time = 0;
    if(MOD::farcon.button_second_half[6] && DWT_GetTimeline_Sec() - last_restart_time > 1.0f)
    {
        last_restart_time = DWT_GetTimeline_Sec();
        MOD::host_pc.RestartSLAM();
    }
}