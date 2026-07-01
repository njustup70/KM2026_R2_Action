#include "VacuumClaw.hpp"
#include "StateCore.hpp"

#define SNAP_TOTAL_POS  297000.0f // 总实际行程是 297227，逆时针开爪，爪闭合是正

VacuumClaw& APP::vac_claw = VacuumClaw::GetInstance();

void VacuumClaw::Start()
{
    motor_stretch.Init(Hardware::hcan_main, 6, DJI_C620);
    motor_snap.Init(Hardware::hcan_main, 7, DJI_C620);

    // ADRC控制
    motor_stretch.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(1.0e-4f, 0.30f, 0.005f)
        .ADRC_Limit(10.0f)
        .SpdLimit(1000.0f)
        .ADRC_MaxPlannedVel(800.0f)
        .ADRC_SOTF(0.12f)
        .Apply();
    motor_stretch.driver.Enable();
    motor_stretch.SetDt(0.002);

    // ADRC控制
    motor_snap.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(9.0e-5f, 0.30f, 0.005f)
        .ADRC_Limit(15.0f)
        .SpdLimit(1500.0f)
        .ADRC_MaxPlannedVel(1000.0f)
        .ADRC_SOTF(0.0067f)
        .Apply();
    motor_snap.driver.Enable();

    /****  翻转 DM (hcan_sub, ID 10) ，零位：吸盘面竖直，总行程 0 到 -1.6 rad ****/ 
    motor_pitch.Init(Hardware::hcan_sub, 11, 21, POSANDVEL, DM_J4340);
    // motor_pitch.SetAutoEnable(1);
    motor_pitch.SetFeedbackRange(2.0f, 20.0f, 20.0f);
    motor_pitch.SetPosLimit(pitch_lim_max, pitch_lim_min);
    motor_pitch.SetVelLimit(5.0f);
    motor_pitch.Enable();

    // Seq::Wait(1.0f);
    // motor_pitch.SetZeroPosi();
    // Seq::Wait(0.03);
}

void VacuumClaw::Update()
{
    if (!enabled) return;

    if(System.out_from_debugmode)
    {
        motor_stretch.Neutral();
        motor_snap.Neutral();
        motor_pitch.Disable();
    }

    // 控制前伸和收回
    motor_stretch.SetPos(stretch_pos); // 伸出为正
    motor_snap.SetPos(snap_pos); // 张开为负

    // if(!_snap_origined)
    // {
    //     _GetClawOrigin();
    // }
    
    // 第一次 update 的时候达妙设置一次零点
    if(set_damiao_zero)
    {
        set_damiao_zero = false;
        motor_pitch.SetPosVel(0.0f, 1.5);
    }

    static int damiao_cnt = 0;
    damiao_cnt++;
    {
        if(damiao_cnt >= 4)
        {
            damiao_cnt = 0;
            motor_pitch.SetPosVel(-pitch_pos, 1.5);
        }
    }

    if(is_out_debug)
    {
        is_out_debug = false;
        StretchOut();
    }

    if(is_back_debug)
    {
        is_back_debug = false;
        StretchBack();
    }
}

/**
  * @brief 伸出夹爪，保持开启状态
  * @details 水平快速伸出，同时Pitch变为水平状态，用于夹取的snap电机打开
  */
void VacuumClaw::StretchOut()
{
    pitch_pos = 0.0f;
    motor_pitch.SetPosVel(-pitch_pos, 1.5);

    // 张开前爪
//    snap_pos

    // 等爪子pitch竖直
    Seq::Wait(1.5f);

    // 伸出
    stretch_pos = 252000;
}

void VacuumClaw::StretchBack()
{
    pitch_pos = 1.7; // 翻转物块
    motor_pitch.SetPosVel(-pitch_pos, 1.5);

    // 等物块被翻转过来
    Seq::Wait(1.5f);

    // 收回
    stretch_pos = 1000.0; // 防堵转，给个略大于0的值
}

void VacuumClaw::Enable()
{
    enabled = true;
}

void VacuumClaw::Disable()
{
    enabled = false;
}

void VacuumClaw::OpenSnap()
{
    snap_pos = 1000.0f; // 留点余量
}

void VacuumClaw::CloseSnap()
{
    snap_pos = 279500.0f; // 留点余量
}

// void VacuumClaw::_GetClawOrigin()
// {
//     static uint32_t runed_tick = 0;  
//     static int32_t snap_probe_code = 0;
//     static int32_t snap_probe_cnt = 0;
//     runed_tick++;

//     // 对于竖直方向的电机，让他慢慢撞向右边的限位（PosCode增大的方向）
//     motor_snap.SetPos(-snap_probe_code);

//     // 如果撞到限位了（用eso的扰动观测来确定），连续5个Tick，就停止
//     if (fabs(motor_snap.motor_adrc.eso.z3) > 2000 && runed_tick > 30) // 注意正负号！
//     {
//         snap_probe_cnt++;
//         if (snap_probe_cnt >= 10)
//         {
//             // 完成零位校准
//             _snap_origined = true;
//             // 记录中点偏移
//             _snap_origin_code = motor_snap.driver.measure.total_angle;
//             // 往回一点点，防止卡住影响其他轴
//             snap_pos = _snap_origin_code + 150000;
//             motor_snap.SetPos(snap_pos);
//         }
//     }
//     else
//     {
//         snap_probe_cnt = 0;
//         // 每周期（5ms）加 20 Code，每秒加 4000 Code
//         if (runed_tick > 30)    snap_probe_code -= 80;
//         // 但是最开始缓慢前进，目的是破除静态库伦摩擦力
//         else                    snap_probe_code -= 15;
//     }   
// }
