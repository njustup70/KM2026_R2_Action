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
        .SetDt(0.002f)
        .Apply();
    motor_stretch.driver.Enable();
    
    // ADRC控制
    motor_snap.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(9.0e-5f, 0.30f, 0.005f)
        .ADRC_Limit(15.0f)
        .SpdLimit(2000.0f)
        .ADRC_MaxPlannedVel(1800.0f)
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

    Pin clawzero_ir = {'E', 12};
    // 调零的小黄
    clawzero_detector.Init(clawzero_ir);

    // 达妙初始为竖直
    pitch_pos = 1.6f;

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

    if(!_snap_origined)
    {
        if (!motor_snap.driver.online)
        {
            if (motor_snap.driver.enabled)
            {
                APP::monit.LogError("Dangerous to turn on claw later!");
                motor_snap.driver.Disable();
            }
            else return;
        }
        _GetClawOrigin();
    }

    static int damiao_cnt = 0;
    damiao_cnt++;
    if(damiao_cnt >= 4)
    {
        damiao_cnt = 0;
        motor_pitch.SetPosVel(-pitch_pos, 1.5);
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

    // // 伸出
    // stretch_pos = 252000;

    // 第一阶段：快速伸到 20000
    stretch_pos = 180000;
    Seq::Wait(0.3f);

    // 第二阶段：梯形缓慢伸出到 252000
    constexpr float target_pos = 252000.0f;
    constexpr float step_size  = 2800.0f;   // 每步增量
    constexpr float step_dt    = 0.1f;       // 每步间隔

    for (float pos = 180000.0f + step_size; pos < target_pos; pos += step_size)
    {
        stretch_pos = pos;
        Seq::Wait(step_dt);
    }
    stretch_pos = target_pos;
}

// 取块第一步，先伸出一点，可以定心
void VacuumClaw::StretchOut_First()
{
    // pitch 竖直
    pitch_pos = 0.0f;
    motor_pitch.SetPosVel(-pitch_pos, 1.5);    

    stretch_pos = 150000;
}

// 取块第二步，开泵了，继续伸出剩下部分
void VacuumClaw::StretchOut_Second()
{
    stretch_pos = 252000;
}

void VacuumClaw::StretchBack()
{
    pitch_pos = 1.7; // 翻转物块
    motor_pitch.SetPosVel(-pitch_pos, 1.5);

    // 等物块被翻转过来
    Seq::Wait(2.4f);

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
    if (_snap_origined) 
        snap_pos = _snap_origin_code - 102000; // 留点余量
    else
        APP::monit.LogWarning("Unorigined snapclaw!");
}

void VacuumClaw::OpenSnapMini()
{
    if (_snap_origined) 
        snap_pos = _snap_origin_code + 180000; // 这是合上爪子后，为了能吸住物块，微微张开爪子让车往前走一小段彻底吸住物块
    else
        APP::monit.LogWarning("Unorigined snapclaw!");   
}

void VacuumClaw::CloseSnap()
{
    if (_snap_origined)
        snap_pos = _snap_origin_code + 185000;
    else
        APP::monit.LogWarning("Unorigined snapclaw!");
}

void VacuumClaw::_GetClawOrigin()
{
    static uint32_t runed_tick = 0;  
    static int32_t snap_probe_code = 0;
    static int32_t snap_probe_cnt = 0;
    runed_tick++;

    // 对于竖直方向的电机，让他慢慢撞向右边的限位（PosCode增大的方向）
    motor_snap.SetPos(snap_probe_code);

    // 如果被小黄看到了，连续5个Tick，就停止
    if (clawzero_detector.Read() && runed_tick > 30) // 注意正负号！
    {
        snap_probe_cnt++;
        if (snap_probe_cnt >= 10)
        {
            // 完成零位校准
            _snap_origined = true;

            // 记录中点偏移
            _snap_origin_code = motor_snap.driver.measure.total_angle;

            // 走到最大位置
            snap_pos = _snap_origin_code - 102000;
            motor_snap.SetPos(snap_pos);
        }
    }
    else
    {
        snap_probe_cnt = 0;
        // 每周期（5ms）加 20 Code，每秒加 4000 Code
        if (runed_tick > 30)    snap_probe_code += 80;
        // 但是最开始缓慢前进，目的是破除静态库伦摩擦力
        else                    snap_probe_code += 15;
    }
}
