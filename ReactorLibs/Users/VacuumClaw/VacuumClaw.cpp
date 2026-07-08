#include "VacuumClaw.hpp"
#include "StateCore.hpp"

#define SNAP_TOTAL_POS  297000.0f // 总实际行程是 297227，逆时针开爪，爪闭合是正
#define FETCH_PITCH_POS 300000.0f // 0 - 90° 的 code 值，转到 90° 是正的，回到零位是负的

VacuumClaw& APP::vac_claw = VacuumClaw::GetInstance();

void VacuumClaw::Start()
{
    motor_stretch.Init(Hardware::hcan_main, 6, DJI_C620);
    motor_snap.Init(Hardware::hcan_main, 7, DJI_C620);
    motor_pitch.Init(Hardware::hcan_main, 8, DJI_C610);

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

    /****  翻转 2006 ，零位：吸盘面竖直，总行程  ****/ 
    motor_pitch.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(14.0f, 3.6f)
        .ADRC_Physic(1.75e-4f, 0.18f)
        .ADRC_Limit(5.0f)
        .SpdLimit(5000.0f)
        .ADRC_MaxPlannedVel(4800.0f)
        .ADRC_SOTF(0.008f)
        .SetDt(0.004f)
        .Apply();
    motor_pitch.driver.Enable();

    // /****  翻转 DM (hcan_sub, ID 10) ，零位：吸盘面竖直，总行程 0 到 -1.6 rad ****/ 
    // motor_pitch.Init(Hardware::hcan_sub, 11, 21, POSANDVEL, DM_J4340);
    // // motor_pitch.SetAutoEnable(1);
    // motor_pitch.SetFeedbackRange(2.0f, 20.0f, 20.0f);
    // motor_pitch.SetPosLimit(pitch_lim_max, pitch_lim_min);
    // motor_pitch.SetVelLimit(5.0f);
    // motor_pitch.Enable();

    Pin clawzero_ir = {'E', 12};
    // 调零的小黄
    clawzero_detector.Init(clawzero_ir);

    // // 达妙初始为竖直
    // pitch_pos = 1.6f;

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
        motor_pitch.Neutral();
    }

    if(!_origined)
    {
        // 前置条件：如果有任一电机不在线，夹爪模块拒绝运作
        if (!motor_stretch.driver.online || !motor_pitch.driver.online || !motor_snap.driver.online)
        {
            APP::monit.LogError("It's Dangerous to use [VacuumClaw], without motor online!");
            enabled = false;
            return;
        }
        _Update_GetOrigin();
        return;
    }
    else
    {
        // 控制前伸和收回
        motor_stretch.SetPos(stretch_pos); // 伸出为正
        motor_snap.SetPos(snap_pos); // 张开为负
        motor_pitch.SetPos(pitch_pos); //        
    }

    // 要先等 pitch 调零完 snap 才能调零
    // if(!_snap_origined)
    // {
    //     if (!motor_snap.driver.online)
    //     {
    //         if (motor_snap.driver.enabled)
    //         {
    //             APP::monit.LogError("Dangerous to turn on claw later!");
    //             motor_snap.driver.Disable();
    //         }
    //         else return;
    //     }
    //     _GetClawOrigin();
    // }

    // if(!_pitch_origined)
    // {
    //     if (!motor_pitch.driver.online)
    //     {
    //         if (motor_pitch.driver.enabled)
    //         {
    //             APP::monit.LogError("Dangerous to turn over claw later!");
    //             motor_pitch.driver.Disable();
    //         }
    //         else return;
    //     }
    //     _GetPitchOrigin();
    // }


    // static int damiao_cnt = 0;
    // damiao_cnt++;
    // if(damiao_cnt >= 4)
    // {
    //     damiao_cnt = 0;
    //     motor_pitch.SetPosVel(-pitch_pos, 1.5);
    // }

    // if(is_out_debug)
    // {
    //     is_out_debug = false;
    //     StretchOut();
    // }

    // if(is_back_debug)
    // {
    //     is_back_debug = false;
    //     StretchBack();
    // }
}

void VacuumClaw::_Update_GetOrigin()
{
    // 先 pitch
    if(!_pitch_origined)
    {
        _GetPitchOrigin();
    }
    // 再 snap
    if(_pitch_origined && !_snap_origined)
    {
        _GetClawOrigin();
    }

    if(_snap_origined && _pitch_origined)
    {
        _origined = true;
    }
}

/**
  * @brief 伸出夹爪，保持开启状态
  * @details 水平快速伸出，同时Pitch变为水平状态，用于夹取的snap电机打开
  */
void VacuumClaw::StretchOut()
{
    pitch_pos = pitch_origined_pos + FETCH_PITCH_POS;
    // motor_pitch.SetPosVel(-pitch_pos, 1.5);

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
    pitch_pos = pitch_origined_pos + FETCH_PITCH_POS;
    // motor_pitch.SetPosVel(-pitch_pos, 1.5);    
    stretch_pos = 190000;
}

// 取块第二步，开泵了，继续伸出剩下部分
void VacuumClaw::StretchOut_Second()
{
    // stretch_pos = 252000;
    constexpr float target_pos = 255000.0f;
    constexpr float step_size  = 8800.0f;   // 每步增量
    constexpr float step_dt    = 0.1f;       // 每步间隔

    for (float pos = 180000.0f + step_size; pos < target_pos; pos += step_size)
    {
        stretch_pos = pos;
        Seq::Wait(step_dt);
    }
    stretch_pos = target_pos;

    // 要收紧爪子，让物块不晃
    snap_pos = _snap_origin_code - 102000 + 272040;
}

///@brief 三区取块的动作，只伸出15cm左右
void VacuumClaw::StretchOut_Mini()
{
    // pitch 竖直
    pitch_pos = pitch_origined_pos + FETCH_PITCH_POS;

    // stretch_pos = 252000;
    constexpr float target_pos = 105000.0f;
    constexpr float step_size  = 20800.0f;   // 每步增量
    constexpr float step_dt    = 0.1f;       // 每步间隔

    for (float pos = 0.0f + step_size; pos < target_pos; pos += step_size)
    {
        stretch_pos = pos;
        Seq::Wait(step_dt);
    }
    stretch_pos = target_pos;
}

void VacuumClaw::StretchBack()
{
    pitch_pos = pitch_origined_pos + 2000; // 翻转物块，改成2006后好像不能翻的太过了
    // motor_pitch.SetPosVel(-pitch_pos, 1.7);

    // 等物块被翻转过来
    Seq::Wait(2.4f);

    // 收回
    stretch_pos = 100.0; // 防堵转，给个略大于0的值
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
        snap_pos = _snap_origin_code + 130000; // 这是合上爪子后，为了能吸住物块，微微张开爪子让车往前走一小段彻底吸住物块
    else
        APP::monit.LogWarning("Unorigined snapclaw!");   
}

void VacuumClaw::CloseSnap()
{
    if (_snap_origined)
        snap_pos = _snap_origin_code - 102000 + 233040; // 可以刚好定心但不会带动物块
    else
        APP::monit.LogWarning("Unorigined snapclaw!");
}

// 更激进的关爪子
void VacuumClaw::CloseSnap_Hard()
{
    if (_snap_origined)
        snap_pos = _snap_origin_code - 102000 + 272040; // 可以刚好定心但不会带动物块
    else
        APP::monit.LogWarning("Unorigined snapclaw!");
}

void VacuumClaw::PitchOrigin()
{
    pitch_pos = pitch_origined_pos;
}

void VacuumClaw::PitchDown()
{
    pitch_pos = pitch_origined_pos + FETCH_PITCH_POS;
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

void VacuumClaw::_GetPitchOrigin()
{
    static uint32_t runed_tick = 0;  
    static int32_t pitch_probe_code = 0;
    static int32_t pitch_probe_cnt = 0;
    runed_tick++;

    // 对于竖直方向的电机，让他慢慢撞向右边的限位（PosCode增大的方向）
    motor_pitch.SetPos(pitch_probe_code);

    // 如果被小黄看到了，连续5个Tick，就停止
    if (fabs(motor_pitch.motor_adrc.eso.z3) > 1300 && runed_tick > 30)
    {
        pitch_probe_cnt++;
        if (pitch_probe_cnt >= 10)
        {
            // 完成零位校准
            _pitch_origined = true;

            // 记录中点偏移
            _pitch_origin_code = motor_pitch.driver.measure.total_angle;

            // 走到最大位置
            pitch_origined_pos = _pitch_origin_code + 35000;
            pitch_pos = pitch_origined_pos;
            motor_pitch.SetPos(pitch_pos);
        }
    }
    else
    {
        pitch_probe_cnt = 0;
        // 每周期（5ms）加 20 Code，每秒加 4000 Code
        if (runed_tick > 30)    pitch_probe_code -= 280;
        // 但是最开始缓慢前进，目的是破除静态库伦摩擦力
        else                    pitch_probe_code -= 15;
    }
}