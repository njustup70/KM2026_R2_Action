// 取矛头的库
// 爪：减速齿轮齿比1：2，2006减速箱齿比36：1，给负值爪顺时针转
// 看向爪正面，x正方向（水平向右）：正值；y正方向（竖直向上）：正值
#include "Jaw.hpp"
#include "Monitor.hpp"
#include "spi.h"
#include "bsp_log.hpp"
#include <cstdio>
#include <cmath>

static float M2006GravityComp(const FeedforwardInfo& info)
{
    // 获取运行时测出的竖直原点 code
    int origin_code = Jaw::GetInstance().GetPitchOriginCode();

    // 从 info.measured (rad) 推算当前的 total_angle (code)
    // 根据 MotorDJI 的逻辑： measured = (total_angle / 8192.0f) * 2 * PI
    float current_code = info.measured / (2.0f * 3.14159265f) * 8192.0f;

    // 获取相对于原点的 code 偏差
    // 竖直位置： current_code == origin_code => 偏差 0
    // 水平位置： current_code == origin_code - 143000 => 偏差 -143000
    float code_diff = current_code - origin_code;

    // 映射到杆的实际角度（0 -> 竖直，-143000 -> 水平(PI/2)）
    float rod_angle = code_diff / -143000.0f * (3.14159265f / 2.0f);
    
    // 当 z3 = 1000 时，系统所需的抵消电流 i = - (1000 / b0) = -0.3778 A
    constexpr float kMaxCompCurrent = -0.3778f * 0.9f; 
    
    // 返回前馈电流，重力矩近似等于 最大幅值 * sin(物理小角度)
    return kMaxCompCurrent * std::sin(rod_angle);
}

static float C620HoriFriction(const FeedforwardInfo& info)
{
    constexpr float kFricTorque = 0.33f;   // 动摩擦力矩 N*m
    constexpr float kKt = 0.30f;          // C620 (M3508) Kt
    constexpr float kVelEpsMin = 0.5f;    // rad/s，低于该值不做摩擦补偿
    constexpr float kVelEpsMax = 5.0f;    // rad/s，高于该值给到满补偿
    constexpr float kTargetCurrent = kFricTorque / kKt;

    if (info.velocity > kVelEpsMax)
        return kTargetCurrent;
    if (info.velocity > kVelEpsMin)
    {
        float ratio = (info.velocity - kVelEpsMin) / (kVelEpsMax - kVelEpsMin);
        return kTargetCurrent * ratio;
    }

    if (info.velocity < -kVelEpsMax)
        return -kTargetCurrent;
    if (info.velocity < -kVelEpsMin)
    {
        float ratio = (-info.velocity - kVelEpsMin) / (kVelEpsMax - kVelEpsMin);
        return -kTargetCurrent * ratio;
    }

    return 0.0f;
}

static float C620VertComp(const FeedforwardInfo& info)
{
    // 0.2A 的恒定重力补偿（向上为正）
    float gravity_part = 1.0f;

    constexpr float kFricTorque = 0.32f;   // 动摩擦力矩 N*m
    constexpr float kKt = 0.30f;          // C620 (M3508) Kt
    constexpr float kVelEpsMin = 0.5f;    // rad/s
    constexpr float kVelEpsMax = 5.0f;    // rad/s
    constexpr float kTargetCurrent = kFricTorque / kKt;

    float friction_part = 0.0f;
    if (info.velocity > kVelEpsMax)
        friction_part = kTargetCurrent;
    else if (info.velocity > kVelEpsMin)
    {
        float ratio = (info.velocity - kVelEpsMin) / (kVelEpsMax - kVelEpsMin);
        friction_part = kTargetCurrent * ratio;
    }
    else if (info.velocity < -kVelEpsMax)
        friction_part = -kTargetCurrent;
    else if (info.velocity < -kVelEpsMin)
    {
        float ratio = (-info.velocity - kVelEpsMin) / (kVelEpsMax - kVelEpsMin);
        friction_part = -kTargetCurrent * ratio;
    }

    return gravity_part + friction_part;
}

void Jaw::Start()
{
    motor_3508_hori.Init(Hardware::hcan_main, 1, DJI_C620);  // 水平方向
    motor_3508_vert.Init(Hardware::hcan_main, 2, DJI_C620);  // 竖直方向
    motor_2006.Init(Hardware::hcan_main, 3, DJI_C610);     // Pitch方向

    // ADRC控制
    motor_3508_hori.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(4.9e-4f, 0.30f, 0.005f)
        .ADRC_Limit(15.0f)
        .SpdLimit(1500.0f)
        .ADRC_MaxPlannedVel(1500.0f)
        .ADRC_SOTF(0.33f)
        .Feedforward(C620HoriFriction)
        .Apply();
    motor_3508_hori.driver.Enable();

    motor_3508_vert.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(4.2e-4f, 0.30f, 0.005f)
        .ADRC_Limit(15.0f)
        .SpdLimit(1500.0f)
        .ADRC_MaxPlannedVel(1500.0f)
        .ADRC_SOTF(0.33f)
        .Feedforward(C620VertComp)
        .Apply();
    motor_3508_vert.driver.Enable();

    motor_2006.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(7.5e-5f, 0.18f)
        .ADRC_Limit(9.0f)
        .SpdLimit(5000.0f)
        .ADRC_MaxPlannedVel(5000.0f)
        .ADRC_SOTF(0.33f)
        .Feedforward(M2006GravityComp)
        .Apply();
    motor_2006.driver.Enable();

    //继电器
    Pin relay_pin = {'F', 1};
    relay.Init(relay_pin, Relay::HIGH_ON);
}

void Jaw::Update()
{
    if(System.out_from_debugmode)
    {
        motor_3508_hori.Neutral();
        motor_3508_vert.Neutral();
        motor_2006.Neutral();
        return;
    }
    
    if (!_origined)
    {
        if (!_hori_origined)
        {
            _GetHoriOrigin();
        }
        if (!_vert_origined)
        {
            _GetVertOrigin();
        }
        if (!_pitch_origined)
        {
            _GetPitchOrigin();
        }

        if (_hori_origined && _vert_origined && _pitch_origined)
        {
            _origined = true;
        }
        return;
    }
    else
    {
        _UploadPos();
		// _Update_Log(); //没有检测开了日志系统会不会卡死，反正我没log_router，不注释程序会飞
        _SetJawState(_state);
    }
}

void Jaw::_Update_Log()
{
    log_tick_cnt_++;
    // 每 50 个 tick 打印一次数据
    if (log_tick_cnt_ < 50)
    {
        return;
    }
    log_tick_cnt_ = 0;

    if (log_header_cnt_ >= 8)
    {
        log_header_cnt_ = 0;
        monit.LogInfo("--- Jaw Position Feedback Log ---");
        monit.LogInfo("%-18s | %-18s | %-18s", "Motor0 X(Pos/On)", "Motor1 Y(Pos/On)", "Motor2 P(Pos/On)");
    }
    log_header_cnt_++;

    auto get_motor_str = [](MotorDJI& m, char* buf)
    {
        sprintf(buf, "%7d %s", (int)m.driver.measure.total_angle, m.driver.online ? "ON " : "OFF");
    };

    char motor0_buf[20], motor1_buf[20], motor2_buf[20];
    get_motor_str(motor_3508_hori, motor0_buf);
    get_motor_str(motor_3508_vert, motor1_buf);
    get_motor_str(motor_2006, motor2_buf);

    monit.LogInfo("%-18s | %-18s | %-18s", motor0_buf, motor1_buf, motor2_buf);
}

void Jaw::_UploadPos()
{

}

/**----------------------    调零    ----------------------**/
void Jaw::_GetHoriOrigin()
{
    static uint32_t runed_tick = 0;  
    static uint32_t hori_probe_code = 20;
    static uint8_t hori_probe_cnt = 0;
    runed_tick++;

    // 对于水平方向的电机，让他慢慢撞向右边的限位（PosCode增大的方向）
    motor_3508_hori.SetPos(hori_probe_code);

    // 如果撞到限位了（用eso的扰动观测来确定），连续5个Tick，就停止
    if (fabs(motor_3508_hori.motor_adrc.eso.z3) > 1000 && runed_tick > 40)
    {
        hori_probe_cnt++;
        if (hori_probe_cnt >= 10)
        {
            // 完成零位校准
            _hori_origined = true;
            // 记录中点偏移
            _hori_origin_code = motor_3508_hori.driver.measure.total_angle;
            // 往回走到中点
            motor_3508_hori.SetPos(_hori_origin_code - 70000);
        }
    }
    else
    {
        hori_probe_cnt = 0;
        // 每周期（5ms）加 200 Code，每秒加 40000 Code
        if (runed_tick > 40)    hori_probe_code += 200;
        // 但是最开始缓慢前进，目的是破除静态库伦摩擦力
        else                    hori_probe_code += 15;
    }   
}

void Jaw::_GetVertOrigin()
{
    static uint32_t runed_tick = 0;  
    static uint32_t vert_probe_code = 20;
    static uint8_t vert_probe_cnt = 0;
    runed_tick++;

    // 对于竖直方向的电机，让他慢慢撞向右边的限位（PosCode增大的方向）
    motor_3508_vert.SetPos(vert_probe_code);

    // 如果撞到限位了（用eso的扰动观测来确定），连续5个Tick，就停止
    if (fabs(motor_3508_vert.motor_adrc.eso.z3) > 1000 && runed_tick > 60)
    {
        vert_probe_cnt++;
        if (vert_probe_cnt >= 10)
        {
            // 完成零位校准
            _vert_origined = true;
            // 记录中点偏移
            _vert_origin_code = motor_3508_vert.driver.measure.total_angle;
            // 往回走到中点
            motor_3508_vert.SetPos(_vert_origin_code - 140000);
        }
    }
    else
    {
        vert_probe_cnt = 0;
        // 每周期（5ms）加 200 Code，每秒加 40000 Code
        if (runed_tick > 60)    vert_probe_code += 200;
        // 但是最开始缓慢前进，目的是破除静态库伦摩擦力
        else                    vert_probe_code += 15;
    }   
}

void Jaw::_GetPitchOrigin()
{
    static uint32_t runed_tick = 0;  
    static uint32_t pitch_probe_code = 0;
    static uint8_t pitch_probe_cnt = 0;
    runed_tick++;

    // 对于竖直方向的电机，让他慢慢撞向右边的限位（PosCode增大的方向）
    motor_2006.SetPos(pitch_probe_code);

    // 如果撞到限位了（用eso的扰动观测来确定），连续5个Tick，就停止
    if (fabs(motor_2006.motor_adrc.eso.z3) > 1300 && runed_tick > 30)
    {
        pitch_probe_cnt++;
        if (pitch_probe_cnt >= 10)
        {
            // 完成零位校准
            _pitch_origined = true;
            // 记录中点偏移
            _pitch_origin_code = motor_2006.driver.measure.total_angle;
            // 往回走到竖直
            motor_2006.SetPos(_pitch_origin_code - 400);
        }
    }
    else
    {
        pitch_probe_cnt = 0;
        // 每周期（5ms）加 20 Code，每秒加 4000 Code
        if (runed_tick > 30)    pitch_probe_code += 50;
        // 但是最开始缓慢前进，目的是破除静态库伦摩擦力
        else                    pitch_probe_code += 15;
    }   
}

void Jaw::_SetJawState(JawState state)
{
    switch (state)
    {
        // 气路是这么接的
        case JawState::OPEN:
            relay.On();  // 打开夹爪
            break;
        case JawState::CLOSE:
            relay.Off(); // 关闭夹爪
            break;
    }
}

void Jaw::Open()
{
    _state = JawState::OPEN;
}

void Jaw::Close()
{
    _state = JawState::CLOSE;
}