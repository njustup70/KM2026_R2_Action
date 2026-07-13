// 取矛头的库
// 爪：减速齿轮齿比1：2，2006减速箱齿比36：1，Pitch向上抬为正
// 看向爪正面，x正方向（水平向右）：正值；y正方向（竖直向上）：正值
#include "Jaw.hpp"
#include "HostPC.hpp"
#include "Monitor.hpp"
#include "spi.h"
#include "bsp_log.hpp"
#include "bsp_dwt.hpp"
#include <cstdio>
#include <cmath>
#include <string.h>
#include "relay.hpp"

Jaw& APP::jaw = Jaw::GetInstance();

static Monitor& monit = Monitor::GetInstance();
static HostPC& host_pc = HostPC::GetInstance();

static float C620VertComp(const FeedforwardInfo& info);
static float C620HoriFriction(const FeedforwardInfo& info);
static float M2006GravityComp(const FeedforwardInfo& info);
int jaw_recv_cnt_test = 0;


void Jaw::_SpearHeadCb(uint8_t func_code, const uint8_t* payload, uint8_t payload_len, void* user_ctx)
{
    (void)func_code;
    // 取矛头误差帧 payload 固定为 8 字节：[left_mm(float), up_mm(float)]
    if (user_ctx == nullptr || payload == nullptr || payload_len < 8)
    {
        return;
    }

    Jaw* jaw = static_cast<Jaw*>(user_ctx);

    // 超时锁定后，不再更新工控机发来的视觉 offset，但光通信 bias 不受影响
    if (jaw->_lock_hostpc_bias) return;

    memcpy(&jaw->_spear_left_err_mm, &payload[0], 4);
    memcpy(&jaw->_spear_up_err_mm, &payload[4], 4);

    jaw->_spear_err_valid = true;

    jaw_recv_cnt_test++;
}

void Jaw::Start()
{
    // 接收取矛头误差帧：SOF=0xFA, Func=0xB1, payload=[left_mm, up_mm]
    MOD::host_pc.Regist(HostPC::FuncCode::dock_err, Jaw::_SpearHeadCb, this);

    motor_3508_hori.Init(Hardware::hcan_main, 1, DJI_C620);  // 水平方向
    motor_3508_vert.Init(Hardware::hcan_main, 2, DJI_C620);  // 竖直方向
    motor_2006.Init(Hardware::hcan_main, 3, DJI_C610);     // Pitch方向

    // ADRC控制
    motor_3508_hori.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(5.4e-4f, 0.30f, 0.005f)
        .ADRC_Limit(10.0f)
        .SpdLimit(8000.0f)
        .ADRC_MaxPlannedVel(8000.0f)
        .ADRC_SOTF(0.42f)
        .Feedforward(C620HoriFriction)
        .Apply();
    motor_3508_hori.driver.Enable();

    motor_3508_vert.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(4.2e-4f, 0.30f, 0.005f)
        .ADRC_Limit(10.0f)
        .SpdLimit(8000.0f)
        .ADRC_MaxPlannedVel(8000.0f)
        .ADRC_SOTF(0.42f)
        .Feedforward(C620VertComp)
        .Apply();
    motor_3508_vert.driver.Enable();

    motor_2006.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(6.9e-5f, 0.18f)
        .ADRC_Limit(8.0f)
        .SpdLimit(5000.0f)
        .ADRC_MaxPlannedVel(5000.0f)
        .ADRC_SOTF(0.1f)
        .Feedforward(M2006GravityComp)
        .Apply();
    motor_2006.driver.Enable();

    //继电器
    Pin relay_pin = {'I', 2};
    relay.Init(relay_pin, Relay::HIGH_ON);
    relay.On();

    _enabled = true;
}

void Jaw::Update()
{
    // 如果退出Debug，全部失能保障安全
    if (System.out_from_debugmode || !_enabled)
    {
        motor_3508_hori.driver.Disable();
        motor_3508_vert.driver.Disable();
        motor_2006.driver.Disable();
        return;
    }

    // 如果尚未初始化，寻找初始化原点
    if (!_origined)
    {
        // 前置条件：如果有任一电机不在线，JAW模块拒绝运作
        if (!motor_2006.driver.online || !motor_3508_hori.driver.online || !motor_3508_vert.driver.online)
        {
            APP::monit.LogError("It's Dangerous to use [Jaw], without motor online!");
            _enabled = false;
            return;
        }
        _Update_GetOrigin();
        return;
    }
    else
    {
        if (_in_track_mode)
        {
            _Track_Head();
            // _Update_LogHead();
        }

        // 真正上传位置到电机
        _UploadPos();
    }
}


void Jaw::_Update_GetOrigin()
{
    // 三个方向上的初始化
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

    // 如果三个方向都已经获取了原点，设置 _origined 标志，并计算随动范围
    if (_hori_origined && _vert_origined && _pitch_origined)
    {
        _origined = true;
        
        // pitch 正式回到近原点处
        _pitch_pos = _pitch_origin_code + 500;

        _hori_tracklim_max = _hori_origin_code - _HoriMMToCode(4);
        _hori_tracklim_min = _hori_origin_code - (JawConst::_hori_max_travel_code - _HoriMMToCode(4));
        _vert_tracklim_max = _vert_origin_code - _VertMMToCode(4);
        _vert_tracklim_min = _vert_origin_code - (JawConst::_vert_max_travel_code - _VertMMToCode(4));
    }
}

void Jaw::_Track_Head()
{
    // 获得串口读取到的位置
    Vec2 err_mm = {-_spear_left_err_mm, -_spear_up_err_mm};
    
    // 加上固定偏置和R1光通信动态偏置
    Vec2 target_mm = err_mm + _arcuo_bias_mm + _optic_dock_bias_mm;

    // 转换为 code形式 位置 （注意，原点在右上，正方向为左下，所以都要取反）
    int target_hori_code = -_HoriMMToCode(target_mm.x) + _hori_origin_code;
    int target_vert_code = -_VertMMToCode(target_mm.y) + _vert_origin_code;

    // 限制在范围内
    if (target_hori_code > _hori_tracklim_max) target_hori_code = _hori_tracklim_max;
    if (target_hori_code < _hori_tracklim_min) target_hori_code = _hori_tracklim_min;
    if (target_vert_code > _vert_tracklim_max) target_vert_code = _vert_tracklim_max;
    if (target_vert_code < _vert_tracklim_min) target_vert_code = _vert_tracklim_min;

    _x_pos = target_hori_code;
    _y_pos = target_vert_code;
}

/**
 * @brief 水平方向 Code 转换为 mm
 * @note 物理含义：水平方向，每圈 8192 code，每圈 上升 2mm
 * @param code 
 * @return float 
 */
float Jaw::_HoriCodeToMM(int code)
{
    return (float)code / 4096.0f;
}
int Jaw::_HoriMMToCode(float mm)
{
    return (int)(mm * 4096.0f);
}

/**
 * @brief 竖直方向 Code 转换为 mm
 * @note 物理含义：竖直方向，每圈 8192 code，每圈 上升 2mm
 * @param code 
 * @return float 
 */
float Jaw::_VertCodeToMM(int code)
{
    return (float)code / 4096.0f;
}
int Jaw::_VertMMToCode(float mm)
{
    return (int)(mm * 4096.0f);
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

void Jaw::_Update_LogHead()
{
    log_head_tick_cnt_++;
    // 从 Update 每 5ms 调用一次，5Hz 即每 200ms (40 ticks) 打印一次
    if (log_head_tick_cnt_ < 40)
    {
        return;
    }
    log_head_tick_cnt_ = 0;

    monit.LogInfo("SpearHead Err: Left: %.2f mm, Up: %.2f mm", _spear_left_err_mm, _spear_up_err_mm);
}

void Jaw::_UploadPos()
{
    // 上传位置到电机
    motor_3508_hori.SetPos(_x_pos);
    motor_3508_vert.SetPos(_y_pos);
    motor_2006.SetPos(_pitch_pos);
}

void Jaw::Enable()
{
    // _enabled = true;
}   

void Jaw::Disable()
{
    // _enabled = false;
}

void Jaw::OpenClaw()
{
    relay.On();
}

void Jaw::CloseClaw()
{
    relay.Off();
}

void Jaw::PitchUp()
{
    // 减的这 500 是防堵转
    _pitch_pos = _pitch_origin_code + 500;
}

void Jaw::SoftPitchUp()
{
    float targ_pitch_pos = _pitch_origin_code + 500;
    constexpr float step_size = 6000;
    constexpr float step_dt = 0.005;
    for(float pos = _pitch_pos + step_size; pos < targ_pitch_pos; pos += step_size)
    {
        _pitch_pos = pos;
        Seq::Wait(step_dt);
    }
}


void Jaw::PitchDown()
{
    _pitch_pos = _pitch_origin_code + (JawConst::_pitch_max_travel_code / 2) - 3500;
}

void Jaw::SetPos(float x_mm, float y_mm)
{
    if (!_origined) return;

    int hori_center = _hori_origin_code - JawConst::_hori_max_travel_code / 2;
    int vert_center = _vert_origin_code - JawConst::_vert_max_travel_code / 2;

    int target_hori = hori_center + _HoriMMToCode(x_mm);
    int target_vert = vert_center + _VertMMToCode(y_mm);

    // 限幅后修改
    int hori_min = _hori_origin_code - JawConst::_hori_max_travel_code;
    int hori_max = _hori_origin_code;
    if (target_hori > hori_max) target_hori = hori_max;
    if (target_hori < hori_min) target_hori = hori_min;

    int vert_min = _vert_origin_code - JawConst::_vert_max_travel_code;
    int vert_max = _vert_origin_code;
    if (target_vert > vert_max) target_vert = vert_max;
    if (target_vert < vert_min) target_vert = vert_min;

    _x_pos = target_hori;
    _y_pos = target_vert;
}

void Jaw::SetPosCode(int x_code, int y_code)
{
    if (!_origined) return;

    int hori_min = _hori_origin_code - JawConst::_hori_max_travel_code;
    int hori_max = _hori_origin_code;
    if (x_code > hori_max) x_code = hori_max;
    if (x_code < hori_min) x_code = hori_min;

    int vert_min = _vert_origin_code - JawConst::_vert_max_travel_code;
    int vert_max = _vert_origin_code;
    if (y_code > vert_max) y_code = vert_max;
    if (y_code < vert_min) y_code = vert_min;

    _x_pos = x_code;
    _y_pos = y_code;
}

void Jaw::SetXPos(float x_mm)
{
    if (!_origined) return;

    int hori_center = _hori_origin_code - JawConst::_hori_max_travel_code / 2;
    int target_hori = hori_center + _HoriMMToCode(x_mm);

    // 只约束 x 方向，y 方向保持当前值不变
    int hori_min = _hori_origin_code - JawConst::_hori_max_travel_code;
    int hori_max = _hori_origin_code;
    if (target_hori > hori_max) target_hori = hori_max;
    if (target_hori < hori_min) target_hori = hori_min;

    _x_pos = target_hori;
}

void Jaw::AddOpticDockBias(float delta_x_mm, float delta_y_mm)
{
    _optic_dock_bias_mm.x += delta_x_mm;
    _optic_dock_bias_mm.y += delta_y_mm;
}

void Jaw::ChangePos(float delta_x_mm, float delta_y_mm)
{
    if (!_origined) return;

    _x_pos += _HoriMMToCode(delta_x_mm);
    _y_pos += _VertMMToCode(delta_y_mm);

    // 限幅后修改
    int hori_min = _hori_origin_code - JawConst::_hori_max_travel_code;
    int hori_max = _hori_origin_code;
    if (_x_pos > hori_max) _x_pos = hori_max;
    if (_x_pos < hori_min) _x_pos = hori_min;

    int vert_min = _vert_origin_code - JawConst::_vert_max_travel_code + 4000; // 给个 4000 不堵转
    int vert_max = _vert_origin_code;
    if (_y_pos > vert_max) _y_pos = vert_max;
    if (_y_pos < vert_min) _y_pos = vert_min;
}

// void Jaw::Dock()
// {

// }


/**---------------------------------    调零和前馈    ----------------------------------**/
void Jaw::_GetHoriOrigin()
{
    static uint32_t runed_tick = 0;  
    static uint32_t hori_probe_code = 20;
    static uint8_t hori_probe_cnt = 0;
    runed_tick++;

    // monit.LogInfo("runtick:%d", runed_tick);
    // 对于水平方向的电机，让他慢慢撞向右边的限位（PosCode增大的方向）
    motor_3508_hori.SetPos(hori_probe_code);

    // 如果撞到限位了（用eso的扰动观测来确定），连续5个Tick，就停止
    if (fabs(motor_3508_hori.motor_adrc.eso.z3) > 1300 && runed_tick > 60)
    {
        hori_probe_cnt++;
        if (hori_probe_cnt >= 10)
        {
            // 完成零位校准
            _hori_origined = true;
            // 记录中点偏移
            _hori_origin_code = motor_3508_hori.driver.measure.total_angle;
            // 往回走到中点
            _x_pos = _hori_origin_code - JawConst::_hori_max_travel_code / 2;
            motor_3508_hori.SetPos(_x_pos);
        }
    }
    else
    {
        hori_probe_cnt = 0;
        // 每周期（5ms）加 200 Code，每秒加 40000 Code
        if (runed_tick > 60)    hori_probe_code += 200;
        // 但是最开始缓慢前进，目的是破除静态库伦摩擦力
        else if (runed_tick > 30) hori_probe_code += 65;
        else                    hori_probe_code += 20;
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
    if (fabs(motor_3508_vert.motor_adrc.eso.z3) > 1300 && runed_tick > 60)
    {
        vert_probe_cnt++;
        if (vert_probe_cnt >= 10)
        {
            // 完成零位校准
            _vert_origined = true;
            // 记录中点偏移
            _vert_origin_code = motor_3508_vert.driver.measure.total_angle;
            // 往回走到三分之一
            _y_pos = _vert_origin_code - 200000; //后面加的是夹到矛头最好的位置偏移
            motor_3508_vert.SetPos(_y_pos);
        }
    }
    else
    {
        vert_probe_cnt = 0;
        // 每周期（5ms）加 200 Code，每秒加 40000 Code
        if (runed_tick > 60)    vert_probe_code += 200;
        // 但是最开始缓慢前进，目的是破除静态库伦摩擦力
        else if (runed_tick > 30) vert_probe_code += 65;
        else                    vert_probe_code += 20;
    }   
}

void Jaw::_GetPitchOrigin()
{
    static uint32_t runed_tick = 0;  
    static int32_t pitch_probe_code = 0;
    static uint8_t pitch_probe_cnt = 0;
    runed_tick++;

    // 对于竖直方向的电机，让他慢慢撞向右边的限位（PosCode增大的方向）
    motor_2006.SetPos(pitch_probe_code);

    // 如果撞到限位了（用eso的扰动观测来确定），连续5个Tick，就停止
    if (fabs(motor_2006.motor_adrc.eso.z3) > 2800 && runed_tick > 30)
    {
        pitch_probe_cnt++;
        if (pitch_probe_cnt >= 10)
        {
            // 完成零位校准
            _pitch_origined = true;
            // 记录中点偏移
            _pitch_origin_code = motor_2006.driver.measure.total_angle;
            // 往回一点点，防止卡住影响其他轴
            _pitch_pos = _pitch_origin_code + 10000;
            motor_2006.SetPos(_pitch_pos);
        }
    }
    else
    {
        pitch_probe_cnt = 0;
        // 每周期（5ms）加 20 Code，每秒加 4000 Code
        if (runed_tick > 30)    pitch_probe_code -= 80;
        // 但是最开始缓慢前进，目的是破除静态库伦摩擦力
        else                    pitch_probe_code -= 15;
    }   
}

/**---------------------------    前馈函数    ----------------------------------**/

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
    float rod_angle = code_diff / 143000.0f * (3.14159265f / 2.0f);
    
    // 当 z3 = 1000 时，系统所需的抵消电流 i = - (1000 / b0) = -0.3778 A
    constexpr float kMaxCompCurrent = -0.3f * 0.9f; 
    
    // 返回前馈电流，重力矩近似等于 最大幅值 * sin(物理小角度)
    return kMaxCompCurrent * std::sin(rod_angle);
}

static float C620HoriFriction(const FeedforwardInfo& info)
{
    constexpr float kFricTorque = 0.33f;   // 动摩擦力矩 N*m
    constexpr float kKt = 0.30f;          // C620 (M3508) Kt
    constexpr float kVelEpsMin = 1.0f;    // rad/s，低于该值不做摩擦补偿
    constexpr float kVelEpsMax = 10.0f;    // rad/s，高于该值给到满补偿
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
