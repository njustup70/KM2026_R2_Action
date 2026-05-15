#include "Chassis.hpp"
#include "arm_math.h"
#include "Monitor.hpp"
#include "farcon.hpp"
#include "Interboardcomm.hpp"
#include "HostPC.hpp"
#include "System.hpp"
#include "bsp_log.hpp"
#include "MainFrame.hpp"
#include <cmath>


static HostPC& host_pc = HostPC::GetInstance();

// BoardComm main_board;

void ChassisType::ChasTaskCallback(uint8_t task_id, const uint8_t *payload, uint8_t payload_len, void *user_ctx)
{
    // 这里处理C板发来的自解里程计数据,0x221
    static int send_cnt = 0; // 发送计数器，板间通讯是200hz，算法组要求100hz

    // GetInstance().chas_odom.pos.x = ((int16_t)(payload[0] << 8 | payload[1])) / 1000.0f; 
    // GetInstance().chas_odom.pos.y = ((int16_t)(payload[2] << 8 | payload[3])) / 1000.0f; 
    // GetInstance().chas_odom.pos.z = ((int16_t)(payload[4] << 8 | payload[5])) / 1000.0f; 

    GetInstance().chas_odom.pos.x = UnpackInt16ToFloat(&payload[0], 1000.0f);
    GetInstance().chas_odom.pos.y = UnpackInt16ToFloat(&payload[2], 1000.0f);
    GetInstance().chas_odom.pos.z = UnpackInt16ToFloat(&payload[4], 1000.0f);

    if(++send_cnt >= 2) 
    {
        send_cnt = 0;
        host_pc.SendOdom(GetInstance().chas_odom.pos.x, GetInstance().chas_odom.pos.y, GetInstance().chas_odom.pos.z);
    }
}

void ChassisType::Start()
{
    // // 初始化电机
    // for (int i = 0; i < 4; i++)
    // {
    //     // motors[i].Init(Hardware::hcan_main, i + 1, MotorDJIMode::PID_SpeedControl);
    //     // motors[i].speed_pid.Init(3.6, 2.4, 0.0);
    //     // motors[i].speed_pid.ForwardLize(PidGeneral::SpeedForward, 1.5f, 5, 4.8); 			// 速度型前馈
    //     // motors[i].Enable();
    // }
    
    main_board.RegisterTask(1, ChasTaskCallback, this);
    host_pc.Regist(0xFA, 0xBB, HostCmdCallback, this);
    

    uint8_t payload[1] = {0x01}; // 发送重置底盘和腿的指令，包括电机和里程计
    main_board.SendTask(0x210, 4, payload, 1, false); 
}

void SendChasSpeedCmd(float vx, float vy, float vz)
{
    // 6 字节 uint8，完全匹配接收端格式
    uint8_t payload[6];
    const float scale = 1000.0f;
    // int16_t x = (int16_t)(vx * 1000.0f);
    // int16_t y = (int16_t)(vy * 1000.0f);
    // int16_t z = (int16_t)(vz * 1000.0f);

    // // 大端格式：高字节 <<8 在前，低字节在后
    // // 对应接收端：payload[0] <<8 | payload[1]
    // payload[0] = (x >> 8) & 0xFF; // Vx 高8位
    // payload[1] = x & 0xFF;        // Vx 低8位

    // payload[2] = (y >> 8) & 0xFF; // Vy 高8位
    // payload[3] = y & 0xFF;        // Vy 低8位

    // payload[4] = (z >> 8) & 0xFF; // Vz 高8位
    // payload[5] = z & 0xFF;        // Vz 低8位
    PackFloatToInt16(&payload[0], vx, scale);
    PackFloatToInt16(&payload[2], vy, scale);
    PackFloatToInt16(&payload[4], vz, scale);
    main_board.SendTask(0x210, 1, payload, 6, false);
}



void SendChasTargGes(float x, float y, float z)
{
    uint8_t payload[6];
    const float scale = 1000.0f;
    PackFloatToInt16(&payload[0], x, scale);
    PackFloatToInt16(&payload[2], y, scale);
    PackFloatToInt16(&payload[4], z, scale);

    // int16_t x_int = (int16_t)(x * 1000.0f);
    // int16_t y_int = (int16_t)(y * 1000.0f);
    // int16_t z_int = (int16_t)(z * 1000.0f);

    // payload[0] = (x_int >> 8) & 0xFF;
    // payload[1] = x_int & 0xFF;

    // payload[2] = (y_int >> 8) & 0xFF;
    // payload[3] = y_int & 0xFF;

    // payload[4] = (z_int >> 8) & 0xFF;
    // payload[5] = z_int & 0xFF;

    main_board.SendTask(0x210, 5, payload, sizeof(payload), false);
}

void ChassisType::_SetCtrlMode()
{
    if (farcon.toggle[0] == 1)
    {
        control_mode = (farcon.toggle[1] == 1) ? HOST : FARCON;
        _host_ctrl = (farcon.toggle[1] == 1) ? true : false;
    }     
    else
	{
        control_mode = (farcon.toggle[1] == 1) ? API : OPEN;
        _host_ctrl = false;
	}
}

void ChassisType::_HandleSpeedCmd()
{
    switch(control_mode)
    {
        case OPEN:
        break;

        case FARCON:
            // 读取遥控器数据到底盘控制变量
            targ_speed.x = -farcon.jys_value[3] * 1.0f / 100.f * _max_velo;  // 前后
            targ_speed.y = -farcon.jys_value[2] * 1.0f / 100.f * _max_velo;  // 左右
            targ_speed.z = -farcon.jys_value[0] * 1.0f / 100.f * _max_omega; // 旋转        
        break;

        case API:
        break;

        case HOST:
            _host_ctrl = true;
        break;

        default:
        break;      
    }
    SendChasSpeedCmd(targ_speed.x, targ_speed.y, targ_speed.z);
}

// 可以在walking时出现意外时打断自动，手动控制
void ChassisType::_JudgeJoyAct()
{
    // 检查摇杆是否被推动
    float joystick_deadband = 10.0f; 
    bool joystick_active = (fabs(farcon.jys_value[0]) > joystick_deadband || 
                            fabs(farcon.jys_value[2]) > joystick_deadband || 
                            fabs(farcon.jys_value[3]) > joystick_deadband);
    
    if (!joystick_active) 
    {
        // 遥控器没动，此时不发送速度指令给C板，让 _Walking() 的指令生效
        return; 
    } 
    else 
    {
        // 摇杆动了，打断位置环
        _walking = false;
        _rotating = false;
        control_mode = FARCON;
    }
}

void ChassisType::Update()
{
    if(System.out_from_debugmode)
    {
        uint8_t payload[1] = {0x01}; // 发送重置底盘和腿的指令，包括电机和里程计
        main_board.SendTask(0x210, 3, payload, 1, false); // 发送停止指令
    }
    
    // 管理发送，确保每次只有一个接口发送指令
    Vec3 final_cmd(0, 0, 0);

    // 设置控制模式
    _SetCtrlMode();
    // control_mode = FARCON; // 调试

    if(_walking)
    {
        // 判断有没有摇杆输入，摇杆输入打断自动走位
        _JudgeJoyAct();
        // 发姿态
        SendChasTargGes(targ_ges.x, targ_ges.y, targ_ges.z);
    }
    else
    {
        // 发速度
        _HandleSpeedCmd();   
    }

    targ_velo = targ_speed.Length();

    // // 遥控器控制逻辑

    // 实现闭环的地方，A板在这里去掉，能理解我的恭喜到达阴间尽头
    // if (_walking || _is_pos_locked)
    // {
    //     _Walking();
    //     // SendChasTargGes(targ_ges.x, targ_ges.y, targ_ges.z);
    // }
    // if (_rotating || _is_yaw_locked)
    // {
    //     _Rotating();
    // }

    // // 将底盘的 速度targ_speed 上传到各个电机
    // _UploadSpeed();

    // // 更新自解算里程计
    // _UpdateChasOdom();

    // targ_velo = targ_speed.Length();

    // // 安全锁倒计时
    // _safe_lock_tick -= 5;
}

void ChassisType::_UpdateChasOdom()
{
    // （1）获得当前角度
    float theta_distan = 0; // 单位：米
    for (int i = 0; i < 4; i++)
    {
        theta_distan += motors[i].driver.measure.total_angle;
    }
    theta_distan = theta_distan / (MotorDJIConst::redu_M3508 * 8192) * (PI * WHEEL_DIAMETER) / 4.0f;
    float chas_theta = theta_distan / ROTATE_RADIUS; // 单位：弧度

    // （2）获取车体速度(读取而不是控制的速度，以减少误差)
    Vec3 chas_speed; // 都是线速度
    // 旋转分量
    for (int i = 0; i < 4; i++)
    {
        chas_speed.z += motors[i].driver.measure.speed_rpm;
    }
    chas_speed.z = (chas_speed.z / 240.0f) / (MotorDJIConst::redu_M3508) * (PI * WHEEL_DIAMETER);

    // 获得每个电机不带旋转速度的线速度分量（用于计算x, y方向上的速度）
    float motor_spd_xy[4] = {0};
    for (int i = 0; i < 4; i++)
    {
        motor_spd_xy[i] = (motors[i].driver.measure.speed_rpm / 60.0f / MotorDJIConst::redu_M3508) * (PI * WHEEL_DIAMETER) - chas_speed.z;
    }

    Vec2 chas_vxy;
    chas_vxy.x = (motor_spd_xy[1] - motor_spd_xy[2]) / 2.0f;
    chas_vxy.y = (motor_spd_xy[0] - motor_spd_xy[3]) / 2.0f;

    // （3）更新里程计，还有速率
    Vec2 delta_move = chas_vxy.Rotate(chas_theta + (PI / 4)) / 200.0f;
    chas_odom.velocity = chas_vxy.Length();

    chas_odom.speed = (delta_move * 200.0f).ToVec3();
    chas_odom.speed.z = chas_speed.z;

    chas_odom.pos = chas_odom.pos + delta_move.ToVec3();
    chas_odom.pos.z = chas_theta;
}

void ChassisType::_UploadSpeed()
{
    static float runtime_cnt = 0;
    // 仅当底盘使能时才工作
    if (enabled && _safe_lock_tick > 0)
    {
        runtime_cnt = 0;
        _SendSpdToMotor();
    }
    else
    {
        // 底盘未使能，分两种情况
        // (1) 底盘仍有速度
        if (chas_odom.speed.Length() > 0.15f)
        {
            if (runtime_cnt < 0.001f)
                runtime_cnt = System.runtime_tick;

            // (1.1) 仍有速度，先刹车停
            if (System.runtime_tick - runtime_cnt < 1.0f)
            {
                // 否定其他接口的控制权，并进行刹车
                targ_speed = targ_speed * 0.96f;
                _SendSpdToMotor();
            }
            else // (1.2) 1s还停不下来，强制进入空档
            {
                for (int i = 0; i < 4; i++)
                {
                targ_speed = Vec3(0, 0, 0);
                motors[i].Neutral();
                }
            }
        }
        else // (2) 底盘已经停止，直接进入空档
        {
            for (int i = 0; i < 4; i++)
            {
                targ_speed = Vec3(0, 0, 0);
                motors[i].Neutral();
            }
        }
    }
}

inline void ChassisType::_SendSpdToMotor()
{
    // 计算x, y, w合成分量
    _motor_spd[0] = (-targ_speed.x + targ_speed.y) / (BSP_SQRT2) + targ_speed.z * ROTATE_RADIUS;
    _motor_spd[1] = (targ_speed.x + targ_speed.y) / (BSP_SQRT2) + targ_speed.z * ROTATE_RADIUS;
    _motor_spd[2] = (-targ_speed.x - targ_speed.y) / (BSP_SQRT2) + targ_speed.z * ROTATE_RADIUS;
    _motor_spd[3] = (targ_speed.x - targ_speed.y) / (BSP_SQRT2) + targ_speed.z * ROTATE_RADIUS;

    // 发送速度指令到电机
    for (int i = 0; i < 4; i++)
    {
        // motors[i].SwitchMode(MotorDJIMode::PID_SpeedControl);
        motors[i].SetSpd((_motor_spd[i] * 60.0f) / (PI * WHEEL_DIAMETER));
    }
}

void ChassisType::Enable()
{
    enabled = true;
}

void ChassisType::Disable()
{
    // 停止所有电机
    enabled = false;
}

void ChassisType::MoveAt(Vec2 Pos)
{
    targ_ges = Vec3(Pos.x, Pos.y, targ_ges.z);
    _walking = true;
    // SendChasTargGes(Pos.x, Pos.y, targ_ges.z);
}

void ChassisType::RotateAt(float yaw)
{
    targ_ges.z = yaw;
    _rotating = true;
}

/**
 * @brief 直接设置底盘速度（一个通用的开环行为）
 * @param Spd 期望速度：（x: 前向速度，y：左向速度，w：逆时针）（m/s，m/s，rad/s）
 * @note 轮序：
 *                      前
 *                  0       1
 *
 *
 *                  2       3
 * @warning 每次设置速度都会刷新安全锁，需要持续调用以保持底盘运动。
 * 在指令中断100ms后，底盘会自动进入空档（0电流）状态。
 * （100ms已经很长了，相当于20个指令周期都没有指令输入）
 */
void ChassisType::Move(Vec3 Spd)
{
    if (!enabled)
        return; // 开放控制未使能，直接返回

    _safe_lock_tick = 100;
    targ_speed = Spd;
}

void ChassisType::Move(Vec2 Spd)
{
    if (!enabled)
        return;

    _safe_lock_tick = 100;

    targ_speed.x = Spd.x;
    targ_speed.y = Spd.y;
}

void ChassisType::Rotate(float omega)
{
    if (!enabled)
        return;

    // 验证输入安全
    if (!std::isfinite(omega))
    {
        Monitor::GetInstance().LogError("Chassis: Dangerous omega!");
        return;
    }

    // 输入合法化
    if (omega > _max_omega)
    {
        omega = _max_omega;
    }
    else if (omega < -_max_omega)
    {
        omega = -_max_omega;
    }

    _safe_lock_tick = 100;
    targ_speed.z = omega;
}

/**
 * @brief 基于直接移动到位置的方式
 * @details 被 Update 调用
 * 需要用到的信息流：底盘的控制、当前的位置、底盘的速度
 * @warning 只管xy的姿态，不管yaw角
 */
bool ChassisType::_Walking()
{
    // 计算移动向量
    Vec2 move_vec = targ_ges.ToVec2() - System.position.ToVec2();
    // 带入车体旋转
    move_vec = move_vec.Rotate(-System.position.z);

    // 检查是否到达目标位置, 如果是则返回完成
    if (move_vec.Length() < 0.05f) // 5cm范围内视为到达
    {
        Move(Vec2(0, 0)); // 停止移动
        _walking = false;
        return true; // 动作完成
    }

    // 计算移动速度
    float safe_velo = sqrt(2 * _max_accel * move_vec.Length());
    float out_velo = 3.0f * move_vec.Length();
    // 最终的速度应该为三者中的最小值
    float final_velo = fminf(safe_velo, fminf(out_velo, _max_velo));

    // 更新底盘速度（向量式更新，保证更新量不大于MaxAccel）
    Vec2 targ_speed_vec = move_vec.Norm() * final_velo; // 计算新的目标速度
    Vec2 curr_speed_vec = targ_speed.ToVec2();          // 当前速度

    // 计算速度差
    Vec2 delta_speed_vec = targ_speed_vec - curr_speed_vec;
    float delta_speed_len = delta_speed_vec.Length();

    // 限制加速度（向量长度自带绝对值）
    if (delta_speed_len > (_max_accel / 200.0f)) // 每次调用都是1 / 200s
    {
        delta_speed_vec = delta_speed_vec.Norm() * (_max_accel / 200.0f);
    }
    Vec2 new_speed_vec = curr_speed_vec + delta_speed_vec;

    // 调用移动接口进行移动
    Move(new_speed_vec);

    return false;
}

/**
 * @brief 不断尝试旋转到某个角度
 * @details 被 Update 调用
 * 需要用到的信息流：底盘的控制、当前的位置、底盘的速度
 * @warning 只管yaw角，不管xy的姿态
 */
bool ChassisType::_Rotating()
{
    // 计算旋转向量 （速度Rad / s)
    float rotate_diff = (targ_ges.z - System.position.z);

    // 检查是否到达目标位置, 如果是则返回完成
    if (fabs(rotate_diff) < 0.01f) // 0.01rad范围内视为到达
    {
        Rotate(0); // 停止
        _rotating = false;
        return true; // 动作完成
    }

    // 计算旋转速度 （注意绝对值）
    float safe_omega = sqrt(2 * _max_beta * fabs(rotate_diff));
    float out_omega = 3.0f * fabs(rotate_diff);

    // 最终的速度应该为三者中的最小值
    float targ_omega = fminf(safe_omega, fminf(out_omega, _max_omega));

    // 更新底盘速度（向量式更新，保证更新量不大于MaxAccel）
    // 计算速度差
    float delta_speed = targ_omega * (rotate_diff > 0 ? 1 : -1) - targ_speed.z;

    // 限制加速度（注意绝对值）
    if (fabs(delta_speed) > (_max_beta / 200.0f)) // 每次调用都是1 / 200s
    {
        delta_speed = (delta_speed > 0 ? 1 : -1) * (_max_beta / 200.0f);
    }

    float new_omega = targ_speed.z + delta_speed;

    // 调用旋转接口进行移动
    Rotate(new_omega);

    return false;
}



void ChassisType::HostCmdCallback(uint8_t func_code, const uint8_t* payload, uint8_t payload_len, void* user_ctx)
{
    if(func_code == 0xBB && GetInstance()._host_ctrl)
    {
        Vec3 speed;
        speed.FromBytes(payload);
        GetInstance().targ_speed.x = speed.x;
        GetInstance().targ_speed.y = speed.y;
        GetInstance().targ_speed.z = speed.z;
    }
}