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

ChassisType& APP::chas = ChassisType::GetInstance();

/**
 * @brief 用于处理 C板 发回的自解里程计数据的回调
 * @details 其用于接收消息的端口为 0x221 can_id
 */
void ChassisType::ChasTaskCallback(uint8_t task_id, const uint8_t *payload, uint8_t payload_len, void *user_ctx)
{
    using namespace APP;

    // 发送计数器，板间通讯是200hz，算法组要求100hz
    static int send_cnt = 0; 

    chas.chas_odom.pos.x = StdMath::UnpackInt16ToFloat(&payload[0], 1000.0f);
    chas.chas_odom.pos.y = StdMath::UnpackInt16ToFloat(&payload[2], 1000.0f);
    chas.chas_odom.pos.z = StdMath::UnpackInt16ToFloat(&payload[4], 1000.0f);

    if(++send_cnt >= 2) 
    {
        send_cnt = 0;
        MOD::host_pc.SendOdom(chas.chas_odom.pos.x, chas.chas_odom.pos.y, chas.chas_odom.pos.z);
    }
}

void ChassisType::Start()
{
    MOD::board_can.RegisterTask(1, ChasTaskCallback, this);
    MOD::host_pc.Regist(HostPC::FuncCode::ctrl_spd, HostCmdCallback, this);
    
    // 发送重置底盘和腿的指令，包括电机和里程计
    uint8_t payload[1] = {0x01}; 
    MOD::board_can.SendTask(0x210, 4, payload, 1, false); 
}

/**
 * @brief 向底盘发送速度指令
 */
void _SendSpd(Vec3 Spd)
{
    // 6 字节 uint8，完全匹配接收端格式
    uint8_t payload[6];
    const float scale = 1000.0f;

    StdMath::PackFloatToInt16(&payload[0], Spd.x, scale);
    StdMath::PackFloatToInt16(&payload[2], Spd.y, scale);
    StdMath::PackFloatToInt16(&payload[4], Spd.z, scale);
    MOD::board_can.SendTask(0x210, 1, payload, 6, false);
}

void ChassisType::Update() 
{
    if (!enabled)
    {
        return; // 底盘未使能，直接返回
    }

    if(System.out_from_debugmode)
    {
        uint8_t payload[1] = {0x01}; // 发送重置底盘和腿的指令，包括电机和里程计
        MOD::board_can.SendTask(0x210, 3, payload, 1, false); // 发送停止指令
    }

    // 工控机的刷新先放这里啊，只是图方便，不代表这么做是对的
    MOD::host_pc.TickLoop();
    
    if (_safe_lock_tick > 0)
    {
        // 将速度指令发送到 C板
        // _UploadSpeed();
        _SendSpd(targ_speed);
    }
    else
    {
        _SendSpd(Vec3(0, 0, 0));
    }
    
    targ_velo = targ_speed.Length();

    // 更新安全锁
    _safe_lock_tick -= 5;
}

void ChassisType::_UploadSpeed()
{
    static float runtime_cnt = 0;
    // 仅当底盘使能时才工作
    if (enabled && _safe_lock_tick > 0)
    {
        runtime_cnt = 0;
        _SendSpd(targ_speed);
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
                _SendSpd(targ_speed);
            }
            else // (1.2) 1s还停不下来，强制进入空档
            {
                for (int i = 0; i < 4; i++)
                {
                    targ_speed = Vec3(0, 0, 0);
                }
            }
        }
        else // (2) 底盘已经停止，直接进入空档
        {
            for (int i = 0; i < 4; i++)
            {
                targ_speed = Vec3(0, 0, 0);
            }
        }
    }
}

// inline void ChassisType::_SendSpdToMotor()
// {
//     // 计算x, y, w合成分量
//     _motor_spd[0] = (-targ_speed.x + targ_speed.y) / (BSP_SQRT2) + targ_speed.z * ROTATE_RADIUS;
//     _motor_spd[1] = (targ_speed.x + targ_speed.y) / (BSP_SQRT2) + targ_speed.z * ROTATE_RADIUS;
//     _motor_spd[2] = (-targ_speed.x - targ_speed.y) / (BSP_SQRT2) + targ_speed.z * ROTATE_RADIUS;
//     _motor_spd[3] = (targ_speed.x - targ_speed.y) / (BSP_SQRT2) + targ_speed.z * ROTATE_RADIUS;

//     // 发送速度指令到电机
//     for (int i = 0; i < 4; i++)
//     {
//         // motors[i].SwitchMode(MotorDJIMode::PID_SpeedControl);
//         motors[i].SetSpd((_motor_spd[i] * 60.0f) / (PI * WHEEL_DIAMETER));
//     }
// }

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