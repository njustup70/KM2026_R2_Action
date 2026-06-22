#pragma once
#include "System.hpp"
#include "motor_dji.hpp"
#include "motor_dm.hpp"
#include "infrared_detector.hpp"

// ============ 传感器引脚（TODO: 按实际接线填写） ============
#define PIN_SENSOR_ENTRY  {'?', 0}   // 入口：摩擦带末端，块准备进入下仓
#define PIN_SENSOR_LOWER  {'?', 1}   // 下仓：块在抬升平台上
#define PIN_SENSOR_UPPER  {'?', 2}   // 上仓：单向门上方有块
#define PIN_SENSOR_EXIT   {'?', 3}   // 出口：块被推出确认

class GetBlock : public Application
{
    SINGLETON(GetBlock) : Application("GetBlock") {};

    APPLICATION_OVERRIDE

public:
    // ==================== 公有接口 ====================
    void Store();
    void Put();

    bool IsUpperOccupied() const { return _upper; }
    bool IsLowerOccupied() const { return _lower; }
    int  GetBlockCount() const   { return (int)_upper + (int)_lower; }
    bool IsFull() const          { return _upper && _lower; }

    // ==================== 电机 ====================
    MotorDM   motor_s2325;       // 下摩擦带 (DM, hcan_sub ID 9)
    MotorDJI  motor_2006_fric;   // 上摩擦带 (2006, hcan_main ID 6)
    MotorDM   motor_dm_lift;     // 抬升机构 (DM, hcan_sub ID 10)
    MotorDJI  motor_2006_feed;   // 送块机构 (2006, hcan_main ID 7)

private:
    // ==================== 状态机 ====================
    enum class FSM : uint8_t
    {
        IDLE,

        // --- 存块链路 ---
        STORE_BELT_IN,         // 摩擦带入料(块1/块2共用)
        STORE_LIFT_UP,         // 抬升 ↑ (块通过单向门到上仓)
        STORE_WAIT_SECOND,     // 抬升保持↑，等待块2进入下仓托住块1
        STORE_LIFT_DOWN,       // 抬升 ↓ (单向门打开穿过块2)
        STORE_LIFT_BOTTOM,     // 到最底 → 单向门复位托住块2

        // --- 放块链路 ---
        PUT_FEED_PUSH,         // 送块2006伸出 → 上仓块从前方推出
        PUT_FEED_RETRACT,      // 送块收回
        PUT_LIFT_UP,           // 下仓块抬升到上仓
        PUT_LIFT_DOWN,         // 抬升回低位

        COMPLETE,
        ABORT,
    } _fsm = FSM::IDLE;

    // ==================== 仓状态 ====================
    bool _upper = false;        // 上仓有块
    bool _lower = false;        // 下仓(抬升平台)有块
    bool _storing_b2 = false;   // 正在存第二个块

    bool _store_pending = false;
    bool _put_pending   = false;
    bool _fsm_busy      = false;

    uint32_t _action_tick = 0;  // 阶段开始时间戳 (s)

    // ==================== 四个传感器 ====================
    // [0]=入口  [1]=下仓  [2]=上仓  [3]=出口
    InfraredDetector _sensor[4];
    Pin _pin_sensor[4];
    bool _grid[4] = {false};  // 传感器缓存，_ReadSensors() 刷新

    // ==================== 抬升DM位置常量（需标定） ====================
    float _lift_pos_lower = 0.0f;   // 低位（下仓接料位）
    float _lift_pos_upper = 0.3f;   // 高位（块通过单向门）TODO:标定
    float _lift_vel       = 2.0f;   // 抬升速度 rad/s

    // ==================== 送块2006位置常量（需标定） ====================
    float _feed_pos_retract = 0.0f;     // 收回
    float _feed_pos_extend  = 2000.0f;  // 伸出（上仓块被推出）TODO:标定

    // ==================== 私有方法 ====================
    void _UpdateFSM();
    void _ReadSensors();
    bool _IsTimeout(float sec) const;
    void _StartBelt();
    void _StopBelt();

    bool _LiftNear(float pos_rad, float tol = 0.02f) const
    {
        return fabs(motor_dm_lift.driver.measure.total_angle - pos_rad) < tol;
    }

    bool _FeedNear(float pos_code, float tol = 50.0f) const
    {
        return fabs(motor_2006_feed.driver.measure.total_angle - pos_code) < tol;
    }
};

namespace APP
{
    extern GetBlock& get_block;
}
