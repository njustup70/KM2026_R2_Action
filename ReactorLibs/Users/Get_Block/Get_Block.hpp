#pragma once
#include "System.hpp"
#include "motor_dji.hpp"
#include "motor_dm.hpp"
#include "infrared_detector.hpp"
#include "relay.hpp"

// ============ 小黄 ============
#define PIN_SENSOR_LOWER  {'F', 1}   // 下仓：块在抬升平台上
#define PIN_SENSOR_UPPER  {'E', 5}   // 上仓：有块（被抬升挡片托住触发）
#define PIN_PRESSURE_RELIEF_VALVE  {'E', 4}   // 泄压阀（高电平打开）

// ============ 负压泵的 GPIO ============
#define PIN_PUMP  {'H', 2}   

class GetBlock : public Application
{
    SINGLETON(GetBlock) : Application("GetBlock") {};

    APPLICATION_OVERRIDE

public:
    void Store();
    void Put();
    void OpenPump();
    void ClosePump();
    void OpenPRV();
    void ClosePRV();
    void _StartBelt();
    void _StopBelt();

    bool IsUpperOccupied() const { return _upper; }
    bool IsLowerOccupied() const { return _lower; }
    int  GetBlockCount() const   { return (int)_upper + (int)_lower; }
    bool IsFull() const          { return _upper && _lower; }

    // ==================== 电机 ====================
    MotorDM  motor_s2325;       // 下摩擦带 (DM, hcan_sub ID 9)
    MotorDJI motor_2006_fric;   // 上摩擦带 (2006, hcan_main ID 4)
    MotorDM motor_dm_lift;     // 抬升机构 (DM, hcan_sub ID 10)
    MotorDJI motor_2006_feed;   // 送块机构 (2006, hcan_main ID 5)
    MotorDJI motor_3508_claw; // 取块爪机构 (3508, hcan_main ID 6)

    InfraredDetector _sensor[2];

private:
    // ==================== 状态机 ====================
    enum class FSM : uint8_t
    {
        IDLE,

        // --- 存块链路 ---
        STORE_LIFT_BEFORE_BELT,// 取块1前抬升到高位，再转摩擦带
        STORE_LIFT_DOWN_TO_PICK,// 抬升在高位已入料→降下来接块
        STORE_BELT_IN,         // 摩擦带入料(块1/块2共用)
        STORE_LIFT_UP,         // 抬升 ↑ (挡片托着块到上仓)
        STORE_WAIT_SECOND,     // 抬升保持↑，挡片托着块1，等待块2入料
        STORE_HOLD_PUT,        // 块2已在下仓，但挡片托着块1不能降→等待Put推出块1
        STORE_LIFT_DOWN,       // 抬升 ↓ (挡片从上往下穿过块2→被顶开→在块2下方闭合)
        STORE_LIFT_BOTTOM,     // 到最底 → 挡片已在块2下方闭合，托住块2

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

    bool is_debug = false;
    bool _belt_enable = false;
    bool _store_debug = false;
    bool _put_debug = false;

    uint32_t _action_tick = 0;  // 阶段开始时间戳 (s)

    // ==================== A板控制的负压泵 ====================
    Pin pin_pump;
    Pin pin_PRV; // 泄压阀
    Relay relay_pump;
    Relay relay_PRV; // 泄压阀

    // ==================== 两个传感器 ====================
    // [0]=下仓  [1]=上仓
    // InfraredDetector _sensor[2];
    Pin _pin_sensor[2];
    bool _grid[2] = {false};  // 传感器缓存，_ReadSensors() 刷新

    // ==================== 抬升DM位置常量（需标定） ====================
    // 向上为负 rad
    float _lift_targ_pos = 0.0f; //用于调试的变量
    float _lift_pos_lower = 10.0f;   // 低位（下仓接料位），防卡
    float _lift_pos_upper = 750.0f;   // 高位（块通过单向门）TODO:标定
    float _lift_vel       = 15.0f;   // 抬升速度 rad/s

    // ==================== 送块2006位置常量（需标定） ====================
    float _feed_pos_retract = 0.0f;     // 收回
    float _feed_pos_extend  = 1780000.0f;  // 伸出（上仓块被推出）

    // ==================== 私有方法 ====================
    void _UpdateFSM();
    void _ReadSensors();
    bool _IsTimeout(float sec) const;
    // void _StartBelt();
    // void _StopBelt();

    bool _LiftNear(float pos_rad, float tol = 0.1f) const
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
