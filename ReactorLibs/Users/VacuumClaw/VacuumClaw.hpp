#pragma once
#include "System.hpp"
#include "motor_dji.hpp"
#include "motor_dm.hpp"
#include "relay.hpp"

class VacuumClaw : public Application
{
    SINGLETON(VacuumClaw) : Application("VacuumClaw") {}
    APPLICATION_OVERRIDE

    private:
    /// @brief 用于将水平夹爪伸出的电机
    MotorDJI motor_stretch;

    /// @brief 用于捏合前方物块的电机
    MotorDJI motor_snap;

    /// @brief 用于翻转物块的电机
    MotorDM motor_pitch;

    /***** 电机限幅 *****/
    int stretch_lim_min = 0;
    int stretch_lim_max = 999999;
    float pitch_lim_min = 0.0f;
    float pitch_lim_max = 0.0f; // 单位rad
    int snap_lim_min = 0;
    int snap_lim_max = 999999;

    /***** 内部维护的状态值 *****/
    float stretch_pos = 0.0f;   // 0 - 1。为0时完全收缩，1时完全伸出
    float snap_pos = 0.0f;      // 0 - 1。为0时完全闭合，1时完全张开
    float pitch_pos = 0.0f;     // 0 - 1。为0时完全水平，1时完全竖直
    bool enabled = false;

    bool is_debug = false; // 用于快速debug
    bool is_out_debug = false; // 调试伸爪子
    bool is_back_debug = false;
    bool set_damiao_zero = true; // 第一次 update 调零一次

    public:
    /**
     * @brief 伸出夹爪，保持开启状态
     * @details 水平快速伸出，同时Pitch变为水平状态，用于夹取的snap电机打开
     */
    void StretchOut();

    /**
     * @brief 收回夹爪
     * @details 水平收回，同时Pitch变为竖直状态，用于夹取的snap电机关闭
     */
    void StretchBack();

    /**
     * @brief 
     * 
     */
    void Enable();
    void Disable();

    bool need_to_fetch = false; // 判断是否要伸爪取块

};

namespace APP
{
    extern VacuumClaw& vac_claw;
}