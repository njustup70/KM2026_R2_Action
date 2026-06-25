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
    float pitch_lim_max = PI / 4;
    int snap_lim_min = 0;
    int snap_lim_max = 999999;

    /***** 内部维护的状态值 *****/
    float stretch_pos = 0.0f;   // 0 - 1。为0时完全收缩，1时完全伸出
    float snap_pos = 0.0f;      // 0 - 1。为0时完全闭合，1时完全张开
    float pitch_pos = 0.0f;     // 0 - 1。为0时完全水平，1时完全竖直
    bool enabled = false;

    public:
    /**
     * @brief 伸出夹爪，保持开启状态
     * @details 水平快速伸出，同时Pitch变为水平状态，用于夹取的snap电机打开
     */
    void StretchOut();

    /**
     * @brief 
     * 
     */

};