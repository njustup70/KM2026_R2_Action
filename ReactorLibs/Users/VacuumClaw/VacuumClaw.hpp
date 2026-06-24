#pragma once
#include "System.hpp"
#include "motor_dji.hpp"
#include "motor_dm.hpp"

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

    int stretch_lim_min = 0;
    int stretch_lim_max = 999999;

    float pitch_lim_min = 0.0f;
    float pitch_lim_max = PI / 4;

    int snap_lim_min = 0;
    int snap_lim_max = 999999;
};