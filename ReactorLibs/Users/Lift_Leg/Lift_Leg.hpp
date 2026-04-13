#pragma once

#include "System.hpp"

class Lift_Leg : public Application
{
    SINGLETON(Lift_Leg) : Application("Lift_Leg") {}
    APPLICATION_OVERRIDE

public:
    MotorDJI motor_front_left;
    MotorDJI motor_front_right;

    // 后腿是二合一的
    MotorDJI motor_back;


    MotorDJI motor_Test;

private:
    bool enabled_ = false;
};
