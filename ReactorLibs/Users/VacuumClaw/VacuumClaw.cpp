#include "VacuumClaw.hpp"

void VacuumClaw::Start()
{
    motor_stretch.Init(Hardware::hcan_main, 1, DJI_C620);  // 水平方向
    motor_pitch.Init(Hardware::hcan_main, 2, DJI_C620);  // 竖直方向
}

void VacuumClaw::Update()
{

}