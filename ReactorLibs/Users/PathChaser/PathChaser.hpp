#pragma once
#include "System.hpp"

/**
 * @brief 提前设置好的路径。注意，本库没有动态规划功能
 * 
 */
class Path
{
    public:
    struct Waypoint
    {
        Vec2 pos;      // 位置
        float yaw_rad; // 朝向，单位rad
        Vec2 vel_vec;  // 期望速度向量，单位m/s
        float omega_rad; // 期望角速度，单位rad/s
    };

    Waypoint waypoints[8];  // 一段路径最多8个路径点
};

class PathChaserType : public Application
{

};



