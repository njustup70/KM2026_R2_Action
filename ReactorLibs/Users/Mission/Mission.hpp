#pragma once

#include "PathChaser.hpp"
#include "std_math.hpp"

// 任务上下文
struct MissionCtx 
{
    // 路径
    // static const Path<30> PATH_TO_SPEAR;      // 出发点 -> 取矛头点
    // static const Path<20> PATH_TO_DOCK;      // 取矛头点 -> 对接点
    
    // 状态标志
    bool task_finished = false;
    bool move_done = false;
    bool action_done = false;
    
    Vec2 target_position;
    float target_yaw;
    
    // 重置函数
    void Reset()
    {
        task_finished = false;
        move_done = false;
        action_done = false;
    }; 
};