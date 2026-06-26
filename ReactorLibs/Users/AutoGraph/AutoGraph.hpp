#pragma once
#include "StateCore.hpp"
#include "PathChaser.hpp"
#include "Chassis.hpp"
#include <cstdint>

/**************  移动的封装  **************/ 
namespace MOVE
{
    /// @brief 沿预设路径移动并等待完成
    /// @param path_data 路径数据
    /// @return true-到达终点, false-失败

    template <size_t N>
    bool MoveToTargPos(const Path<N>& path)
    {
        APP::path_chaser.ChasePath(path);
        
        while (!APP::path_chaser.IsFinished())
        {
            Vec3 speed_vec = APP::path_chaser.GetCmdBody();
            APP::chas.Move(speed_vec, 0.01f);
            Seq::Wait(0.005f); // 200hz更新频率
        }
        
        APP::monit.LogInfo("Move to target position.");
        return true;
    }
}

void Config_AutoStateGraph();
extern StateGraph AutoGraph;