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
            APP::chas.Move(speed_vec);
            Seq::Wait(0.005f); // 200hz更新频率
        }
        
        APP::monit.LogInfo("Move to target position.");
        return true;
    }
}

/**************  动作的封装  **************/ 
namespace ACTION
{
    /// @brief 单个动作
    struct ActionItem
    {
        const char* name;
        bool (*func)();     // 返回是否完成
        int max_tries;      // 最大重试次数
        float delay;        // 重试间隔
    };

    class ActionChain
    {
    private:


        static const int MAX_ACTIONS = 10; // 单个动作链最多先弄10个“原子”动作吧
        ActionItem _actions[MAX_ACTIONS];
        int _m_cnt;

    public:
        ActionChain();

        /// @brief 添加一个动作到动作链
        ActionChain& Add(const char* name, bool (*func)(), int retries = 1, float delay = 1.0f);

        /// @brief 执行整个动作链
        bool Run();

        /// @brief 清空动作链
        void Clear();

        /// @brief 获取当前动作数量
        int GetCount() const { return _m_cnt; }
        
        /// @brief 检查是否为空
        bool IsEmpty() const { return _m_cnt == 0; }
    };
}