#pragma once
#include "Monitor.hpp"
#include <cstdint>

/**************  动作的封装  **************/ 

/// @brief 单个动作
struct ActionItem
{
    const char* name;
    bool (*func)();     // 返回是否完成
    int max_tries;      // 最大重试次数s
    float delay;        // 重试间隔
};

class ActionChain
{
private:
    static const int MAX_ACTIONS = 10; // 单个动作链最多先弄10个“原子”动作吧
    const char* _chain_name;           // 动作链名称
    ActionItem _actions[MAX_ACTIONS];
    int _m_cnt;

public:
    ActionChain(const char* name = "Unnamed");

    /// @brief 给动作链取名，链式调用使用
    ActionChain& SetName(const char* name);

    /// @brief 获取动作链名称
    const char* GetName() const { return _chain_name; }

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
