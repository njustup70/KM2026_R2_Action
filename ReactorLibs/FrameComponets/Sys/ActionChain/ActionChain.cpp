#include "ActionChain.hpp"
#include "Monitor.hpp"
#include "StateCore.hpp"

ActionChain::ActionChain(const char* name)
{
    _chain_name = name ? name : "Unnamed";
    _m_cnt = 0;
    for(int i = 0; i < MAX_ACTIONS; i++)
    {
        _actions[i] = {nullptr, nullptr, 1, 1.0f};
    }
}

ActionChain& ActionChain::SetName(const char* name)
{
    _chain_name = name ? name : "Unnamed";
    return *this;
}

ActionChain& ActionChain::Add(const char* name, bool (*func)(), int retries, float delay)
{
    if(_m_cnt >= MAX_ACTIONS)
    {
        APP::monit.LogError("[ActionChain: %s]: Max actions reached (%d), cannot add '%s'", _chain_name, MAX_ACTIONS, name);
        return *this;
    }

    if(func == nullptr)
    {
        APP::monit.LogError("[ActionChain: %s]: Null function for action '%s'", _chain_name, name);
        return *this;
    }
    
    _actions[_m_cnt].name = name; 
    _actions[_m_cnt].func = func;
    _actions[_m_cnt].max_tries = retries > 0 ? retries : 1; 
    _actions[_m_cnt].delay = delay > 0 ? delay : 1.0f;
    _m_cnt++;

    return *this;
}

bool ActionChain::Run()
{
    if (_m_cnt == 0) 
    {
        APP::monit.LogWarning("[ActionChain: %s]: No actions to execute", _chain_name);
        return false;
    }

    APP::monit.LogSpec("--- --- [ActionChain: %s] Start (total %d actions) --- ---\n", _chain_name, _m_cnt);

    for(int i = 0; i < _m_cnt; i++)
    {
        ActionItem& act = _actions[i];

        APP::monit.LogInfo("[%s][%d/%d] Action: %s", _chain_name, i + 1, _m_cnt, act.name);

        bool success = false;

        for(int retry = 0; retry < act.max_tries; ++retry)
        {
            if(act.func())
            {
                if(retry > 0)
                {
                    // 重试 retry 次后成功
                    APP::monit.LogInfo("    Action '%s' succeeded after %d retries", act.name, retry);
                }
                else 
                {
                    // 对于单次执行的动作，或者第一次成功的动作
                    APP::monit.LogInfo("    Action '%s' success", act.name);
                }
                success = true;
                break;
            }

            if(retry < act.max_tries - 1)
            {
                // 有失败的情况
                APP::monit.LogWarning("    Failed, retry %d/%d (delay %.1fs)", retry + 1, act.max_tries, act.delay);
                Seq::Wait(act.delay); 
            }
        }

        if(!success)
        {
            // 多次重试后没成功，或者本来就只执行一次的动作失败了
            APP::monit.LogError("ActionChain Failed at action: %s", act.name);
            APP::monit.LogError("Failed after %d attempts", act.max_tries);
            return false;
        }
    }   
    
    APP::monit.LogSpec("--- --- ActionChain Completed --- ---\n");
    return true;
}

void ActionChain::Clear() 
{
    _m_cnt = 0;
    for (int i = 0; i < MAX_ACTIONS; ++i) 
    {
        _actions[i].name = nullptr;
        _actions[i].func = nullptr;
        _actions[i].max_tries = 1;
        _actions[i].delay = 1.0f;
    }
}