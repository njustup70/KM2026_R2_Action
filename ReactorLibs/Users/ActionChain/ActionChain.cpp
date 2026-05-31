#include "ActionChain.hpp"

namespace ACTION
{

ActionChain::ActionChain()
{
    _m_cnt = 0;
    for(int i = 0; i < MAX_ACTIONS; i++)
    {
        _actions[i] = {nullptr, nullptr, 1, 1.0f};
    }
}

ActionChain& ActionChain::Add(const char* name, bool (*func)(), int retries, float delay)
{
    if(_m_cnt >= MAX_ACTIONS)
    {
        APP::monit.LogError("ActionChain: Max actions reached (%d), cannot add '%s'", MAX_ACTIONS, name);
    }

    if(func == nullptr)
    {
        APP::monit.LogError("Null function for action '%s'", name);
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
    for(int i = 0; i < _m_cnt; i++)
    {
        ActionItem& act = _actions[i];

        bool success = false;

        for(int retry = 0; retry < act.max_tries; ++retry)
        {
            if(act.func())
            {
                success = true;
                break;
            }
            
        }
    }
}

}// namespace ACTION
