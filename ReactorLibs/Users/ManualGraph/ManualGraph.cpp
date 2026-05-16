#include "ManualGraph.hpp"
#include "Monitor.hpp"
#include "Jaw.hpp"
#include "Farcon.hpp"
#include "Chassis.hpp"

// 手动遥控的状态机
StateGraph ManualGraph("Manual");

void Waiting_for_Start(StateCore *core);
void Response_Button(StateCore *core);

void Config_StateGraph()
{
  StateBlock& s_wait = ManualGraph.AddState("wait");
  StateBlock& s_response = ManualGraph.AddState("resp");

  s_wait.StateAction = Waiting_for_Start;
  s_response.StateAction = Response_Button;

  s_wait.LinkTo(&s_wait.Complete, s_response);
  s_response.LinkTo(&s_response.Complete, s_wait);

  // 向状态机核心注册
  APP::state_core.RegistGraph(ManualGraph);
  APP::state_core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
}

void Waiting_for_Start(StateCore *core)
{
  APP::monit.LogInfo("I'm here...");

  Seq::WaitUntil([](){
    return APP::jaw.IsOrigined();
  });

  APP::monit.LogInfo("Ready to Response...");

  core->GetCurState().Complete = true; // 设置状态完成标志，触发状态转移
}

void Response_Button(StateCore *core)
{
    if (MOD::farcon.button_first_half[0] == 1)
    {
        APP::jaw.PitchDown();
    }
    
    if (MOD::farcon.button_first_half[1] == 1)
    {
        APP::jaw.PitchUp();
    }

    if (fabs(MOD::farcon.jys_value[3]) > 10)
    {
        // 计算速度
        float v_x = MOD::farcon.jys_value[3] / 100.0f * 0.5f; // 最大速度0.5m/s
        // 向底盘发送移动指令
        APP::chas.Move(Vec3(v_x, 0, 0));
    }
}
