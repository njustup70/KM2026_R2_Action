#include "MainFrame.hpp"
#include "Monitor.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "Jaw.hpp"
#include "farcon.hpp"
#include "Get_Block.hpp"
#include "HostPC.hpp"

StateCore &core = StateCore::GetInstance();
Monitor &monit = Monitor::GetInstance();

StateGraph example_graph("graph_name");
void Action_of_Dege(StateCore *core);
ChassisType &chas = ChassisType::GetInstance();


/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
//  monit.Track(host_pc.chas_pos.x);
//  monit.Track(host_pc.chas_pos.y);
//  monit.Track(host_pc.chas_pos.z);
  System.RegistApp(chas);
  chas.Enable();
  // System.RegistApp(jaw);
  System.RegistApp(key);
  System.RegistApp(hand);
  
  // 配置状态图为简并模式
  example_graph.Degenerate(Action_of_Dege);

  // 向状态机核心注册
  core.RegistGraph(example_graph);
  core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
}

void Action_of_Dege(StateCore *core)
{
  // // 等待1s，板间通讯让C板复位
  // Seq::Wait(1.0f);
  //   chas.MoveAt(Vec2(1.0f, 0.0f));
  //   core->GetCurState().Complete = true;
}

void Action_of_GetBlock(StateCore *core)
{
    hand.targ_pos = 1100000;
    // Seq::WaitUntil([&](){return hand.reached;}); // 因为我弄的只读，所以只能用WaitUntil在.hpp里的模板函数。但是上电目标位置和读到位置都是0，所以会立刻返回，有bug
    Seq::Wait(3);
    hand.Cylinder_v_Open(); // 打开向下的气缸

    Seq::Wait(0.5);
    hand.PumpValve_Open(); // 打开负压泵连的电磁阀，不连接大气
    hand.Pump_Open(); // 打开负压泵，取块

    Seq::Wait(0.5);
    hand.Cylinder_v_Close(); // 关气缸，收回去
    Seq::Wait(0.5);
    hand.targ_pos = 800000;

    core->GetCurState().Complete = true; 
}
