#include "MainFrame.hpp"
#include "Monitor.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "Jaw.hpp"
#include "farcon.hpp"

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
  // System.RegistApp(lift_leg);
  System.RegistApp(chas);
  chas.Enable();
  System.RegistApp(jaw);
  System.RegistApp(key);
  
  // 配置状态图为简并模式
  example_graph.Degenerate(Action_of_Dege);

  // 向状态机核心注册
  core.RegistGraph(example_graph);
  core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
}

void Action_of_Dege(StateCore *core)
{

}
