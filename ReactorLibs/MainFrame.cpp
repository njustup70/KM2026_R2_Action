#include "MainFrame.hpp"
#include "Monitor.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "Jaw.hpp"
#include "Get_Block.hpp"
#include "HostPC.hpp"



static StateCore &core = StateCore::GetInstance();
static Monitor &monit = Monitor::GetInstance();
static ChassisType &chas = ChassisType::GetInstance();
static Jaw &jaw = Jaw::GetInstance();

// 手动遥控的状态机
StateGraph ManualGraph("Manual");


void Action_of_Dege(StateCore *core);




/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
  // 底盘启动
  System.RegistApp(chas);
  chas.Enable();

  // 初始化遥控器
  farcon.init(Hardware::huart_farcon);

  // System.RegistApp(jaw);
  // System.RegistApp(key);
  // System.RegistApp(hand);
  
  // 配置状态图

  // 向状态机核心注册
  core.RegistGraph(ManualGraph);
  core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
}

void Waiting_for_Start(StateCore *core)
{
    // 等待
    
}