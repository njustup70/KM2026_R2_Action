#include "AutoGraph.hpp"
#include "Monitor.hpp"
#include "Jaw.hpp"
#include "Farcon.hpp"
#include "Chassis.hpp"
#include "Lift_Leg.hpp"
#include "HostPC.hpp"
#include "PathChaser.hpp"
#include "ActionChain.hpp"

/// @brief 所有的路径文件
// #include "r2_area1_1.hpp"


// 自动状态机
StateGraph AutoGraph("Auto");
static void Waiting_for_Start(StateCore *core);
static void Walk_Fetch_TurnAround(StateCore *core);


void Config_AutoStateGraph()
{
  StateBlock& s_wait = AutoGraph.AddState("wait");
  StateBlock& s_area1_1 = AutoGraph.AddState("area1_1");

  s_wait.StateAction = Waiting_for_Start;
  s_area1_1.StateAction = Walk_Fetch_TurnAround;

  s_wait.LinkTo(&s_wait.Complete, s_area1_1);
  s_area1_1.LinkTo(&s_area1_1.Complete, s_wait);

  // 向状态机核心注册
  APP::state_core.RegistGraph(AutoGraph);
  APP::state_core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
}

/**
 * @brief 刚开机时，等待出发信号的状态
 * 
 * @details 现在先设置成等待一段时间吧，后续可以改成比如 等到自检通过了，才允许动之类的
 */
void Waiting_for_Start(StateCore *core)
{
  APP::monit.LogInfo("Waiting for start...");

  // 取矛头先初始化
  Seq::WaitUntil([](){
    return APP::jaw.IsOrigined();
  });

  APP::monit.LogInfo("Jaw Init OK.");

  // 等待工控机连接
  Seq::WaitUntil([](){
    return MOD::host_pc.IsOnline();
  });

  APP::monit.LogInfo("HostPC Online.");

  // 等待一段时间，确保一切稳定
  Seq::Wait(5.0f);

  // 等待出发信号
  APP::monit.LogInfo("Ready to Go...");

  // 等待遥控器的出发指令
  Seq::WaitUntil([](){
    return MOD::farcon.button_second_half[0] == 1;
  });

  core->GetCurState().Complete = true; // 设置状态完成标志，触发状态转移
}

/**
 * @brief 这是R2出发的第一个动作，先走到矛台前，然后取矛，接着转身准备对接
 * 
 * @param core 
 */
void Walk_Fetch_TurnAround(StateCore *core)
{
  // 先放平爪子并张开
  APP::jaw.PitchDown();
  APP::jaw.OpenClaw();

  // 首先移动到矛台前
  APP::monit.LogInfo("Moving to Spear-Table...");
  
  // APP::path_chaser.ChasePath(Path_R2_Area1_1);

  // 等待路径跟踪完成
  // while(!APP::path_chaser.IsFinished())
  // {
  //   // 获取理论移动速度（相对车体坐标系的）
  //   // Vec3 speed_vec = APP::path_chaser.GetCmdBody();
  //   Vec3 speed_vec = Vec3(0, 0, 0);

  //   APP::chas.Move(speed_vec);
  //   Seq::Wait(0.005f);  // 200Hz标准更新频率
  // }

  // 等待路径跟踪到矛台
  // MOVE::MoveToTargPos(Path_R2_Area1_Spear_Table);

  // 接近矛台后，执行取矛动作
  APP::monit.LogInfo("Arrived, Try to fetch.");
  APP::jaw.CloseClaw();

  // 取到矛头，移动到对接位置，准备对接
  APP::jaw.PitchUp();
  // MOVE::MoveToTargPos(Path_R2_Area1_Dock);
  APP::monit.LogInfo("Fetch OK, Wait for Dock");

  // 无限等待，因为后面可能有问题。。
  Seq::WaitUntil([](){
    return false;
  });
}
