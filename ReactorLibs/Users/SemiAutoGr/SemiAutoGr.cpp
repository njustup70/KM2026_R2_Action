#include "SemiAutoGr.hpp"
#include "Monitor.hpp"
#include "Jaw.hpp"
#include "Farcon.hpp"
#include "Chassis.hpp"
#include "Lift_Leg.hpp"
#include "HostPC.hpp"
#include "PathChaser.hpp"


/// @brief 所有的路径文件
// #include "r2_area1_1.hpp"
#include "start_to_table.hpp"
#include "table_to_dock.hpp"

// 自动状态机
StateGraph AutoGraph("Auto");
static void Waiting_for_Start(StateCore *core);
static void Walk_Fetch(StateCore *core);
static void Walk_To_Dock(StateCore *core);

void Config_AutoStateGraph()
{
  StateBlock& s_wait = AutoGraph.AddState("wait");
  StateBlock& s_area1_1 = AutoGraph.AddState("area1_1");
  StateBlock& s_area1_2 = AutoGraph.AddState("area1_2");

  s_wait.StateAction = Waiting_for_Start;
  s_area1_1.StateAction = Walk_Fetch;
  s_area1_2.StateAction = Walk_To_Dock;

  s_wait.LinkTo(&s_wait.Complete, s_area1_1);
  s_area1_1.LinkTo(&s_area1_1.Complete, s_area1_2);
  s_area1_2.LinkTo(&s_area1_2.Complete, s_wait);

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
  Seq::Wait(2.0f);

  // 等待出发信号
  APP::monit.LogInfo("Ready to Go...");

  APP::jaw.CloseClaw(); 
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
void Walk_Fetch(StateCore *core)
{
  APP::jaw.PitchDown();
  Seq::Wait(0.5);
  APP::jaw.OpenClaw();
  Seq::Wait(0.5);

  MOVE::MoveToTargPos(StartToTablePath);
  
  APP::jaw.CloseClaw();
  // 先往上抬 1.5cm
  APP::jaw.ChangePos(0, 15);
  Seq::Wait(0.5);
  // 然后抬起 Pitch爪子
  APP::jaw.PitchUp();
  Seq::Wait(0.5);

  // 等待遥控器指令，然后才进入下一个状态
  Seq::WaitUntil(MOD::farcon.button_first_half[0]);

  core->GetCurState().Complete = true;
}

/**
 * @brief 一区第二个动作，取到矛头后，转身走到对接位置，准备对接
 * 
 * @param core 
 */

void Walk_To_Dock(StateCore *core)
{

  MOVE::MoveToTargPos(TableToDockPath);
  Seq::Wait(1.0f);

  APP::jaw.SetTrackMode(true);

  // Seq::WaitUntil()

  // 等待遥控器指令，然后才进入下一个状态
  Seq::WaitUntil(MOD::farcon.button_first_half[0]);

  core->GetCurState().Complete = true;
}