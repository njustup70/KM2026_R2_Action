#include "ChallengeGraphBlue.hpp"
#include "Monitor.hpp"
#include "Jaw.hpp"
#include "Farcon.hpp"
#include "Chassis.hpp"
#include "Lift_Leg.hpp"
#include "HostPC.hpp"
#include "PathChaser.hpp"
#include "VacuumClaw.hpp"
#include "Area2_Act.hpp"
#include "Area1_Act.hpp"
#include "CommCenter.hpp"
#include "Get_Block.hpp"
#include "MainLoop.hpp"

/// @brief 所有的路径文件
#include "ttd_1.hpp"
#include "ttd_2.hpp"
#include "ttd_3.hpp"

#include "a12_1.hpp"
#include "a12_2.hpp"
#include "a12_3.hpp"

#include "a12_b1.hpp"
#include "a12_b2.hpp"
#include "a12_b3.hpp"


#include "a23_2.hpp"

static void Response_Button();

/// @brief 当前需要取的矛头编号
static uint8_t need_spear_id = 3;

static bool fetch_again = false;    // 是否需要再次取矛头
static bool move_to_area2 = false;  // 是否可以移动到二区了

// 自动状态机
StateGraph R2_Challenge_Blue_Graph("R2_Challenge");
static void Waiting_for_Start(StateCore *core);
static void Walk_Fetch(StateCore *core);
static void Walk_To_Dock(StateCore *core);
static void Dock_Fetch_And_Back(StateCore *core);
static void Goto_Area2(StateCore *core);
static void Area2_HostAction(StateCore *core);
static void Area2End_BackOrigin(StateCore *core);

static void Walk_into_Area2(StateCore *core);

int Config_R2_Challenge_Blue_StateGraph()
{
  StateBlock& s_wait = R2_Challenge_Blue_Graph.AddState("wait");
  StateBlock& s_area1_f1 = R2_Challenge_Blue_Graph.AddState("area1_f1");       // fetch our first spear
  StateBlock& s_area1_td = R2_Challenge_Blue_Graph.AddState("area1_td");       // to dock
  StateBlock& s_dock_fetch = R2_Challenge_Blue_Graph.AddState("dock_fetch");   // fetch and dock again
 
  StateBlock& s_walk_from_a1_to_a2 = R2_Challenge_Blue_Graph.AddState("walk_a12");
  StateBlock& s_a2_hostact = R2_Challenge_Blue_Graph.AddState("area2_hostact");  // 二区由工控机控制的动作序列
  StateBlock& s_a2end_backorigin = R2_Challenge_Blue_Graph.AddState("a2end_backor");  // 二区结束后，回到原点

  s_wait.StateAction = Waiting_for_Start;

  /**   一区的状态**/
  s_area1_f1.StateAction = Walk_Fetch;
  s_area1_td.StateAction = Walk_To_Dock;
  s_dock_fetch.StateAction = Dock_Fetch_And_Back;

  /**   二区的状态**/
  s_walk_from_a1_to_a2.StateAction = Goto_Area2;
  s_a2_hostact.StateAction = Area2_HostAction;
  s_a2end_backorigin.StateAction = Area2End_BackOrigin;

  /**----------     状态转移链接    ----------**/
  s_wait.LinkTo(&s_wait.Complete, s_area1_f1);
  s_area1_f1.LinkTo(&s_area1_f1.Complete, s_area1_td);
  s_area1_td.LinkTo(&s_area1_td.Complete, s_dock_fetch);

  // 区分：再次去取矛头，还是进入二区
  s_dock_fetch.LinkTo(&fetch_again, s_dock_fetch);
  s_dock_fetch.LinkTo(&move_to_area2, s_walk_from_a1_to_a2);
  
  // 进入二区，并执行任务
  s_walk_from_a1_to_a2.LinkTo(&s_walk_from_a1_to_a2.Complete, s_a2_hostact);
  s_a2_hostact.LinkTo(&s_a2_hostact.Complete, s_a2end_backorigin);
  // 可以选择回到原点
  s_a2end_backorigin.LinkTo(&s_a2end_backorigin.Complete, s_wait);
  
  /**----------     向状态机核心注册    ----------**/
  return APP::state_core.RegistGraph(R2_Challenge_Blue_Graph);
  // APP::state_core.Enable(0);     // 启动状态机核心，指定初始状态图为0号图
}

/*******************************************************************************/

/**
 * @brief 刚开机时，等待出发信号的状态
 * 
 * @details 现在先设置成等待一段时间吧，后续可以改成比如 等到自检通过了，才允许动之类的
 */
void Waiting_for_Start(StateCore *core)
{
  
  Area2::mei_lin_array.is_blue = true; // 设置为蓝场

  APP::jaw.CloseClaw();
  APP::monit.LogInfo("Waiting for start...");
  APP::main_loop.test_led.Breath(Color::White, 2.0f);

  APP::leg.F_AlmostOrigin();
  APP::leg.B_AlmostOrigin();

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  APP::main_loop.test_led.Lit(Color::White);

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
  Seq::Wait(0.75f);

  // 等待出发信号
  APP::monit.LogInfo("Ready to Go...");   
  
  // 等待遥控器的出发指令
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  core->GetCurState().Complete = true; // 设置状态完成标志，触发状态转移
}

/*******************************************************************************/

/**
 * @brief 这是R2出发的第一个动作，先走到矛台前，然后取矛，接着转身准备对接
 * 
 * @param core 
 */
void Walk_Fetch(StateCore *core)
{
  APP::leg.SetFrontPos(-10);
  APP::leg.SetBackPos(-10);

  // 灯带：渐变
  APP::main_loop.test_led.GradientFlow(Color::Up_Purpor * 0.65f, Color::Up_Yellow * 0.65f, 1.0f);
  
  // 从原点出发取矛头
  FetchSpear_FromOrigin_Blue(3);

  core->GetCurState().Complete = true;
}

/*******************************************************************************/

/**
 * @brief 一区第二个动作，取到矛头后，转身走到对接位置，准备对接
 * 
 * @param core 
 */
void Walk_To_Dock(StateCore *core)
{
  // 走到对接点
  GotoDock_Blue(3);

  // 进入对接模式
  APP::jaw.SetTrackMode(true);

  APP::main_loop.test_led.Running(Color::Up_Yellow, 0.4f, 0.8f);

  // 等待遥控器指令，然后才进入下一个状态
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);

    if (APP::comm_center.IsDocked())
    {
      break;
    }
  }

  // 灯带：进度条
  float wait_progress = 0.0f;
  APP::main_loop.test_led.ProgBar(Color::Up_Yellow, &wait_progress);

  // 打开夹爪，给R1时间拿回兵器
  APP::jaw.OpenClaw();    // 依旧OpenClaw

  // 等待 6 秒
  float wait_time_s = 0.0f;
  while (wait_time_s < 6.0f)
  {
    wait_time_s += 0.05f;
    wait_progress = wait_time_s / 6.0f;
    Seq::Wait(0.05f);
  }

  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }
  
  APP::jaw.SetTrackMode(false);

  APP::jaw.SetPosCode(APP::jaw._hori_origin_code - JawConst::_hori_max_travel_code / 2 - 100000, 
    APP::jaw._vert_origin_code - 226000);

  APP::monit.LogInfo("Go to Climb.");

  need_spear_id++;
  APP::comm_center.ClearDockFlag(); // 清除对接标志，避免下次误触发

  core->GetCurState().Complete = true;
}

/*******************************************************************************/

/**
 * @brief 后续的从对接点取矛头并返回的逻辑
 * 
 * @param core 
 */
void Dock_Fetch_And_Back(StateCore *core)
{
  // 记录是否放弃对接（每次进入都默认为false）
  bool giveup_dock = false;

  fetch_again = false;

  // 灯带：渐变
  APP::main_loop.test_led.GradientFlow(Color::Up_Purpor * 0.65f, Color::Up_Yellow * 0.65f, 1.0f);
  // 从对接点出发取矛头
  FetchSpear_FromDock_Blue(need_spear_id);

  /**--------------     返回对接点    ---------------**/

  // 走到对接点
  GotoDock_Blue(need_spear_id);

  // 进入对接模式
  APP::jaw.SetTrackMode(true);

  APP::main_loop.test_led.Running(Color::Up_Yellow, 0.4f, 0.8f);

  // 等待遥控器指令，然后才进入下一个状态
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);

    if (APP::comm_center.IsDocked())
    {
      // 打开夹爪，给R1时间拿回兵器
      APP::jaw.OpenClaw();    // 依旧OpenClaw
      break;
    }

    if (APP::comm_center.IsDockGiveUp())
    {
      // 离开对接模式
      APP::jaw.SetTrackMode(false);
      // 清除标志位
      APP::comm_center.ClearDockFlag();
      // 跳出等待，准备进入二区
      giveup_dock = true;
      break;
    }
  }

  // 灯带：进度条
  float wait_progress = 0.0f;
  APP::main_loop.test_led.ProgBar(Color::Up_Yellow, &wait_progress);

  // 等待 6 秒
  float wait_time_s = 0.0f;
  while (wait_time_s < 6.0f)
  {
    wait_time_s += 0.05f;
    wait_progress = wait_time_s / 6.0f;
    Seq::Wait(0.05f);
  }

  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  APP::jaw.CloseClaw();
  APP::jaw.SetTrackMode(false);

  // 只有没有放弃对接时，才将矛头回中。否则害怕损坏机构
  if (!giveup_dock)
  {
    APP::jaw.SetPosCode(APP::jaw._hori_origin_code - JawConst::_hori_max_travel_code / 2 - 100000,
      APP::jaw._vert_origin_code - 226000);
  }

  APP::monit.LogInfo("Go to Climb.");

  // 只有 "没取够头" / "不希望放弃对接" 时，才会再次取矛头。否则就直接进入二区
  if (need_spear_id < 5 && !giveup_dock)
  {
    need_spear_id++;
    APP::comm_center.ClearDockFlag();  // 清除对接标志，避免下次误触发
    fetch_again = true;
  }
  else
  {
    APP::comm_center.ClearDockFlag();  // 清除对接标志，避免下次误触发
    fetch_again = false;
    move_to_area2 = true;
  }
}

/*******************************************************************************/

void Goto_Area2(StateCore *core)
{
  APP::main_loop.test_led.Lit(Color::Red);

  // 确认工控机已经发送了动作序列
  Seq::WaitUntil([](){
    return Area2::mei_lin_array.host_a2act_recved;
  });

  APP::main_loop.test_led.Lit(Color::White);

  // 首先，根据工控机的要求，走到相应的块的面前
  switch (Area2::mei_lin_array.enter_code)
  {
    case Area2Code::ENTER_0:
      MOVE::MoveToTargPos(A12_B3);
      break;
    case Area2Code::ENTER_1:
      MOVE::MoveToTargPos(A12_B2);
      break;
    case Area2Code::ENTER_2:
      MOVE::MoveToTargPos(A12_B1);
      break;
    default:
      APP::monit.LogError("Invalid A2EnterCode!");
      Seq::Wait(3600.0f);
      break;
  }

  APP::comm_center.SlamCorrection();

  Seq::Wait(1.0f);

  APP::monit.LogInfo("sick_pos:%.2f,%.2f,%.2f", APP::comm_center.slam_pos.x,
  APP::comm_center.slam_pos.y, APP::comm_center.slam_pos.z);

  APP::monit.LogInfo("I'm Ahead of MeiLin.");

  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }
  
  core->GetCurState().Complete = true;
}

/*******************************************************************************/

/**
 * @brief 基于工控机发来的动作指令序列，来执行梅林的动作
 * 
 * @param core 
 */
void Area2_HostAction(StateCore *core)
{
  APP::monit.LogInfo("Now, Start Area2 Actions.");

  // 此时走到了梅林跟前，准备执行二区序列动作
  Area2::mei_lin_array.ExecuteSequence();

  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  core->GetCurState().Complete = true;
}

/*******************************************************************************/

/**
 * @brief 技能挑战赛——“崇武探幽”结束后，可以选择自动回到原点
 * 
 * @param core 
 */
void Area2End_BackOrigin(StateCore *core)
{
  // 再次确认，确保安全
  APP::monit.LogInfo("Please Confirm Again: Back to Origin?");
  APP::main_loop.test_led.GradientFlow(Color::Up_Yellow, Color::Up_Purpor, 1.25f);

  Seq::Wait(1.0f);

  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  // 回到原点
  APP::monit.LogInfo("Back to Origin...");

  // 走到原点
  MOVE::MoveToTargGes(Vec3(8.7f, 0.55f, 0.0f));
  MOVE::MoveToTargGes(Vec3(2.45f, 0.55f, 0.0f));
  MOVE::MoveToTargGes(Vec3(0.5f, 1.39f, 0.0f));

  // 成功回到原点
  APP::monit.LogOK("All Process Done. Congratulations!");

  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  core->GetCurState().Complete = true;
}

/*******************************************************************************/

const static float max_farcon_alpha = 1.8f; // 最大加速度 m/s^2
const static float max_farcon_beta = 1.8f; // 最大角加速度 rad/s^2
static void Response_Button()
{
    static float max_velo = 1.5f;
    if(MOD::farcon.toggle[2] == 0)
    {
      max_velo = 1.0f;
    }
    else if(MOD::farcon.toggle[2] == 1)
    {
      max_velo = 0.33f;
    }

    // 计算遥控器死区：0-15不算，将15-100映射到原来的0.0-1.0
    constexpr float deadzone = 15.0f;
    auto map_axis = [deadzone](int v) -> float {
      float a = (v >= 0) ? float(v) : -float(v);
      if (a <= deadzone) return 0.0f;
      return (v > 0 ? 1.0f : -1.0f) * ((a - deadzone) / (100.0f - deadzone));
    };

    Vec3 vel;
    // 计算速度
    vel.x = -map_axis(MOD::farcon.jys_value[3]) * max_velo; // 最大速度0.5m/s
    vel.y = -map_axis(MOD::farcon.jys_value[2]) * max_velo; // 最大速度0.5m/s
    vel.z = -map_axis(MOD::farcon.jys_value[0]) * max_velo; // 最大角速度1.0rad/s

    // 简单加速度限制：避免遥控器输入突变导致底盘速度跳变
    static Vec3 last_vel(0, 0, 0);
    static float last_accel_time = DWT_GetTimeline_Sec();
    float now = DWT_GetTimeline_Sec();
    float dt = now - last_accel_time;
    if (dt > 0.05f) dt = 0.05f;
    if (dt < 0.0f) dt = 0.0f;
    last_accel_time = now;

    auto limit_delta = [](float delta, float max_delta) -> float {
      if (delta > max_delta) return max_delta;
      if (delta < -max_delta) return -max_delta;
      return delta;
    };

    vel.x = last_vel.x + limit_delta(vel.x - last_vel.x, max_farcon_alpha * dt);
    vel.y = last_vel.y + limit_delta(vel.y - last_vel.y, max_farcon_alpha * dt);
    vel.z = last_vel.z + limit_delta(vel.z - last_vel.z, max_farcon_beta * dt);
    last_vel = vel;

    if (vel.Length() > 0.03f)
    {
      // 向底盘发送移动指令
      APP::chas.Move(vel, 0.2f);
    }
    else
    {
      // 如果摇杆没有动作，则发送零速度指令
      APP::chas.Move(Vec3(0, 0, 0), 0.2f);
    }

    static float last_trigger_time = 0;
    if(MOD::farcon.button_second_half[7] == 1 && DWT_GetTimeline_Sec() - last_trigger_time > 1.0f)
    {
      static bool is_daji_enable = false;
      if(is_daji_enable)
      {
        // 打开后边小轮(妲己)
        APP::leg.relay_daji.Off();
      }
      else
      {
        APP::leg.relay_daji.On();
      }
 
      is_daji_enable = !is_daji_enable;
      last_trigger_time = DWT_GetTimeline_Sec();
    }

    if(MOD::farcon.button_second_half[4] == 1)
    {
      // 抬升达妙
      APP::get_block.motor_dm_lift.SetPosVel(-760, 15.0f);
    }

    if(MOD::farcon.button_second_half[5] == 1)
    {
      // 放下达妙
      APP::get_block.motor_dm_lift.SetPosVel(0.0f, 15.0f);
    }

    /**************   均为左半硬件  *************/
    // 抬腿
    if (MOD::farcon.button_first_half[0] == 1)
    {
      APP::leg.SetFrontPosSlow(-390.0f, 5.0f);
      APP::leg.SetBackPosSlow(-390.0f, 5.0f);
    }

    // 收回前腿
    if (MOD::farcon.button_first_half[1] == 1)
    {
      APP::leg.SetFrontPosSlow(-4, 5.0f);
    }

    // 收回后腿
    if (MOD::farcon.button_first_half[2] == 1)
    {
      APP::leg.SetBackPosSlow(-4, 5.0f);
    }

    // 一起收回
    if (MOD::farcon.button_first_half[3] == 1)
    {
      APP::leg.SetFrontPosSlow(-4, 5.0f);
      APP::leg.SetBackPosSlow(-4, 5.0f);
    }
}

/*-----------------------------------------------------------------
 * @brief 下方是之前测试用的状态代码，不用于正式比赛
 * 
 *----------------------------------------------------------------*/

 /**
 * @brief 实际情况中，需要算法组指定 分支走向。
 * 这里是调试，所以简化了情况。假设直接走中间
 * 
 * @param core 
 */
void Walk_into_Area2(StateCore *core)
{
  if (!MOVE::MoveToTargPos(A12_2))
  {
    APP::monit.LogError("[SemiAuto] Walk_into_Area2: path rejected.");
    Seq::Wait(3600.0f);
    return;
  }

  // 此时走到了梅林跟前
  APP::main_loop.test_led.Breath(Color::White, 2.0f);

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  APP::monit.LogInfo("Try to Climb.");
  // 来自文件：Area2_Act.hpp。里边主要是一些二区的行为
  Area2::mei_lin_array.Climb_Up200();

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  Area2::mei_lin_array.Climb_Up200();

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  Area2::mei_lin_array.Climb_Up200();

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  Area2::mei_lin_array.Decend_200();

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  Area2::mei_lin_array.Decend_400();

  // 避免西克打出去了
  Seq::Wait(3.00f);
  APP::comm_center.SlamCorrection();
  Seq::Wait(3.00f);
  APP::monit.LogInfo("Pos corrected: (%.3f, %.3f, %.3f)",
  System.position.x, System.position.y, System.position.z);

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  APP::monit.LogInfo("Now, Go to Area3--ARENA");

  core->GetCurState().Complete = true;
}
