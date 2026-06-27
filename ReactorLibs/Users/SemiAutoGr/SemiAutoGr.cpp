#include "SemiAutoGr.hpp"
#include "Monitor.hpp"
#include "Jaw.hpp"
#include "Farcon.hpp"
#include "Chassis.hpp"
#include "Lift_Leg.hpp"
#include "HostPC.hpp"
#include "PathChaser.hpp"
#include "VacuumClaw.hpp"
#include "Area2_Act.hpp"
#include "AutoGraph.hpp"
#include "CommCenter.hpp"
#include "Get_Block.hpp"

/// @brief 所有的路径文件
// #include "r2_area1_1.hpp"
#include "table_to_dock.hpp"
#include "stt.hpp"
#include "stt_r.hpp"
#include "a12_2.hpp"
#include "a23_2.hpp"

void Response_Button();

// 自动状态机
StateGraph SemiAutoGr("SemiAuto");
static void Waiting_for_Start(StateCore *core);
static void Walk_Fetch(StateCore *core);
static void Walk_To_Dock(StateCore *core);
static void Walk_into_Area2(StateCore *core);
static void Walk_into_Area3(StateCore *core);

void Config_SemiAutoStateGraph()
{
  StateBlock& s_wait = SemiAutoGr.AddState("wait");
  StateBlock& s_area1_1 = SemiAutoGr.AddState("area1_1");
  StateBlock& s_area1_2 = SemiAutoGr.AddState("area1_2");
  StateBlock& s_walk_from_a1_to_a2 = SemiAutoGr.AddState("walk_a1_to_a2");
  StateBlock& s_walk_from_a2_to_a3 = SemiAutoGr.AddState("walk_a2_to_a3");

  s_wait.StateAction = Waiting_for_Start;
  s_area1_1.StateAction = Walk_Fetch;
  s_area1_2.StateAction = Walk_To_Dock;
  s_walk_from_a1_to_a2.StateAction = Walk_into_Area2;
  s_walk_from_a2_to_a3.StateAction = Walk_into_Area3;

  s_wait.LinkTo(&s_wait.Complete, s_area1_1);
  s_area1_1.LinkTo(&s_area1_1.Complete, s_area1_2);
  s_area1_2.LinkTo(&s_area1_2.Complete, s_walk_from_a1_to_a2);
  s_walk_from_a1_to_a2.LinkTo(&s_walk_from_a1_to_a2.Complete, s_walk_from_a2_to_a3);
  s_walk_from_a2_to_a3.LinkTo(&s_walk_from_a2_to_a3.Complete, s_wait);

  // 向状态机核心注册
  APP::state_core.RegistGraph(SemiAutoGr);
  APP::state_core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
}

/**
 * @brief 刚开机时，等待出发信号的状态
 * 
 * @details 现在先设置成等待一段时间吧，后续可以改成比如 等到自检通过了，才允许动之类的
 */
void Waiting_for_Start(StateCore *core)
{
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  APP::monit.LogInfo("Waiting for start...");

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

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

  // APP::jaw.CloseClaw(); 
  
  // 等待遥控器的出发指令
  // Seq::WaitUntil([](){
  //   return MOD::farcon.button_second_half[0] == 1;
  // });

  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

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
  APP::jaw.OpenClaw();

  MOVE::MoveToTargPos(STT_R);
  
  APP::jaw.CloseClaw();

  // 记录抓取位置，以及西克数据
  APP::monit.LogInfo("Catch At:(%.2f,%.2f), sick:%.3f", APP::comm_center.slam_pos.x,
    APP::comm_center.slam_pos.y, APP::comm_center._sick_ch0_data);

  // 先往上抬 1.5cm
  APP::jaw.ChangePos(0, 15);
  Seq::Wait(0.5);
  // 然后抬起 Pitch爪子
  APP::jaw.PitchUp();
  Seq::Wait(0.5);

  // 等待遥控器指令，然后才进入下一个状态
  // Seq::WaitUntil([]()
  // {
  //   return MOD::farcon.button_first_half[0] == 1;
  // });
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

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
  // Seq::WaitUntil([]()
  // {
  //   return MOD::farcon.button_first_half[0] == 1;
  // });

  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);

    if (APP::comm_center.IsDocked())
    {
      break;
    }
  }

  // 打开夹爪，给R1时间拿回兵器
  APP::jaw.OpenClaw();    // 依旧OpenClaw
  Seq::Wait(2.0f);

  APP::monit.LogInfo("Go to Climb.");

  core->GetCurState().Complete = true;
}

/**
 * @brief 实际情况中，需要算法组指定 分支走向。
 * 这里是调试，所以简化了情况。假设直接走中间
 * 
 * @param core 
 */
void Walk_into_Area2(StateCore *core)
{
  MOVE::MoveToTargPos(Area1_to2_Line2);

  // 此时走到了梅林跟前

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  APP::monit.LogInfo("Try to Climb.");
  // 来自文件：Area2_Act.hpp。里边主要是一些二区的行为
  Climb_Up200();

  // 等待遥控器指令，然后才进入下一步
  // while(MOD::farcon.button_second_half[0] != 1)
  // {
  //   Response_Button();
  //   Seq::Wait(0.01f);
  // }

  Climb_Up200();

  // 等待遥控器指令，然后才进入下一步
  // while(MOD::farcon.button_second_half[0] != 1)
  // {
  //   Response_Button();
  //   Seq::Wait(0.01f);
  // }

  Climb_Up200();

  // 等待遥控器指令，然后才进入下一步
  // while(MOD::farcon.button_second_half[0] != 1)
  // {
  //   Response_Button();
  //   Seq::Wait(0.01f);
  // }

  Decend_200();

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  Decend_400();

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  APP::monit.LogInfo("Now, Go to Area3--ARENA");

  core->GetCurState().Complete = true;
}


void Walk_into_Area3(StateCore *core)
{
  // 跑到Arena九宫格跟前
  APP::leg.SetFrontPos(-40);
  APP::leg.SetBackPos(-40);
  MOVE::MoveToTargPos(Area2_to3_Line2);

  // 抬腿但是（还不知道该抬多高）
  APP::leg.B_Down_400();
  APP::leg.F_Down_400();

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  APP::get_block.Put();
}


void Response_Button()
{
    static float max_velo = 1.5f;
    if(MOD::farcon.toggle[2] == 0)
    {
      max_velo = 1.5f;
    }
    else if(MOD::farcon.toggle[2] == 1)
    {
      max_velo = 5.0f;
    }
 
    Vec3 vel;
    // 计算速度
    vel.x = (-MOD::farcon.jys_value[3] / 100.0f) * max_velo; // 最大速度0.5m/s
    vel.y = (-MOD::farcon.jys_value[2] / 100.0f) * max_velo;  // 最大速度0.5m/s
    vel.z = (-MOD::farcon.jys_value[0] / 100.0f) * max_velo;  // 最大角速度1.0rad/s

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
}