#include "ChlgArenaPK_Graph.hpp"
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
static void Response_Button();

StateGraph Arena_PK_Graph("Arena_PK");

// 状态定义
static void Waiting_for_Start(StateCore *core);
static void Put_First_Block(StateCore *core);
static void Waiting_R1_Instru(StateCore *core);

/**
 * @brief 技能挑战赛：“九宫藏宝” 状态图初始化
 * 
 */
int ChallengeArenaPK_Graph()
{
  StateBlock& s_wait = Arena_PK_Graph.AddState("wait");         // 等待开始

  // 从原点出发，走到九宫格擂台前，准备放块
  StateBlock& s_put_first_block = Arena_PK_Graph.AddState("put_first_block");
  // 等待 R1 的指令
  StateBlock& s_waiting_R1_instru = Arena_PK_Graph.AddState("waiting_R1");       // to dock
  // （1）放指定的二层块
  StateBlock& s_put_L2_block = Arena_PK_Graph.AddState("put_L2_block");   // fetch and dock again
  // （2）从 R1 身上拿块
  StateBlock& s_fetch_on_R1 = Arena_PK_Graph.AddState("walk_a12");
  // （3）合体
  StateBlock& s_combine = Arena_PK_Graph.AddState("combine");  // 二区由工控机控制的动作序列
  // （4）放三层块——仅在合体后有效
  StateBlock& s_put_L3_block = Arena_PK_Graph.AddState("put_L3_block");  // 二区结束后，回到原点

  /**----------     配置状态行为函数    ----------**/
  s_wait.StateAction = Waiting_for_Start;
  s_put_first_block.StateAction = Put_First_Block;
  s_waiting_R1_instru.StateAction = Waiting_R1_Instru;

  /**----------     进行状态间链接    ----------**/
  s_wait.LinkTo(&(s_wait.Complete), s_put_first_block);
  s_put_first_block.LinkTo(&(s_put_first_block.Complete), s_waiting_R1_instru);

  /**----------     向状态机核心注册    ----------**/
  return APP::state_core.RegistGraph(Arena_PK_Graph);
}

/*******************************************************************************/

/**
 * @brief 刚开机时，等待出发信号的状态
 * 
 * @details 现在先设置成等待一段时间吧，后续可以改成比如 等到自检通过了，才允许动之类的
 */
void Waiting_for_Start(StateCore *core)
{
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
 * @brief 这是“九宫藏宝”技能挑战赛，R2出发的第一个动作。先走到九宫格前，然后放块。如果块已经被放置，则换一个方格放块。
 * 
 * @param core 
 */
void Put_First_Block(StateCore *core)
{
  // 灯带：滚动
  APP::main_loop.test_led.Running(Color::Up_Yellow * 0.75f, 0.5f, 1.0f);
  
  // 首先走上斜坡
  MOVE::MoveToTargGes(Vec3(11.45f, 0.77f, 0.0f));

  // 然后走到九宫格前
  MOVE::MoveToTargGes(Vec3(11.45f, 2.45f, 0.0f));
  MOVE::MoveToTargGes(Vec3(10.75f, 5.30f, 1.571f));

  // 等待，保障稳定性
  Seq::Wait(1.0f);

  // 抬起腿
  APP::leg.F_Down_400();
  APP::leg.B_Down_400();
  // 等腿抬好
  Seq::Wait(1.5f);

  // 检测里边有没有块，如果有，换另一个格子
  // if (APP::get_block.IsBlockPresent())
  // {
  //   // 块存在，执行相应操作
  // }

  // 放块
  APP::get_block.Put();

  // 等待放块完成
  Seq::Wait(1.0f);

  // 走到等待点，等待R1命令
  core->GetCurState().Complete = true;
}

/*******************************************************************************/

/**
 * @brief 此时，R2靠边站好。并将光通信 / 红外接收端向外对准，准备接收信息。并路由至不同状态
 * @note 这个函数里如果没有阻塞，那么根据状态机的设计，这个状态将会以 200Hz 循环
 * @param core 
 */
void Waiting_R1_Instru(StateCore *core)
{
  /**
   * @brief 根据R1的指令，路由至不同状态
   * 
   */
  switch (APP::comm_center.Get_R1_InstruCode())
  {
    case Area3ActCode::Put_L2_N4_Block:
    {
      break;
    }
    case Area3ActCode::Put_L2_N5_Block:
    {
      break;
    }
    case Area3ActCode::Put_L2_N6_Block:
    {
      break;
    }
    case Area3ActCode::Put_L3:
    {
      break;
    }
    case Area3ActCode::Combine:
    {
      break;
    }
    case Area3ActCode::Fetch_GroundBlock_0:
    {
      break;
    }
    case Area3ActCode::Fetch_GroundBlock_1:
    {
      break;
    }
    default:
    {
      break;
    }
  }
}

/*******************************************************************************/
























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

    // 向底盘发送移动指令
    APP::chas.Move(vel, 0.2f);

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
      // 抬升达妙，向上为负
      APP::get_block.motor_dm_lift.SetPosVel(-750, 15.0f);
    }

    if(MOD::farcon.button_second_half[5] == 1)
    {
      // 放下达妙
      APP::get_block.motor_dm_lift.SetPosVel(0.0f, 15.0f);
    }
}
