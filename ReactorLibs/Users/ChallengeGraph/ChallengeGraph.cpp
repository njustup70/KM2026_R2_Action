#include "ChallengeGraph.hpp"
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
// #include "r2_area1_1.hpp"
#include "ttd_1.hpp"
#include "ttd_2.hpp"
#include "ttd_3.hpp"

#include "stt_r.hpp"
#include "stt_rl_2.hpp"
#include "a12_2.hpp"
#include "a23_2.hpp"

void Response_Button();

/// @brief 当前需要取的矛头编号
static uint8_t need_spear_id = 0;

static bool fetch_again = false;    // 是否需要再次取矛头
static bool move_to_area2 = false;  // 是否可以移动到二区了


// 自动状态机
StateGraph R2_Challenge_Graph("R2_Challenge");
static void Waiting_for_Start(StateCore *core);
static void Walk_Fetch(StateCore *core);
static void Walk_To_Dock(StateCore *core);
void Dock_Fetch_And_Back(StateCore *core);

static void Walk_into_Area2(StateCore *core);
static void Walk_into_Area3(StateCore *core);

void Config_R2_Challenge_StateGraph()
{
  StateBlock& s_wait = R2_Challenge_Graph.AddState("wait");
  StateBlock& s_area1_1 = R2_Challenge_Graph.AddState("area1_1");
  StateBlock& s_area1_2 = R2_Challenge_Graph.AddState("area1_2");
  StateBlock& s_dock_fetch = R2_Challenge_Graph.AddState("dock_fetch");
  StateBlock& s_walk_from_a1_to_a2 = R2_Challenge_Graph.AddState("walk_a1_to_a2");
  StateBlock& s_walk_from_a2_to_a3 = R2_Challenge_Graph.AddState("walk_a2_to_a3");

  s_wait.StateAction = Waiting_for_Start;
  s_area1_1.StateAction = Walk_Fetch;
  s_area1_2.StateAction = Walk_To_Dock;
  s_dock_fetch.StateAction = Dock_Fetch_And_Back;
  s_walk_from_a1_to_a2.StateAction = Walk_into_Area2;
  s_walk_from_a2_to_a3.StateAction = Walk_into_Area3;

  s_wait.LinkTo(&s_wait.Complete, s_area1_1);
  s_area1_1.LinkTo(&s_area1_1.Complete, s_area1_2);
  s_area1_2.LinkTo(&s_area1_2.Complete, s_dock_fetch);

  s_dock_fetch.LinkTo(&fetch_again, s_dock_fetch);
  s_dock_fetch.LinkTo(&move_to_area2, s_walk_from_a1_to_a2);
  
  s_walk_from_a1_to_a2.LinkTo(&s_walk_from_a1_to_a2.Complete, s_walk_from_a2_to_a3);
  s_walk_from_a2_to_a3.LinkTo(&s_walk_from_a2_to_a3.Complete, s_wait);

  // 向状态机核心注册
  APP::state_core.RegistGraph(R2_Challenge_Graph);
  APP::state_core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
}

/**
 * @brief 刚开机时，等待出发信号的状态
 * 
 * @details 现在先设置成等待一段时间吧，后续可以改成比如 等到自检通过了，才允许动之类的
 */
void Waiting_for_Start(StateCore *core)
{
  APP::jaw.CloseClaw();
  APP::monit.LogInfo("Waiting for start...");
  APP::main_loop.test_led.Breath(Color::White, 4.0f);

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  // APP::main_loop.test_led.GradientFlow(Color::Up_Purpor * 0.3f, Color::Up_Yellow * 0.3f, 1.0f);
  APP::main_loop.test_led.Running(Color::White, 0.3f, 2.0f);

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
  APP::leg.F_AlmostOrigin();
  APP::leg.B_AlmostOrigin();
  
  // 等待遥控器的出发指令
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
  APP::leg.SetFrontPos(-10);
  APP::leg.SetBackPos(-10);

  APP::main_loop.test_led.GradientFlow(Color::Up_Purpor * 0.65f, Color::Up_Yellow * 0.65f, 1.0f);
  
  // 从原点出发取矛头
  FetchSpear_FromOrigin(0);

  core->GetCurState().Complete = true;
}

/**
 * @brief 一区第二个动作，取到矛头后，转身走到对接位置，准备对接
 * 
 * @param core 
 */
void Walk_To_Dock(StateCore *core)
{
  // 走到对接点
  GotoDock(0);

  // 进入对接模式
  APP::jaw.SetTrackMode(true);

  // 等待遥控器指令，然后才进入下一个状态
  APP::main_loop.test_led.Expand(Color::Up_Yellow, 1.2f);

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

  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  APP::jaw.CloseClaw();

  APP::monit.LogInfo("Go to Climb.");

  core->GetCurState().Complete = true;
}



/**
 * @brief 后续的从对接点取矛头并返回的逻辑
 * 
 * @param core 
 */
void Dock_Fetch_And_Back(StateCore *core)
{
  fetch_again = false;

  APP::main_loop.test_led.GradientFlow(Color::Up_Purpor * 0.65f, Color::Up_Yellow * 0.65f, 1.0f);

  FetchSpear_FromDock(need_spear_id);

  /**--------------     返回对接点    ---------------**/

  // 走到对接点
  GotoDock(need_spear_id);

  // 进入对接模式
  APP::jaw.SetTrackMode(true);

  // 等待遥控器指令，然后才进入下一个状态
  APP::main_loop.test_led.Expand(Color::Up_Yellow, 1.2f);

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

  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  APP::jaw.CloseClaw();

  APP::monit.LogInfo("Go to Climb.");

  if (need_spear_id < 2)
  {
    need_spear_id++;
    fetch_again = true;
  }
  else
  {
    fetch_again = false;
    move_to_area2 = true;
  }
}




/**
 * @brief 实际情况中，需要算法组指定 分支走向。
 * 这里是调试，所以简化了情况。假设直接走中间
 * 
 * @param core 
 */
void Walk_into_Area2(StateCore *core)
{
  // if (!MOVE::MoveToTargPos(A12_2))
  // {
  //   APP::monit.LogError("[SemiAuto] Walk_into_Area2: path rejected.");
  //   Seq::Wait(3600.0f);
  //   return;
  // }
  MOVE::MoveToTargGes(Vec3(2.5f, 3.0f, 0.0f));
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
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  Climb_Up200();

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  Climb_Up200();

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  Decend_200();

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  Decend_400();

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

/// @brief 伸爪取块
/// @param 
static void FetchBlock()
{
  Seq::Wait(1.0f);

  // 伸出夹爪
  APP::vac_claw.StretchOut();

  Seq::Wait(0.5f);

  APP::get_block.OpenPump();

  Seq::Wait(1.5f);

  APP::vac_claw.StretchBack();

  Seq::Wait(0.5f);

  // 这里应该加负压泵的阀，快速放块

  APP::get_block.ClosePump();
}

/// @brief 转向指定角度（相对当前朝向）
/// @param delta_rad 正值=逆时针，负值=顺时针
static void TurnRelative(float delta_rad)
{
    float target_yaw = APP::chas.chas_odom.pos.z + delta_rad;
    APP::path_chaser.ChaseYaw(target_yaw);

    while (!APP::path_chaser.IsFinished())
    {
        APP::chas.Move(APP::path_chaser.GetCmdBody(), 0.01f);
        Seq::Wait(0.005f);
    }
}

void Walk_into_Area2_Host(StateCore *core)
{
  if (!MOVE::MoveToTargPos(A12_2))
  {
    APP::monit.LogError("[SemiAuto] Walk_into_Area2_Host: path rejected.");
    Seq::Wait(3600.0f);
    return;
  }

  // 此时走到了梅林跟前

  // ====== 二区功能码 ======

  // ====== 工控机方发送 ======
  static constexpr uint8_t CMD_UP_200     = 0x64;
  static constexpr uint8_t CMD_DOWN_200   = 0x65;
  static constexpr uint8_t CMD_UP_400     = 0x66;
  static constexpr uint8_t CMD_DOWN_400   = 0x67;
  static constexpr uint8_t CMD_TURN_CW    = 0x68; // 正方向转 PI/2
  static constexpr uint8_t CMD_TURN_CCW   = 0x69; // 负方向转 PI/2
  static constexpr uint8_t CMD_GET_BLOCK  = 0x6A;
  static constexpr uint8_t CMD_AREA_DONE  = 0x6B;   // ← 二区流程完成→退出
  // ====== A板发送 ======
  static constexpr uint8_t ACK_DONE       = 0x6C;   // ← 某动作完成

  static volatile uint8_t s_cmd = 0;
  static volatile bool s_pending = false;

  // 注册回调（只一次）
  static bool s_reg = false;
  if (!s_reg)
  {
    auto handler = [](uint8_t func_code, const uint8_t*, uint8_t, void*) {
      s_cmd = func_code;
      s_pending = true;
    };
    MOD::host_pc.Regist(CMD_UP_200,    handler);
    MOD::host_pc.Regist(CMD_DOWN_200,  handler);
    MOD::host_pc.Regist(CMD_UP_400,    handler);
    MOD::host_pc.Regist(CMD_DOWN_400,  handler);
    MOD::host_pc.Regist(CMD_TURN_CW,   handler);
    MOD::host_pc.Regist(CMD_TURN_CCW,  handler);
    MOD::host_pc.Regist(CMD_GET_BLOCK, handler);
    MOD::host_pc.Regist(CMD_AREA_DONE, handler);
    s_reg = true;
  }

  APP::monit.LogInfo("[Area2] Waiting for commands from host...");

  // ====== 命令响应主循环 ======
  while (true)
  {
    // 留个遥控器保险
    Response_Button();

    if (s_pending)
    {
      s_pending = false;

      // 收到 0x6B → 二区流程完成，退出
      if (s_cmd == CMD_AREA_DONE)
      {
        APP::monit.LogInfo("[Area2] Area2 done, exit");
        break;
      }

      APP::monit.LogInfo("[Area2] Cmd=0x%02X received", s_cmd);

      switch (s_cmd)
      {
        case CMD_UP_200:    Climb_Up200();            break;
        case CMD_DOWN_200:  Decend_200();              break;
        case CMD_UP_400:    Climb_Up400();             break;
        case CMD_DOWN_400:  Decend_400();              break;
        case CMD_TURN_CW:   TurnRelative(PI/2);  break;
        case CMD_TURN_CCW:  TurnRelative(-PI/2);    break;
        case CMD_GET_BLOCK: /* 这个需要把爪和取块库封装一下 */   break;
        default:
          APP::monit.LogWarning("[Area2] Unknown cmd=0x%02X", s_cmd);
          break;
      }

      // 动作完成 → 发 0x6C
      MOD::host_pc.SendAck(ACK_DONE, 0); // 0表示成功
      APP::monit.LogInfo("[Area2] Cmd=0x%02X Complet", s_cmd);
    }

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

  // 一步一步跑，先测跑点
  MOVE::MoveToTargGes(Vec3(8.65f, 0.77f, 0.0f));

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  // 等待下一个指令
  MOVE::MoveToTargGes(Vec3(11.45f, 0.77f, 0.0f));

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  // 等待下一个指令
  MOVE::MoveToTargGes(Vec3(10.75f, 5.4f, 1.57f));

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

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
