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
#include "Area1_Act.hpp"
#include "CommCenter.hpp"
#include "Get_Block.hpp"
#include "MainLoop.hpp"
#include "PathCenter/PathCenter.hpp"

/// @brief 所有的路径文件
// #include "r2_area1_1.hpp"
#include "ttd_3.hpp"

#include "a12_1.hpp"

static void Response_Button();

// 自动状态机
StateGraph SemiAutoGr("SemiAuto");
static void Waiting_for_Start(StateCore *core);
static void Walk_Fetch(StateCore *core);
static void Walk_To_Dock(StateCore *core);
static void Walk_into_Area2(StateCore *core);
static void Walk_into_Area3(StateCore *core);

int Config_SemiAutoStateGraph()
{
  StateBlock& s_wait = SemiAutoGr.AddState("wait");
  // StateBlock& s_area1_1 = SemiAutoGr.AddState("area1_1");
  // StateBlock& s_area1_2 = SemiAutoGr.AddState("area1_2");
  StateBlock& s_walk_from_a1_to_a2 = SemiAutoGr.AddState("walk_a1_to_a2");
  StateBlock& s_walk_from_a2_to_a3 = SemiAutoGr.AddState("walk_a2_to_a3");

  s_wait.StateAction = Waiting_for_Start;
  // s_area1_1.StateAction = Walk_Fetch;
  // s_area1_2.StateAction = Walk_To_Dock;
  s_walk_from_a1_to_a2.StateAction = Walk_into_Area2;
  s_walk_from_a2_to_a3.StateAction = Walk_into_Area3;

  s_wait.LinkTo(&s_wait.Complete, s_walk_from_a1_to_a2);
  s_walk_from_a1_to_a2.LinkTo(&s_walk_from_a1_to_a2.Complete, s_walk_from_a2_to_a3);
  s_walk_from_a2_to_a3.LinkTo(&s_walk_from_a2_to_a3.Complete, s_wait);

  // 向状态机核心注册
  return APP::state_core.RegistGraph(SemiAutoGr);
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
  Seq::Wait(2.0f);

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

  // 取矛头
  FetchSpear_FromOrigin(2);

  core->GetCurState().Complete = true;
}

/**
 * @brief 一区第二个动作，取到矛头后，转身走到对接位置，准备对接
 * 
 * @param core 
 */
void Walk_To_Dock(StateCore *core)
{
  // 先向前走一点

  // 走到对接点
  MOVE::MoveToTargPos(TTD_3);

  // 进入对接模式
  APP::jaw.SetTrackMode(true);
  APP::main_loop.test_led.Breath(Color::Up_Purpor, 2.4f);

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
  APP::jaw.CloseClaw();

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
  MOVE::MoveToTargGes(Vec3(2.4f, 3.0f, 0.0f));
  // 此时走到了梅林跟前

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  APP::monit.LogInfo("Try to Climb.");
  // 来自文件：Area2_Act.hpp。里边主要是一些二区的行为
  // APP::vac_claw.need_to_fetch = true;
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
  // APP::comm_center.SlamCorrection();
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

static void Walk_into_Area2_Host(StateCore *core)
{
  if (!MOVE::MoveToTargPos(PATH::A12_2()))
  {
    APP::monit.LogError("[SemiAuto] Walk_into_Area2_Host: path rejected.");
    Seq::Wait(3600.0f);
    return;
  }

  // 此时走到了梅林跟前，准备执行二区序列动作
  Area2::mei_lin_array.ExecuteSequence();

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

  // 跑上三区的平台
  MOVE::MoveToTargGes(Vec3(11.45f, 0.77f, 0.0f));

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  // 跑到中间的格子放
  // MOVE::MoveToTargPos(A3_To_Arena2);

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  // 抬腿但是（还不知道该抬多高）
  APP::leg.B_Down_400();
  APP::leg.F_Down_400();

  Seq::Wait(5.0f);

  // 等待遥控器指令，然后才进入下一步
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  APP::get_block.Put();

  Seq::Wait(999.9f);
}


static void Response_Button()
{
    static float max_velo = 1.5f;
    // if(MOD::farcon.toggle[2] == 0)
    // {
    //   max_velo = 1.0f;
    // }
    // else if(MOD::farcon.toggle[2] == 1)
    // {
    //   max_velo = 0.33f;
    // }

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
      APP::get_block.LiftUp();
    }

    if(MOD::farcon.button_second_half[5] == 1)
    {
      // 放下达妙
      APP::get_block.LiftDown();
    }
}

const static float max_farcon_alpha = 1.8f; // 最大加速度 m/s^2
const static float max_farcon_beta = 1.8f; // 最大角加速度 rad/s^2
void Response_Button_main()
{
    static float max_velo = 1.5f;
    if(MOD::farcon.toggle[2] == 0)
    {
      max_velo = 1.5f;
    }
    else if(MOD::farcon.toggle[2] == 1)
    {
      max_velo = 0.5f;
    }

    // 计算遥控器死区：0-15不算，将15-100映射到原来的0.0-1.0
    constexpr float deadzone = 15.0f;
    auto map_axis = [deadzone](int v) -> float {
      float a = (v >= 0) ? float(v) : -float(v);
      if (a <= deadzone) return 0.0f;
      return (v > 0 ? 1.0f : -1.0f) * ((a - deadzone) / (100.0f - deadzone));
    };

    // 计算目标速度（摇杆直接映射到的期望速度）
    Vec3 target_vel;
    target_vel.x = -map_axis(MOD::farcon.jys_value[3]) * max_velo;
    target_vel.y = -map_axis(MOD::farcon.jys_value[2]) * max_velo;
    target_vel.z = -map_axis(MOD::farcon.jys_value[0]) * max_velo;

    // 加速度限制：平滑过渡到目标速度
    static Vec3 last_vel(0, 0, 0);
    static float last_accel_time = DWT_GetTimeline_Sec();

    float now = DWT_GetTimeline_Sec();
    float dt = now - last_accel_time;
    // 限制最大 dt，防止长时间阻塞后速度跳变
    if (dt > 0.05f) dt = 0.05f;
    if (dt < 0.0f)  dt = 0.0f;  // 防止时间回绕
    last_accel_time = now;

    // 限幅函数
    auto limit_delta = [](float delta, float max_delta) -> float {
        if (delta > max_delta) return max_delta;
        if (delta < -max_delta) return -max_delta;
        return delta;
    };

    Vec3 vel;
    vel.x = last_vel.x + limit_delta(target_vel.x - last_vel.x, max_farcon_alpha * dt);
    vel.y = last_vel.y + limit_delta(target_vel.y - last_vel.y, max_farcon_alpha * dt);
    vel.z = last_vel.z + limit_delta(target_vel.z - last_vel.z, max_farcon_beta  * dt);
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

    // 处理泄压阀的开关
    static float last_prv_time = 0;
    if(MOD::farcon.button_second_half[1] == 1 && DWT_GetTimeline_Sec() - last_prv_time > 1.0f)
    {
      static bool is_PRV_open = false;
      if(!is_PRV_open)
      {
        APP::get_block.OpenPRV();
        is_PRV_open = true;
      }
      else
      {
        APP::get_block.ClosePRV();
        is_PRV_open = false;
      }
      last_prv_time = DWT_GetTimeline_Sec();
    }

    // 处理抬升平台的升降
    static float last_lift_time = 0;
    if(MOD::farcon.button_second_half[3] == 1 && DWT_GetTimeline_Sec() - last_lift_time > 1.0f)
    {
      static bool is_lift_up = false;
      if(!is_lift_up)
      {
        APP::get_block.LiftUp();
        is_lift_up = true;
      }
      else
      {
        APP::get_block.LiftDown();
        is_lift_up = false;
      }
      last_lift_time = DWT_GetTimeline_Sec();
    }

    // 处理摩擦带的启停
    static float last_belt_time = 0;
    if(MOD::farcon.button_second_half[2] == 1 && DWT_GetTimeline_Sec() - last_belt_time > 1.0f)
    {
      static bool is_belt_on = false;
      if(!is_belt_on)
      {
        APP::get_block.motor_2006_fric.SetSpd(15.0f * MotorDJIConst::redu_M2006 * 60.0f / (2 * PI));
        APP::get_block.dm_fric_spd = 15.0f;
        is_belt_on = true;
      }
      else
      {
        APP::get_block.motor_2006_fric.SetSpd(0);
        APP::get_block.dm_fric_spd = 0.0f;
        is_belt_on = false;
      }
      last_belt_time = DWT_GetTimeline_Sec();
    }

    // 处理放块
    static float last_put_time = 0;
    if(MOD::farcon.button_second_half[4] == 1 && DWT_GetTimeline_Sec() - last_put_time > 1.0f)
    {
      static bool is_put = false;
      if(!is_put)
      {
        APP::get_block.motor_2006_feed.SetPos(-1780000.0f);
        is_put = true;
      }
      else
      {
        APP::get_block.motor_2006_feed.SetPos(-10000.0f);
        is_put = false;
      }
      last_put_time = DWT_GetTimeline_Sec();
    }    

    /**************   均为左半按键  *************/
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

    if(MOD::farcon.button_first_half[6] == 1)
    {
      APP::vac_claw.OpenSnap();
    }

    if(MOD::farcon.button_first_half[7] == 1)
    {
      APP::vac_claw.CloseSnap();
    }

    // 处理梅林的上下
    if(MOD::farcon.button_middle[0][0] == 1) // KFS0
    {
      Area2::mei_lin_array.Climb_Up200();
    }
    if(MOD::farcon.button_middle[0][1] == 1) // KFS1
    {
      Area2::mei_lin_array.Decend_200();
    }
    if(MOD::farcon.button_middle[0][2] == 1) // KFS2
    {
      Area2::mei_lin_array.Climb_Up400();
    }
    if(MOD::farcon.button_middle[1][0] == 1) // KFS3
    {
      Area2::mei_lin_array.Decend_400();
    }
    if(MOD::farcon.button_middle[1][1])
    {
      Area2::mei_lin_array.TurnLeft();
    }
    if(MOD::farcon.button_middle[1][2])
    {
      Area2::mei_lin_array.TurnRight();
    }

    // 进行梅林上的位置闭环
    if(MOD::farcon.button_first_half[4] == 1)
    {
      Area2::mei_lin_array.ChaseCurrentGrid();
    }
}
