#include "ManualGraph.hpp"
#include "Monitor.hpp"
#include "Jaw.hpp"
#include "Farcon.hpp"
#include "Chassis.hpp"
#include "Lift_Leg.hpp"
#include "Area3_Act.hpp"
#include "Get_Block.hpp"

// 手动遥控的状态机
StateGraph ManualGraph("Manual");

static bool OK_Flag = true;

static void Waiting_for_Start(StateCore *core);
static void Response_Button(StateCore *core);

void Config_StateGraph()
{
  StateBlock& s_wait = ManualGraph.AddState("wait");
  StateBlock& s_response = ManualGraph.AddState("resp");

  s_wait.StateAction = Waiting_for_Start;
  s_response.StateAction = Response_Button;

  s_wait.LinkTo(&s_wait.Complete, s_response);
  s_response.LinkTo(&s_response.Complete, s_wait);

  // 向状态机核心注册
  APP::state_core.RegistGraph(ManualGraph);
  APP::state_core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
}

void Waiting_for_Start(StateCore *core)
{
  APP::monit.LogInfo("I'm here...");

  // Seq::WaitUntil([](){
  //   return APP::jaw.IsOrigined();
  // });

  Seq::Wait(2.0f);

  // Seq::WaitUntil(OK_Flag);

  APP::monit.LogInfo("Ready to Response...");

  core->GetCurState().Complete = true; // 设置状态完成标志，触发状态转移
}

const static float max_farcon_alpha = 1.8f; // 最大加速度 m/s^2
const static float max_farcon_beta = 1.8f; // 最大角加速度 rad/s^2
void Response_Button(StateCore *core)
{
    if (MOD::farcon.button_first_half[0] == 1)
    {
        APP::jaw.PitchDown();
    }
    
    if (MOD::farcon.button_first_half[1] == 1)
    {
        APP::jaw.PitchUp();
    }

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
    auto map_axis = [](int v) -> float {
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

    // 抬升逻辑
    static float last_trigger_time = 0;
    if(MOD::farcon.button_second_half[0] == 1 && DWT_GetTimeline_Sec() - last_trigger_time > 1.0f)
    {
      APP::leg.SetFrontPosSlow(-300, 1.5f);
      APP::leg.SetBackPosSlow(-300, 1.5f);
      last_trigger_time = DWT_GetTimeline_Sec();
    }
    if(MOD::farcon.button_second_half[4] == 1 && DWT_GetTimeline_Sec() - last_trigger_time > 1.0f)
    {
      APP::leg.SetFrontPosSlow(50, 1.5f);
      APP::leg.SetBackPosSlow(50, 1.5f);
      last_trigger_time = DWT_GetTimeline_Sec();
    }

    static float last_store = 0;
    if(MOD::farcon.button_second_half[1] == 1 && DWT_GetTimeline_Sec() - last_store > 15.0f)
    {
      Area3::FetchBlockOnTheGround();
      last_store = DWT_GetTimeline_Sec();
    }

    static float last_put = 0;
    if(MOD::farcon.button_second_half[5] == 1 && DWT_GetTimeline_Sec() - last_put > 8.0f)
    {
      APP::get_block.Put();
      last_put = DWT_GetTimeline_Sec();
    }
    // // 检测 9 号键，如果按下了，就开关爪子（每1s最多触发一次）
    // static float last_trigger_time = 0;
    // if (MOD::farcon.button_second_half[0] == 1 && DWT_GetTimeline_Sec() - last_trigger_time > 1.0f)
    // {
    //     static bool claw_open = false;
    //     if (claw_open)
    //     {
    //         APP::jaw.CloseClaw();
    //     }
    //     else
    //     {
    //         APP::jaw.OpenClaw();
    //     }
    //     APP::monit.LogInfo("Claw %s", claw_open ? "Closed" : "Opened");
    //     claw_open = !claw_open;
    //     last_trigger_time = DWT_GetTimeline_Sec();
    // }
    // // else if (MOD::farcon.button_second_half[1] == 1)
    // // {
    // //   APP::monit.LogInfo("In Cooldown.");  
    // // }

    // // 限制每 0.05 秒最多触发一次
    // static float trigger_cnt = 0;
    // if (DWT_GetTimeline_Sec() - trigger_cnt > 0.05f)
    // {
    //   // 10号，前放 200  
    //   if (MOD::farcon.button_second_half[1] == 1)
    //   {
    //     APP::leg.F_Down_200();
    //   }
    //   // 14号，前放 400
    //   if (MOD::farcon.button_second_half[5] == 1)
    //   {
    //     APP::leg.F_Down_400();
    //   }

    //   // 11号，后放 200
    //   if (MOD::farcon.button_second_half[2] == 1)
    //   {
    //     APP::leg.B_Down_200();
    //   }
    //   // 15号，后放 400
    //   if (MOD::farcon.button_second_half[6] == 1)
    //   {
    //     APP::leg.B_Down_400();
    //   }

    //   // 12号，前归 0 位
    //   if (MOD::farcon.button_second_half[3] == 1)
    //   {
    //     APP::leg.F_Origin();
    //   }
    //   // 16号，后归 0 位
    //   if (MOD::farcon.button_second_half[7] == 1)
    //   {
    //     APP::leg.B_Origin();
    //   } 

    //   trigger_cnt = DWT_GetTimeline_Sec();
    // }
}
