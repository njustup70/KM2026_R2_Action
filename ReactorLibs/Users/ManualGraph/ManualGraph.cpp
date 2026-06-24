#include "ManualGraph.hpp"
#include "Monitor.hpp"
#include "Jaw.hpp"
#include "Farcon.hpp"
#include "Chassis.hpp"
#include "Lift_Leg.hpp"

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

  Seq::WaitUntil([](){
    return APP::jaw.IsOrigined();
  });

  Seq::Wait(2.0f);

  // Seq::WaitUntil(OK_Flag);

  APP::monit.LogInfo("Ready to Response...");

  core->GetCurState().Complete = true; // 设置状态完成标志，触发状态转移
}

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

    if(APP::chas.control_mode == APP::chas.FARCON)
    {
        float max_velo = 1.5f;
        Vec3 vel;
        // 计算速度
        vel.x = (-MOD::farcon.jys_value[3] / 100.0f) * max_velo; // 最大速度0.5m/s
        vel.y = (-MOD::farcon.jys_value[2] / 100.0f) * max_velo;  // 最大速度0.5m/s
        vel.z = (-MOD::farcon.jys_value[0] / 100.0f) * max_velo;  // 最大角速度1.0rad/s

        if (vel.Length() > 0.03f)
        {
        // 向底盘发送移动指令
            APP::chas.Move(vel);
        }
    }

    // 切换控制模式
    if(MOD::farcon.toggle[0]) // 最左上拨杆拨为1
    {
        APP::chas.control_mode = MOD::farcon.toggle[1] ? APP::chas.HOST : APP::chas.FARCON; // 左2拨杆也拨为1就是上位机控制，否则遥控器
    }
    else  
    {
        APP::chas.control_mode = MOD::farcon.toggle[1] ? APP::chas.API : APP::chas.OPEN;
    } 

    // 是否锁yaw
    if(MOD::farcon.toggle[3]) // 最右上拨杆拨为1
    {
        APP::chas.LockYaw(true);
    }
    else
    {
        APP::chas.LockYaw(false);
    }

    // 检测 9 号键，如果按下了，就开关爪子（每1s最多触发一次）
    static float last_trigger_time = 0;
    if (MOD::farcon.button_second_half[0] == 1 && DWT_GetTimeline_Sec() - last_trigger_time > 1.0f)
    {
        static bool claw_open = false;
        if (claw_open)
        {
            APP::jaw.CloseClaw();
        }
        else
        {
            APP::jaw.OpenClaw();
        }
        APP::monit.LogInfo("Claw %s", claw_open ? "Closed" : "Opened");
        claw_open = !claw_open;
        last_trigger_time = DWT_GetTimeline_Sec();
    }
    // else if (MOD::farcon.button_second_half[1] == 1)
    // {
    //   APP::monit.LogInfo("In Cooldown.");  
    // }

    // 限制每 0.05 秒最多触发一次
    static float trigger_cnt = 0;
    if (DWT_GetTimeline_Sec() - trigger_cnt > 0.05f)
    {
      // 10号，前放 200  
      if (MOD::farcon.button_second_half[1] == 1)
      {
        APP::leg.F_Down_200();
      }
      // 14号，前放 400
      if (MOD::farcon.button_second_half[5] == 1)
      {
        APP::leg.F_Down_400();
      }

      // 11号，后放 200
      if (MOD::farcon.button_second_half[2] == 1)
      {
        APP::leg.B_Down_200();
      }
      // 15号，后放 400
      if (MOD::farcon.button_second_half[6] == 1)
      {
        APP::leg.B_Down_400();
      }

      // 12号，前归 0 位
      if (MOD::farcon.button_second_half[3] == 1)
      {
        APP::leg.F_Origin();
      }
      // 16号，后归 0 位
      if (MOD::farcon.button_second_half[7] == 1)
      {
        APP::leg.B_Origin();
      } 

      trigger_cnt = DWT_GetTimeline_Sec();
    }
}
