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
#include "start_to_table.hpp"
#include "table_to_dock.hpp"

static void Response_Button(StateCore *core);

// 自动状态机
StateGraph AutoGraph("Auto");
static void RequestTakeover();
static void Waiting_for_Start(StateCore *core);
static void Walk_Fetch(StateCore *core);
static void Walk_To_Dock(StateCore *core);

void Config_AutoStateGraph()
{
  StateBlock& s_wait = AutoGraph.AddState("wait");
  StateBlock& s_area1_1 = AutoGraph.AddState("area1_1");
  StateBlock& s_area1_2 = AutoGraph.AddState("area1_2");
  StateBlock& s_manul = AutoGraph.AddState("manual");

  s_wait.StateAction = Waiting_for_Start;
  s_area1_1.StateAction = Walk_Fetch;
  s_area1_2.StateAction = Walk_To_Dock;
  s_manul.StateAction = Response_Button;

  s_wait.LinkTo(&s_wait.Complete, s_area1_1);
  s_area1_1.LinkTo(&s_area1_1.Complete, s_area1_2);
  s_area1_2.LinkTo(&s_area1_2.Complete, s_wait);

  s_area1_2.LinkTo(&s_area1_2.Complete, s_manul);
  s_manul.LinkTo(&s_manul.Complete, s_wait);

  // 注册全局动作：在每个状态执行周期前检查遥控器接管按钮
  AutoGraph.SetGlobalAct([](StateCore *core) {
      // 检测遥控器上的"接管"按钮，例如 button_first_half[0]
      if (MOD::farcon.button_second_half[7] == 1) {
          RequestTakeover();
      }
  });

  // 向状态机核心注册
  APP::state_core.RegistGraph(AutoGraph);
  APP::state_core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
}

// void SwitchToManualGraph()
// {
//   Seq::WaitUntil([](){
//     return MOD::farcon.button_second_half[1] == 1;
//   });
//   APP::state_core.SwitchGraph("Manual","resp");
// }

// ============================================================
// 遥控器接管机制 
// ============================================================

/// @brief 遥控器接管请求标志
static bool s_takeover_requested = false;

/// @brief 请求遥控器接管（由轮询或全局动作触发）
static void RequestTakeover()
{
    if (!s_takeover_requested)
    {
        s_takeover_requested = true;
        APP::monit.LogInfo("[Takeover] 接管已请求，当前动作完成后切换");
    }
}

/// @brief 检查是否有待处理的接管请求
static bool IsTakeoverRequested()
{
    return s_takeover_requested;
}

/// @brief 执行接管 — 直接切换到 Manual 图的指定状态
/// @return true=已执行切换, false=无待处理请求
static bool ExecuteTakeover()
{
    if (!s_takeover_requested)
        return false;

    s_takeover_requested = false;

    for (uint8_t i = 0; i < APP::state_core.graph_nums; i++)
    {
        if (strcmp(APP::state_core.graphs[i]->name, "Manual") == 0)
        {
            // 切换到 Manual 图的 "resp" 状态，直接进入遥控响应模式
            uint8_t targ_state = 1; // 0="wait", 1="resp"

            APP::state_core.at_graph_id = i;
            APP::state_core.graphs[i]->executor_at_id = targ_state;
            APP::state_core.graphs[i]->current_state = &APP::state_core.graphs[i]->states[targ_state];

            APP::monit.LogInfo("[Takeover] 已切入 Manual 图 → %s 状态",
                APP::state_core.graphs[i]->states[targ_state].name);
            return true;
        }
    }

    APP::monit.LogError("[Takeover] 未找到 Manual 图，接管失败！");
    return false;
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
bool PitchDown() 
{
    APP::jaw.PitchDown();
    Seq::Wait(0.5);
    return true;
}
bool PitchUp() 
{
  APP::jaw.PitchUp(); 
  Seq::Wait(0.5);
  return true;
}
bool OpenClaw() 
{
  APP::jaw.OpenClaw(); 
  Seq::Wait(1.0);
  return true;
}
bool CloseClaw() 
{
  APP::jaw.CloseClaw(); 
  return true;
}
bool MoveToTable() 
{
  bool success = MOVE::MoveToTargPos(StartToTablePath);
  return success;
}

void Walk_Fetch(StateCore *core)
{
  if (ExecuteTakeover()) return;
  bool success = ActionChain("Walk_Fetch_TurnAround")
    .Add("jaw_down", PitchDown)
    .Add("claw_open", OpenClaw)
    .Add("move_to_table", MoveToTable)
    .Add("fetch_spear", CloseClaw)
    .Add("jaw_up", PitchUp)
    .Run();

  if (ExecuteTakeover()) return;
  core->GetCurState().Complete = true; // 设置状态完成标志，触发状态转移
  // // 无限等待，因为后面可能有问题。。
  // Seq::WaitUntil([](){
  //   return false;
  // });
}

/**
 * @brief 一区第二个动作，取到矛头后，转身走到对接位置，准备对接
 * 
 * @param core 
 */

bool MoveToDock()
{
  bool success = MOVE::MoveToTargPos(TableToDockPath);
  return success;
}

bool StartDock()
{
  // 等一会
  Seq::Wait(1.0f);
  // 开始接收对接偏移
  APP::jaw.SetTrackMode(true);
  return true;
}

void Walk_To_Dock(StateCore *core)
{
  if (ExecuteTakeover()) return;
  bool success = ActionChain("Walk_To_Dock")
    .Add("move_to_dock", MoveToDock)
    .Add("start_dock",StartDock)
    .Run();

  ExecuteTakeover();
  // 无限等待，因为后面可能有问题。。
  // Seq::WaitUntil([](){
  //   return false;
  // });
  core->GetCurState().Complete = true; // 设置状态完成标志，触发状态转移
  // SwitchToManualGraph();
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
    else if (MOD::farcon.button_second_half[1] == 1)
    {
      APP::monit.LogInfo("In Cooldown.");  
    }

    // 限制每 0.05 秒最多触发一次
    // static float trigger_cnt = 0;
    // if (DWT_GetTimeline_Sec() - trigger_cnt > 0.05f)
    // {
    //   if (MOD::farcon.button_first_half[2] == 1)
    //   {
    //     APP::leg.ChangeFrontPos(3);
    //   }

    //   if (MOD::farcon.button_first_half[3] == 1)
    //   {
    //     APP::leg.ChangeFrontPos(-3);
    //   }

    //   if (MOD::farcon.button_first_half[6] == 1)
    //   {
    //     APP::leg.ChangeBackPos(3);
    //   }

    //   if (MOD::farcon.button_first_half[7] == 1)
    //   {
    //     APP::leg.ChangeBackPos(-3);
    //   }
    //   trigger_cnt = DWT_GetTimeline_Sec();
    // }
}

