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
#include "Area3_Act.hpp"
#include "Area1_Act.hpp"
#include "CommCenter.hpp"
#include "Get_Block.hpp"
#include "MainLoop.hpp"

/// @brief 所有的路径文件
#include "PathCenter/PathCenter.hpp"
#include "ttd_3.hpp"
#include "a12_1.hpp"

static void Response_Button();
static bool IR_Read(InfraredDetector& ir);
static bool NIR_Read(InfraredDetector& ir);

// 自动状态机
StateGraph SemiAutoGr("SemiAuto");
/// @brief 当前需要取的矛头编号
static uint8_t need_spear_id = 0;

/*********    额外的状态转移Flag    *********/
static bool fetch_again = false;    // 是否需要再次取矛头
static bool move_to_area2 = false;  // 是否可以移动到二区了

static bool retry_dir_area2 = false; // 重试：是否需要直接进入二区
static bool retry_comm_err = false; // 重试：是否需要重新请求工控机发送动作序

// ── 状态函数声明 ──
static void Waiting_for_Start(StateCore *core);
// 一区
static void Walk_Fetch(StateCore *core);
static void Walk_To_Dock(StateCore *core);
static void Dock_Fetch_And_Back(StateCore *core);
// 二区
static void Goto_Area2(StateCore *core);
static void Area2_HostAction(StateCore *core);
// 场间跑点
static void Walk_into_Area3(StateCore *core);
// 三区
static void Put_First_Block(StateCore *core);
static void Wait_R1_Instru(StateCore *core);
// 重试
static void Retry_Direct_ToArea2(StateCore *core);
static void Retry_Comm_Error(StateCore *core);


int Config_SemiAutoStateGraph()
{
  /************************ 状态添加 ************************/
  // 启动前
  StateBlock& s_wait = SemiAutoGr.AddState("wait");

  // 一区
  StateBlock& s_area1_f1   = SemiAutoGr.AddState("area1_f1");      // 取第一个矛头
  StateBlock& s_area1_td   = SemiAutoGr.AddState("area1_td");      // 走到对接点
  StateBlock& s_dock_fetch = SemiAutoGr.AddState("dock_fetch");   // 对接取矛并返回

  // 二区
  StateBlock& s_goto_area2     = SemiAutoGr.AddState("goto_area2");     // 走进二区（梅林跟前）
  StateBlock& s_area2_hostact  = SemiAutoGr.AddState("area2_hostact");  // 执行二区工控机动作序列

  // 场间跑点
  StateBlock& s_walk_a2_to_a3 = SemiAutoGr.AddState("walk_a2_a3");     // 二区→三区

  // 三区
  StateBlock& s_put_first  = SemiAutoGr.AddState("put_first");         // 放第一个块
  StateBlock& s_wait_r1    = SemiAutoGr.AddState("wait_r1");           // 等待R1指令

  // 重试
  StateBlock& s_retry_dirctA2  = SemiAutoGr.AddState("retry_dircA2");   // 重试：直接进二区
  StateBlock& s_retry_comm_err = SemiAutoGr.AddState("retry_comm_err"); // 重试：光通信异常

  /************************ 状态函数绑定 ************************/
  s_wait.StateAction = Waiting_for_Start;

  s_area1_f1.StateAction   = Walk_Fetch;
  s_area1_td.StateAction   = Walk_To_Dock;
  s_dock_fetch.StateAction = Dock_Fetch_And_Back;

  s_goto_area2.StateAction    = Goto_Area2;
  s_area2_hostact.StateAction = Area2_HostAction;

  s_walk_a2_to_a3.StateAction = Walk_into_Area3;

  s_put_first.StateAction  = Put_First_Block;
  s_wait_r1.StateAction    = Wait_R1_Instru;

  s_retry_dirctA2.StateAction  = Retry_Direct_ToArea2;
  s_retry_comm_err.StateAction = Retry_Comm_Error;

  /************************ 状态链接 ************************/
  // ── 正常流程 ──
  s_wait.LinkTo(&s_wait.Complete, s_area1_f1);       // 等待完成 → 取矛头
  s_area1_f1.LinkTo(&s_area1_f1.Complete, s_area1_td); // 取完 → 去对接
  s_area1_td.LinkTo(&s_area1_td.Complete, s_dock_fetch); // 对接完成 → 对接取矛

  // dock_fetch 分支：继续取矛 或 进二区
  s_dock_fetch.LinkTo(&fetch_again, s_dock_fetch);
  s_dock_fetch.LinkTo(&move_to_area2, s_goto_area2);

  // 二区 → 三区
  s_goto_area2.LinkTo(&s_goto_area2.Complete, s_area2_hostact);
  s_area2_hostact.LinkTo(&s_area2_hostact.Complete, s_walk_a2_to_a3);

  // 三区
  s_walk_a2_to_a3.LinkTo(&s_walk_a2_to_a3.Complete, s_put_first);
  s_put_first.LinkTo(&s_put_first.Complete, s_wait_r1);

  // ── 重试流程 ──
  s_wait.LinkTo(&retry_dir_area2, s_retry_dirctA2);
  s_wait.LinkTo(&retry_comm_err, s_retry_comm_err);
  s_retry_dirctA2.LinkTo(&s_retry_dirctA2.Complete, s_goto_area2);
  s_retry_comm_err.LinkTo(&s_retry_comm_err.Complete, s_goto_area2);

  /**----------     向状态机核心注册    ----------**/
  return APP::state_core.RegistGraph(SemiAutoGr);
}


/*******************************************************************************
 *  @brief 刚开机时，等待出发信号的状态
 ******************************************************************************/
void Waiting_for_Start(StateCore *core)
{
  Area2::mei_lin_array.is_blue = false; // 设置为红场

  APP::jaw.CloseClaw();
  APP::monit.LogInfo("Waiting for start...");
  if (APP::main_loop.retry_id == 0)
  {
    APP::main_loop.test_led.Breath(Color::White, 2.0f);
  }
  else
  {
    APP::main_loop.test_led.Breath(Color::Yellow, 1.0f);
  }

  APP::leg.F_AlmostOrigin();
  APP::leg.B_AlmostOrigin();

  Seq::Wait(0.1f);

  // 等待遥控器确认
  while (MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  APP::chas.Enable();
  APP::main_loop.test_led.Lit(Color::White);

  // 取矛头初始化
  Seq::WaitUntil([]() { return APP::jaw.IsOrigined(); });
  APP::monit.LogInfo("Jaw Init OK.");

  // 等待工控机连接
  Seq::WaitUntil([]() { return MOD::host_pc.IsOnline(); });
  APP::monit.LogInfo("HostPC Online.");

  Seq::Wait(0.75f);
  APP::monit.LogInfo("Ready to Go...");

  // 等待出发指令
  while (MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  /***    重试    ***/
  if (APP::main_loop.retry_id == 1)
  {
    APP::monit.LogRespond("Retry: Direct to Area2.");
    retry_dir_area2 = true;
    return;
  }
  else if (APP::main_loop.retry_id == 2)
  {
    APP::monit.LogRespond("Retry: Comm Error.");
    retry_comm_err = true;
    return;
  }

  core->GetCurState().Complete = true;
}


/*******************************************************************************
 *  @brief 一区：从原点出发，取第一个矛头
 ******************************************************************************/
void Walk_Fetch(StateCore *core)
{
  APP::leg.SetFrontPos(-5);
  APP::leg.SetBackPos(-5);
  APP::main_loop.test_led.GradientFlow(Color::Up_Purpor * 0.65f, Color::Up_Yellow * 0.65f, 1.0f);

  // 取矛头，第一个取 2 号，后续由光通信决定
  need_spear_id = 2;
  FetchSpear_FromOrigin(need_spear_id);

  core->GetCurState().Complete = true;
}


/*******************************************************************************
 *  @brief 一区：走到对接位置，进入随动模式
 ******************************************************************************/
void Walk_To_Dock(StateCore *core)
{
  static int track_mode_cnt = 0;
  GotoDock(need_spear_id);

  APP::jaw.SetTrackMode(true);
  APP::main_loop.test_led.Running(Color::Up_Yellow, 0.4f, 1.2f);

  // 等待遥控器或对接完成
  while (MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);

    track_mode_cnt++;
    if (track_mode_cnt > 800) // 超过 8s 锁死工控机视觉 offset
    {
      APP::jaw.LockHostPCBias();
    }

    if (APP::comm_center.IsDocked()) break;
  }

  // 灯带：进度条
  float wait_progress = 0.0f;
  APP::main_loop.test_led.ProgBar(Color::Up_Yellow, &wait_progress);

  APP::jaw.OpenClaw();

  // 等待 6 秒
  float wait_time_s = 0.0f;
  while (wait_time_s < 6.0f)
  {
    wait_time_s += 0.05f;
    wait_progress = wait_time_s / 6.0f;
    Seq::Wait(0.05f);
  }

  APP::jaw.SetTrackMode(false);
  APP::jaw.SetPosCode(APP::jaw._hori_origin_code - JawConst::_hori_max_travel_code / 2,
                      APP::jaw._vert_origin_code - 200000);

  APP::monit.LogInfo("Fetch and Dock again.");
  need_spear_id++;
  APP::comm_center.ClearDockFlag();

  core->GetCurState().Complete = true;
}


/*******************************************************************************
 *  @brief 一区：从对接点取矛头并返回
 ******************************************************************************/
void Dock_Fetch_And_Back(StateCore *core)
{
  bool giveup_dock = false;
  fetch_again = false;

  APP::main_loop.test_led.Running(Color::Up_Yellow, 0.4f, 1.2f);
  FetchSpear_FromDock(need_spear_id);

  // 返回对接点
  GotoDock(need_spear_id);
  APP::jaw.SetTrackMode(true);
  APP::main_loop.test_led.Running(Color::Up_Yellow, 0.4f, 1.2f);

  while (true)
  {
    Response_Button();
    Seq::Wait(0.01f);

    if (APP::comm_center.IsDocked() || MOD::farcon.button_second_half[0] == 1)
    {
      APP::jaw.OpenClaw();
      break;
    }
    if (APP::comm_center.IsDockGiveUp())
    {
      giveup_dock = true;
      break;
    }
  }

  APP::jaw.SetTrackMode(false);

  // 区分：是否还要再取一个矛头
  if (need_spear_id < 2 && !giveup_dock)
  {
    float wait_progress = 0.0f;
    APP::main_loop.test_led.ProgBar(Color::Up_Yellow, &wait_progress);
    float t = 0.0f;
    while (t < 12.0f) { t += 0.05f; wait_progress = t / 12.0f; Seq::Wait(0.05f); }

    if (!giveup_dock)
      APP::jaw.SetPosCode(APP::jaw._hori_origin_code - JawConst::_hori_max_travel_code / 2,
                          APP::jaw._vert_origin_code - 200000);

    need_spear_id++;
    APP::comm_center.ClearDockFlag();
    fetch_again = true;
    APP::monit.LogInfo("Go Fetch Again.");
  }
  else
  {
    APP::comm_center.ClearDockFlag();
    Seq::Wait(0.5f);
    APP::main_loop.test_led.Running(Color::Orange, 0.4f, 1.2f);

    while (true)
    {
      Response_Button();
      Seq::Wait(0.01f);
      if (APP::comm_center.IsDocked() || MOD::farcon.button_second_half[0] == 1
          || APP::comm_center.IsDockGiveUp())
        break;
    }

    float wait_progress = 0.0f;
    APP::main_loop.test_led.ProgBar(Color::Up_Yellow, &wait_progress);
    float t = 0.0f;
    while (t < 6.0f) { t += 0.05f; wait_progress = t / 6.0f; Seq::Wait(0.05f); }

    APP::comm_center.ClearDockFlag();
    fetch_again = false;
    move_to_area2 = true;
    APP::monit.LogInfo("Go to Climb.");
  }
}


/*******************************************************************************
 *  @brief 二区：走向梅林块跟前（使用 RunIntoArea2，中途自动抬腿）
 ******************************************************************************/
void Goto_Area2(StateCore *core)
{
  APP::jaw.CloseClaw();
  APP::main_loop.test_led.Blink(Color::Pink, 2, 0.5f);

  // 确认工控机已经发送了动作序列
  Seq::WaitUntil([]() { return Area2::mei_lin_array.host_a2act_recved; });

  APP::main_loop.test_led.Lit(Color::White);
  APP::monit.LogInfo("entercode:%x", Area2::mei_lin_array.enter_code);

  switch (Area2::mei_lin_array.enter_code)
  {
    case Area2Code::ENTER_0:
      Area2::RunIntoArea2(PATH::A12_3(), Vec3(0.15f, 0.15f, 0.0f), true);
      break;
    case Area2Code::ENTER_1:
      Area2::RunIntoArea2(PATH::A12_2(), Vec3(0.15f, 0.15f, 0.0f), false);
      break;
    case Area2Code::ENTER_2:
      Area2::RunIntoArea2(PATH::A12_1(), Vec3(0.15f, 0.15f, 0.0f), true);
      break;
    default:
      APP::monit.LogError("Invalid A2EnterCode!");
      Seq::Wait(3600.0f);
      break;
  }

  APP::monit.LogInfo("I'm Ahead of MeiLin.");
  APP::monit.LogInfo("Pos Corrected : %.2f, %.2f, %.2f",
                     System.position.x, System.position.y, System.position.z);

  core->GetCurState().Complete = true;
}


/*******************************************************************************
 *  @brief 二区：执行工控机发来的梅林动作序列
 ******************************************************************************/
void Area2_HostAction(StateCore *core)
{
  APP::monit.LogInfo("Now, Start Area2 Actions.");
  Area2::mei_lin_array.ExecuteSequence();
  APP::monit.LogInfo("Area2 Done. Now go to Area3--ARENA.");

  core->GetCurState().Complete = true;
}


/*******************************************************************************
 *  @brief 场间跑点：从二区走到三区（九宫格擂台）
 ******************************************************************************/
void Walk_into_Area3(StateCore *core)
{
  // 放低腿准备跑路
  APP::leg.SetFrontPos(-40);
  APP::leg.SetBackPos(-40);

  // 从二区出口 → 斜坡入口 → 擂台上
  MOVE::MoveToTargGes(Vec3(8.65f, 0.77f, 0.0f));

  while (MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  // 上斜坡，跑到擂台上的中间位置
  MOVE::MoveToTargGes(Vec3(11.45f, 0.77f, 0.0f));

  while (MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  // 跑到九宫格第二层中间位置
  MOVE::MoveToTargPos(PATH::A23_2(), Vec3(0.01f, 0.01f, 0.01f));
  MOVE::MoveToTargGes(Vec3(10.75f, 5.20f, 1.571f));

  // 西克激光对准
  Seq::Wait(0.25f);
  int sick_cnt = 0;
  const float sick_kd = 0.25f;
  const float sick_d_alpha = 0.7f;
  float last_err_dist = APP::comm_center.sick_data_m[1] - APP::comm_center.sick_data_m[0];
  float d_err_dist_filt = 0.0f;
  uint32_t sick_log_tick = 0;

  while (sick_cnt < 10)
  {
    float err_dist = APP::comm_center.sick_data_m[1] - APP::comm_center.sick_data_m[0];
    if (fabs(err_dist) < 0.004f) sick_cnt++;
    else sick_cnt = 0;

    float d_err_dist = (err_dist - last_err_dist) / 0.005f;
    d_err_dist_filt = d_err_dist_filt * sick_d_alpha + d_err_dist * (1.0f - sick_d_alpha);
    float err_spd = err_dist * 1.0f - sick_kd * d_err_dist_filt;
    last_err_dist = err_dist;

    Vec2 sick_fix = Vec2(err_spd, 0.0f).Rotate(-System.position.z);
    float sick_speed = sick_fix.Length();
    if (sick_speed > 0.3f) sick_fix = sick_fix * (0.3f / sick_speed);

    float yaw_err = 1.5707f - System.position.z;
    Vec3 targ_body_vec = sick_fix.ToVec3();
    targ_body_vec.z = yaw_err * 0.8f;

    if (++sick_log_tick >= 20)
    {
      sick_log_tick = 0;
      APP::monit.LogInfo("sf:e%.3f v%.3f s0%.3f s1%.3f",
                         err_dist, err_spd,
                         APP::comm_center.sick_data_m[0],
                         APP::comm_center.sick_data_m[1]);
    }
    APP::chas.Move(targ_body_vec, 0.01f);
    Seq::Wait(0.005f);
  }

  APP::monit.LogInfo("Finished sick fix. s0:%.3f s1:%.3f",
                     APP::comm_center.sick_data_m[0], APP::comm_center.sick_data_m[1]);
  APP::chas.Move(Vec3(0, 0, 0), 0.25f);

  // 西克校准后修正坐标
  int cnt = 0;
  float _avg_x = 0.0f;
  while (cnt < 10) { _avg_x += System.position.x; cnt++; Seq::Wait(0.005f); }
  _avg_x /= 10.0f;
  float arena_x_bias = 10.75f - _avg_x;
  APP::monit.LogInfo("[arena_bias]:%.3f", arena_x_bias);
  System.SetPositionBias(Vec3(arena_x_bias, 0.0f, 0.0f));

  Seq::Wait(0.5f);

  APP::monit.LogInfo("Arrived at Arena, ready to put first block.");
  core->GetCurState().Complete = true;
}


/*******************************************************************************
 *  @brief 三区：放第一个块到九宫格 L2
 ******************************************************************************/
void Put_First_Block(StateCore *core)
{
  // 抬起腿
  APP::leg.SetFrontPos(-388);
  APP::leg.SetBackPos(-380);

  Seq::WaitUntil([]() {
    return fabs(APP::leg.real_front_pos_mm - APP::leg.front_pos) < 6
        && fabs(APP::leg.real_back_pos_mm - APP::leg.back_pos) < 6;
  });

  // 检查当前位置是否有块，有则平移避让
  if (APP::get_block.IsBlockPresent())
  {
    APP::monit.LogInfo("Middle block exists, move to right");
    MOVE::MoveToTargGes(Vec3(11.29f, 5.20f, 1.571f));

    if (APP::get_block.IsBlockPresent())
    {
      APP::monit.LogInfo("Right block also exists, move to left");
      MOVE::MoveToTargGes(Vec3(10.21f, 5.20f, 1.571f));

      if (APP::get_block.IsBlockPresent())
        APP::monit.LogInfo("All three have blocks, need combine!");
    }
  }

  APP::monit.LogSpec("Put At:%.2f, %.2f", System.position.x, System.position.y);
  APP::get_block.Push();

  Seq::Wait(1.0f);

  // 确认
  while (MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  core->GetCurState().Complete = true;
}


/*******************************************************************************
 *  @brief 三区：等待 R1 光通信指令（非阻塞循环，200Hz 调用）
 ******************************************************************************/
void Wait_R1_Instru(StateCore *core)
{
  auto code = APP::comm_center.Get_R1_InstruCode();
  if (code == Area3ActCode::Idle)
    return;  // 无指令，保持等待

  // 收到指令，做个等待给R1让路
  float wait_progress = 0.0f;
  APP::main_loop.test_led.ProgBar(Color::Up_Yellow, &wait_progress);
  float t = 0.0f;
  while (t < 6.0f) { t += 0.05f; wait_progress = t / 6.0f; Seq::Wait(0.05f); }

  switch (code)
  {
    case Area3ActCode::Put_L2_N4_Block:
    case Area3ActCode::Put_L2_N5_Block:
    case Area3ActCode::Put_L2_N6_Block:
    {
      // TODO: 根据R1指令完成后续放块
      break;
    }
    case Area3ActCode::Combine:
    {
      // TODO: 合体逻辑
      break;
    }
    case Area3ActCode::Fetch_GroundBlock_0:
    case Area3ActCode::Fetch_GroundBlock_1:
    {
      // TODO: 从等待点出发取地面块
      break;
    }
    default: break;
  }

  APP::comm_center.Reset_R1_InstruCode();
}


/*******************************************************************************
 *  @brief 重试：直接前往二区
 ******************************************************************************/
void Retry_Direct_ToArea2(StateCore *core)
{
  MOVE::MoveToTargGes(Vec3(1.3f, 1.0f, -1.5707f));
  core->GetCurState().Complete = true;
}


/*******************************************************************************
 *  @brief 重试：光通信异常，重新请求
 ******************************************************************************/
void Retry_Comm_Error(StateCore *core)
{
  MOVE::MoveToTargGes(Vec3(0.8f, 1.3f, 0.0f));

  Seq::WaitUntil([]() { return APP::comm_center.IsKFSSeqRecved(); });

  float wait_progress = 0.0f;
  APP::main_loop.test_led.ProgBar(Color::Up_Yellow, &wait_progress);
  float t = 0.0f;
  while (t < 6.0f) { t += 0.05f; wait_progress = t / 6.0f; Seq::Wait(0.05f); }

  core->GetCurState().Complete = true;
}


/*═══════════════════════════════════════════════════════════════════════════════
 *  @brief 遥控器响应（内部用，无加速度限制）
 *═════════════════════════════════════════════════════════════════════════════*/
static void Response_Button()
{
    static float max_velo = 1.5f;

    constexpr float deadzone = 15.0f;
    auto map_axis = [deadzone](int v) -> float {
      float a = (v >= 0) ? float(v) : -float(v);
      if (a <= deadzone) return 0.0f;
      return (v > 0 ? 1.0f : -1.0f) * ((a - deadzone) / (100.0f - deadzone));
    };

    Vec3 vel;
    vel.x = -map_axis(MOD::farcon.jys_value[3]) * max_velo;
    vel.y = -map_axis(MOD::farcon.jys_value[2]) * max_velo;
    vel.z = -map_axis(MOD::farcon.jys_value[0]) * max_velo;

    APP::chas.Move(vel, 0.2f);
}

/*═══════════════════════════════════════════════════════════════════════════════
 *  @brief 红外读取辅助
 *═════════════════════════════════════════════════════════════════════════════*/
static bool IR_Read(InfraredDetector& ir)
{
  static int read_cnt = 0;
  if (ir.Read()) { read_cnt++; if (read_cnt >= 5) { read_cnt = 0; return true; } }
  else { read_cnt = 0; }
  return false;
}

static bool NIR_Read(InfraredDetector& ir)
{
  static int read_cnt = 0;
  if (!ir.Read()) { read_cnt++; if (read_cnt >= 5) { read_cnt = 0; return true; } }
  else { read_cnt = 0; }
  return false;
}