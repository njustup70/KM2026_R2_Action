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
static void Area3_SickFix();

static bool IR_Read(InfraredDetector& ir);
static bool NIR_Read(InfraredDetector& ir);

// 自动状态机
StateGraph SemiAutoGr("SemiAuto");
/// @brief 当前需要取的矛头编号
static uint8_t need_spear_id = 0;

/*********    额外的状态转移Flag    *********/
static bool fetch_again = false;    // 是否需要再次取矛头
static bool move_to_area2 = false;  // 是否可以移动到二区了

static bool soft_retry_chasdisable = false; // 软重试：底盘失能状态
static bool retry_fetch_spear = false;      // 软重试：重新去取对应矛头

static bool retry_dir_area2 = false; // 重试：是否需要直接进入二区
static bool retry_comm_err = false; // 重试：是否需要重新请求工控机发送动作序
static bool retry_area3 = false;    // 重试：从三区重试点开始等待放块指令

// ── 状态函数声明 ──
static void Waiting_for_Start(StateCore *core);
// 一区
static void Walk_Fetch(StateCore *core);
static void Walk_To_Dock(StateCore *core);
// static void Dock_Fetch_And_Back(StateCore *core);

// 二区
static void Goto_Area2(StateCore *core);
static void Area2_HostAction(StateCore *core);
// 场间跑点
static void Walk_into_Area3(StateCore *core);
// 三区
static void Put_First_Block(StateCore *core);
static void Wait_Human_Instru(StateCore *core);
// 重试
static void Retry_Direct_ToArea2(StateCore *core);
static void Retry_Comm_Error(StateCore *core);
static void Retry_ChasDisabled(StateCore *core);


// 从R1等待位置出发的状态函数
static void Robot_Combine();

static Vec2 Arena_R2_KFS_FetchPos[2] = {
  Vec2(11.1, 3.555), Vec2(11.1, 2.835)
}; 
// 放中间九宫格块的位置
static Vec3 Arena_R2_L2_Mid_Ges(10.75f, 5.20f, 1.571f);


int Config_SemiAutoStateGraph()
{
  /************************ 状态添加 ************************/
  // 启动前
  StateBlock& s_wait = SemiAutoGr.AddState("wait");

  // 一区
  StateBlock& s_area1_f1   = SemiAutoGr.AddState("area1_f1");      // 取第一个矛头
  StateBlock& s_area1_td   = SemiAutoGr.AddState("area1_td");      // 走到对接点
  // StateBlock& s_dock_fetch = SemiAutoGr.AddState("dock_fetch");   // 对接取矛并返回

  // 二区
  StateBlock& s_goto_area2     = SemiAutoGr.AddState("goto_area2");     // 走进二区（梅林跟前）
  StateBlock& s_area2_hostact  = SemiAutoGr.AddState("area2_hostact");  // 执行二区工控机动作序列

  // 场间跑点
  StateBlock& s_walk_a2_to_a3 = SemiAutoGr.AddState("walk_a2_a3");     // 二区→三区

  // 三区
  StateBlock& s_put_first  = SemiAutoGr.AddState("put_first");         // 放第一个块
  StateBlock& s_wait_r1    = SemiAutoGr.AddState("wait_r1");           // 等待R1指令

  // 重试 （含软重试）
  StateBlock& s_retry_dirctA2  = SemiAutoGr.AddState("retry_dircA2");   // 重试：直接进二区
  StateBlock& s_retry_comm_err = SemiAutoGr.AddState("retry_comm_err"); // 重试：光通信异常

  // 用于拖动车辆的底盘失能状态
  StateBlock& s_chas_disabled = SemiAutoGr.AddState("chas_disa");

  /************************ 状态函数绑定 ************************/
  s_wait.StateAction = Waiting_for_Start;

  s_area1_f1.StateAction   = Walk_Fetch;
  s_area1_td.StateAction   = Walk_To_Dock;
  // s_dock_fetch.StateAction = Dock_Fetch_And_Back;

  s_goto_area2.StateAction    = Goto_Area2;
  s_area2_hostact.StateAction = Area2_HostAction;

  s_walk_a2_to_a3.StateAction = Walk_into_Area3;

  s_put_first.StateAction  = Put_First_Block;
  s_wait_r1.StateAction    = Wait_Human_Instru;

  s_retry_dirctA2.StateAction  = Retry_Direct_ToArea2;
  s_retry_comm_err.StateAction = Retry_Comm_Error;
  s_chas_disabled.StateAction = Retry_ChasDisabled;

  /************************ 状态链接 ************************/
  // ── 正常流程 ──
  s_wait.LinkTo(&s_wait.Complete, s_area1_f1);       // 等待完成 → 取矛头
  s_area1_f1.LinkTo(&s_area1_f1.Complete, s_area1_td); // 取完 → 去对接
  s_area1_td.LinkTo(&s_area1_td.Complete, s_goto_area2); // 对接完成 → 对接取矛

  // 对接时，可通过遥控器软“重试”，进入底盘失能状态。便于拖车。
  s_area1_td.LinkTo(&soft_retry_chasdisable, s_chas_disabled);
  s_chas_disabled.LinkTo(&retry_fetch_spear, s_area1_f1);
  s_chas_disabled.LinkTo(&retry_dir_area2, s_retry_dirctA2);



  // dock_fetch 分支：继续取矛 或 进二区
  // s_dock_fetch.LinkTo(&fetch_again, s_dock_fetch);
  // s_dock_fetch.LinkTo(&move_to_area2, s_goto_area2);

  // 二区 → 三区
  s_goto_area2.LinkTo(&s_goto_area2.Complete, s_area2_hostact);
  s_area2_hostact.LinkTo(&s_area2_hostact.Complete, s_walk_a2_to_a3);

  // 三区
  s_walk_a2_to_a3.LinkTo(&s_walk_a2_to_a3.Complete, s_put_first);
  s_put_first.LinkTo(&s_put_first.Complete, s_wait_r1);

  // ── 重试流程 ──
  s_wait.LinkTo(&retry_dir_area2, s_retry_dirctA2);
  s_wait.LinkTo(&retry_comm_err, s_retry_comm_err);
  s_wait.LinkTo(&retry_area3, s_wait_r1);
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
  else if (APP::main_loop.retry_id == 3)
  {
    APP::monit.LogRespond("Retry: Area3 retry point.");
    retry_area3 = true;
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

  // 正常流程取 2 号；软重试流程使用操作手在 Retry_ChasDisabled 中确认的编号。
  if (!retry_fetch_spear)
  {
    need_spear_id = 2;
  }
  retry_fetch_spear = false;
  FetchSpear_FromOrigin(need_spear_id);

  core->GetCurState().Complete = true;
}


/*******************************************************************************
 *  @brief 一区：走到对接位置，进入随动模式
 ******************************************************************************/
void Walk_To_Dock(StateCore *core)
{
  GotoDock(need_spear_id);

  // 进入对接模式，允许工控机发送偏置
  APP::jaw.SetTrackMode(true);
  APP::jaw.UnlockHostPCBias();
  APP::main_loop.test_led.Running(Color::Up_Yellow, 0.4f, 1.2f);

  // 等待遥控器或对接完成
  while (MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);

    if (APP::comm_center.IsDocked()) break;

    // 一个遥控器逻辑，按下某个按键后，进入“软重试”逻辑。目的是这样就可以不用R1通信，也能进入二区了。
    if (MOD::farcon.button_first_half[4] == 1)
    {
      // 软重试灯
      APP::main_loop.test_led.GradientFlow(Color::Up_Purpor * 0.85f, Color::Up_Yellow * 0.85f, 0.65f);
      soft_retry_chasdisable = true;
      return;
    }
  }

  // 灯带：进度条
  float wait_progress = 0.0f;
  APP::main_loop.test_led.ProgBar(Color::Up_Yellow, &wait_progress);

  APP::jaw.OpenClaw();

  // 等待 12 秒
  float wait_time_s = 0.0f;
  while (wait_time_s < 12.0f)
  {
    wait_time_s += 0.05f;
    wait_progress = wait_time_s / 12.0f;

    // 一个软重试逻辑，同上。放在这里，这样对接完成后也能触发
    if (MOD::farcon.button_first_half[4] == 1)
    {
      // 软重试灯：
      APP::main_loop.test_led.GradientFlow(Color::Up_Purpor * 0.85f, Color::Up_Yellow * 0.85f, 0.65f);
      soft_retry_chasdisable = true;
      return;
    }

    Seq::Wait(0.05f);
  }

  APP::jaw.SetTrackMode(false);
  APP::jaw.SetPosCode(APP::jaw._hori_origin_code - JawConst::_hori_max_travel_code / 2,
                      APP::jaw._vert_origin_code - 200000);

  APP::monit.LogInfo("Finish Dock.");
  APP::comm_center.ClearDockFlag();

  core->GetCurState().Complete = true;
}

/*******************************************************************************
 * @brief 逻辑分支：如果使用重试
 * @note 在这个状态中，车辆的底盘是完全失能的。要想脱离这个状态，只能通过遥控器按键进行选择式跳脱
 ******************************************************************************/
void Retry_ChasDisabled(StateCore *core)
{
  soft_retry_chasdisable = false;
  retry_dir_area2 = false;
  need_spear_id = 0;

  // 如果按下该按钮，进入一个再不跳出的循环，直到跳出状态
  if (MOD::farcon.button_second_half[2] == 1)
  {
    float cooldown = 0.0f;
    // 灯带分成六段；已亮段数等于当前选择的矛头编号加一。
    APP::main_loop.test_led.Segment(6);
    while (MOD::farcon.button_second_half[0] != 1)
    {
      // 再次按下按键，就能选择取哪个矛头了
      if (MOD::farcon.button_second_half[2] == 1 && DWT_GetTimeline_Sec() - cooldown > 0.35f)
      {
        need_spear_id++;
        if (need_spear_id > 5) need_spear_id = 0;
        
        // 根据选择的矛头编号，显示不同的颜色
        for (uint8_t i = 0; i < 6; ++i)
        {
          if (i <= need_spear_id)
          {
            APP::main_loop.test_led.SegLit(i, (i % 2 == 0) ? Color::Yellow : Color::Green);
          }
          else
          {
            APP::main_loop.test_led.SegLit(i, Color(0.0f, 0.0f, 0.0f));
          }
        }

        cooldown = DWT_GetTimeline_Sec();
      }

      Seq::Wait(0.01f);
    }

    // 选择完成，进入取矛头流程
    retry_fetch_spear = true;
    return;
  }

  // 如果按下该按钮，直接进入二区，参考 Retry_Direct_ToArea2
  if (MOD::farcon.button_second_half[3] == 1)
  {
    APP::monit.LogRespond("Retry: Direct to Area2.");
    retry_dir_area2 = true;
    return;
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
      Area2::RunIntoArea2(PATH::A12_3(), Vec3(0.1f, 0.1f, 0.05f), true);
      break;
    case Area2Code::ENTER_1:
      Area2::RunIntoArea2(PATH::A12_2(), Vec3(0.1f, 0.1f, 0.05f), false);
      break;
    case Area2Code::ENTER_2:
      Area2::RunIntoArea2(PATH::A12_1(), Vec3(0.1f, 0.1f, 0.05f), true);
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

  // 上斜坡，跑到擂台上的中间位置
  MOVE::MoveToTargGes(Vec3(11.45f, 0.77f, 0.0f));

  // 放低腿，防止sick打出去
  APP::leg.F_Origin();
  APP::leg.B_Origin();

  // 跑到九宫格第二层中间位置
  MOVE::MoveToTargPos(PATH::A23_2(), Vec3(0.01f, 0.01f, 0.01f));
  MOVE::MoveToTargGes(Arena_R2_L2_Mid_Ges);

  Area3_SickFix();

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

  APP::get_block.Put();
  Seq::WaitUntil([]()
  {
    return !APP::get_block.IsBusy();
  }, 15.0f);

  Seq::Wait(1.0f);

  // 收腿后返回三区重试点，准备接收后续操作手指令
  APP::leg.F_AlmostOrigin();
  APP::leg.B_AlmostOrigin();

  // 先回九宫格中点，保证回撤路径的起点一致
  MOVE::MoveToTargGes(Vec3(10.75f, 5.20f, 1.571f));
  MOVE::MoveToTargPos(PATH::A32_RetryBack(), Vec3(0.01f, 0.01f, 0.01f));

  core->GetCurState().Complete = true;
}

// 放二层块的封装
static void PutBlock_L2(uint8_t id)
{
  // 此时接收到R1的命令，等待2.0s，R1让开
  Seq::Wait(2.0f);

  // 根据需求，走不同的路线
  switch (id)
  {
    case 0:
    {
      // 放最靠近二区的格子
      MOVE::MoveToTargPos(PATH::W_TO_N4(), Vec3(0.01f, 0.01f, 0.015f));
      break;
    }
    case 1:
    {
      // 放中间的格子
      MOVE::MoveToTargPos(PATH::W_TO_N5(), Vec3(0.01f, 0.01f, 0.015f));
      break;
    }
    case 2:
    {
      // 放最靠近三区的格子
      MOVE::MoveToTargPos(PATH::W_TO_N6(), Vec3(0.01f, 0.01f, 0.015f));
      break;
    }
  }

  // 等待到达
  APP::monit.LogInfo("Forward is Arena.");

  Seq::Wait(0.5f);

  // 抬腿
  APP::leg.SetFrontPos(-388);
  APP::leg.SetBackPos(-380);

  // 等腿抬好
  Seq::WaitUntil([](){
    return ( fabs(APP::leg.real_front_pos_mm - APP::leg.front_pos) < 6  && fabs(APP::leg.real_back_pos_mm - APP::leg.back_pos) < 6 );
  });

  // 根据需求，走不同的路线
  switch (id)
  {
    case 0:
    {
      // 放最靠近二区的格子
      MOVE::MoveToTargGes(Vec3(10.21f, 5.20f, 1.571f));
      break;
    }
    case 1:
    {
      // 放中间的格子
      MOVE::MoveToTargGes(Vec3(10.75f, 5.20f, 1.571f));
      break;
    }
    case 2:
    {
      // 放最靠近三区的格子
      MOVE::MoveToTargGes(Vec3(11.29f, 5.20f, 1.571f));
      break;
    }
  }

  // 放块
  APP::get_block.Put();

  APP::monit.LogSpec("Put At:%.2f, %.2f", System.position.x, System.position.y);

  // 等待放块完成
  Seq::Wait(7.0);

  // 得先放下腿，再回家
  APP::leg.F_AlmostOrigin();
  APP::leg.B_AlmostOrigin();

  // 根据需求，走不同的路线回到光通信位置，等待下一个指令
  switch (id)
  {
    case 0:
    {
      // 放最靠近二区的格子
      MOVE::MoveToTargPos(PATH::N4_BACK(), Vec3(0.015f, 0.015f, 0.015f));
      break;
    }
    case 1:
    {
      // 放中间的格子
      MOVE::MoveToTargPos(PATH::N5_BACK(), Vec3(0.015f, 0.015f, 0.015f));
      break;
    }
    case 2:
    {
      // 放最靠近三区的格子
      MOVE::MoveToTargPos(PATH::N6_BACK(), Vec3(0.015f, 0.015f, 0.015f));
      break;
    }
  }
}

void RetryZone_PutL2(uint8_t id)
{
  // 走到中心点
  MOVE::MoveToTargPos(PATH::A23_Retry(), Vec3(0.01f, 0.01f, 0.01f));
  
  // 尝试西克纠正
  Area3_SickFix();

  switch (id)
  {
    case 0:   // 放四号格子
    {
      MOVE::MoveToTargGes(Vec3(10.21f, 5.20f, 1.571f));
      // 抬起腿
      APP::leg.SetFrontPos(-388);
      APP::leg.SetBackPos(-380);
      Seq::Wait(0.5f);
      // 放块
      APP::get_block.Put();
      break;
    }
    case 1:   // 放五号格子
    {
      MOVE::MoveToTargGes(Vec3(10.75f, 5.20f, 1.571f));
      // 抬起腿
      APP::leg.SetFrontPos(-388);
      APP::leg.SetBackPos(-380);
      Seq::Wait(0.5f);
      // 放块
      APP::get_block.Put();
      break;
    }
    case 2:   // 放六号格子
    {
      MOVE::MoveToTargGes(Vec3(11.29f, 5.20f, 1.571f));
      // 抬起腿
      APP::leg.SetFrontPos(-388);
      APP::leg.SetBackPos(-380);
      Seq::Wait(0.5f);
      // 放块
      APP::get_block.Put();
      break;
    }
    default:
    {
      APP::monit.LogError("Invalid Retry Put ID!");
      break;
    }
  }

  // 等块放完
  Seq::WaitUntil([]()
  {
    return !APP::get_block.IsBusy();
  });
  Seq::Wait(1.0f);

  // 放腿
  APP::leg.F_AlmostOrigin();
  APP::leg.B_AlmostOrigin();  

  // 回到中间
  MOVE::MoveToTargGes(Vec3(10.75f, 5.20f, 1.571f));

  // 返回重试区
  MOVE::MoveToTargPos(PATH::A32_RetryBack(), Vec3(0.01f, 0.01f, 0.01f));
}


/*******************************************************************************
 *  @brief 三区：等待 操作手 指令（非阻塞循环，200Hz 调用）
 ******************************************************************************/
/**
 * @brief 此时，R2靠边站好（在重试区内）。准备接收操作手信息。并路由至不同状态
 * @note 这个函数里如果没有阻塞，那么根据状态机的设计，这个状态将会以 200Hz 循环
 * @param core 
 */
void Wait_Human_Instru(StateCore *core)
{
  retry_area3 = false;

  /// @brief 根据操作手的指令，路由至不同状态
  Area3ActCode code = Area3ActCode::Idle;

  // 无指令则持续阻塞，等待下面的等待和做动作
  while (code == Area3ActCode::Idle)
  {
    if (MOD::farcon.button_second_half[2] == 1)
    {
      code = Area3ActCode::Put_L2_N4_Block;
    }
    else if (MOD::farcon.button_second_half[3] == 1)
    {
      code = Area3ActCode::Put_L2_N5_Block;
    }
    else if (MOD::farcon.button_second_half[4] == 1)
    {
      code = Area3ActCode::Put_L2_N6_Block;
    }
    Seq::Wait(0.1f);
  }

  // 灯带：运行
  APP::main_loop.test_led.Running(Color::Green, 0.1f, 0.5f);

  switch (code)
  {
    case Area3ActCode::Put_L2_N4_Block:
    {
      RetryZone_PutL2(0);
      break;
    }
    case Area3ActCode::Put_L2_N5_Block:
    {      
      RetryZone_PutL2(1);
      break;
    }
    case Area3ActCode::Put_L2_N6_Block:
    {
      RetryZone_PutL2(2);
      break;
    }
    case Area3ActCode::Put_L3:
    {
      break;
    }
    case Area3ActCode::Combine:
    {
      Robot_Combine();
      break;
    }
    default:
    {
      break;
    }
  }
  // 清除，防止重复执行
  // APP::comm_center.Reset_R1_InstruCode();
  code = Area3ActCode::Idle;
}

/*******************************************************************************/

void Robot_Combine()
{
  // 首先，走到R1跟前
  MOVE::MoveToTargPos(PATH::W_TO_COMB(), Vec3(0.015f, 0.015f, 0.015f));

  APP::leg.F_Origin();
  APP::leg.B_Origin();

  // 遥控器确认
  while (MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }
  Seq::Wait(1.0f);
  APP::main_loop.test_led.TempBlink(Color::Orange, 3, 0.3f);

  // 抬腿
  APP::leg.SetFrontPos(-375);
  APP::leg.SetBackPos(-375);

  // 等待腿抬好
  Seq::Wait(5.0f);

  // 遥控器确认
  while (MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }
  Seq::Wait(1.0f);

  APP::main_loop.test_led.TempBlink(Color::Orange, 3, 0.3f);

  // 向 y 正方向走，同时锁定 x 为_avg_x，yaw 角为1.571，直到小黄0号看到障碍
  while (true)
  {
    // -------直接向前走-------- // 
    float yaw_err = 1.571f - System.position.z;
    APP::chas.Move(Vec3(0.2f, 0.0f, yaw_err * 1.0f), 0.006f);
    Seq::Wait(0.005f);

    if (IR_Read(APP::leg.detector[0])) break;
  }

  // 此时，小黄看到了障碍，停止移动
  APP::chas.Move(Vec3(0.0f, 0.0f, 0.0f), 0.5f);
  Seq::Wait(0.5f);

  // 遥控器确认
  while (MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }
  APP::main_loop.test_led.TempBlink(Color::Orange, 3, 0.3f);

  // 抬前腿
  APP::leg.F_Origin();

  // 等待前腿抬好
  Seq::Wait(5.0f);

  // 遥控器确认
  while (MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }
  APP::main_loop.test_led.TempBlink(Color::Orange, 3, 0.3f);

  // 继续慢慢向前走，同上锁定，直到小黄2号看到障碍（最长20s超时）
  float combine_start_t = DWT_GetTimeline_Sec();

  while (!IR_Read(APP::leg.detector[2]) && DWT_GetTimeline_Sec() - combine_start_t < 15.0f)
  {
    float yaw_err = 1.571f - System.position.z;
    APP::chas.Move(Vec3(0.15f, 0.0f, yaw_err * 0.5f), 0.5f);
    Seq::Wait(0.5f);

    yaw_err = 1.571f - System.position.z;
    APP::chas.Move(Vec3(0.0f, 0.0f, yaw_err * 1.0f), 0.5f);
    Seq::Wait(0.5f);
  }

  // 此时，小黄看到了障碍，停止移动
  APP::chas.Move(Vec3(0.0f, 0.0f, 0.0f), 0.5f);
  Seq::Wait(0.5f);

  // 遥控器确认
  while (MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }
  APP::main_loop.test_led.TempBlink(Color::Orange, 3, 0.3f);

  APP::chas.Move(Vec3(-0.05f, 0.0f, 0.0f), 0.5f);
  Seq::Wait(0.5f);

  // 抬后腿
  APP::leg.B_Origin();

  // 等待后腿抬好
  Seq::Wait(5.0f);

  // 遥控器确认
  while (MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }
  APP::main_loop.test_led.TempBlink(Color::Orange, 3, 0.3f);

  // 缓慢向前走
  APP::chas.Move(Vec3(0.08f, 0.0f, 0.0f), 1.5f);
  Seq::Wait(1.5f);

  // 结束
  Seq::Wait(3600.0f);
}


/*******************************************************************************
 *  @brief 重试：直接前往二区
 ******************************************************************************/
void Retry_Direct_ToArea2(StateCore *core)
{
  retry_dir_area2 = false;

  MOVE::MoveToTargGes(Vec3(1.3f, 5.0f, 1.5707f));
  core->GetCurState().Complete = true;
}


/*******************************************************************************
 *  @brief 重试：光通信异常，重新请求
 ******************************************************************************/
void Retry_Comm_Error(StateCore *core)
{
  retry_comm_err = false;

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

void Area3_SickFix()
{
  static bool _sick_fixed = false;

  if (_sick_fixed) return;

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

  int cnt = 0;
  float avg_x = 0.0f;
  while (cnt < 10) { avg_x += System.position.x; cnt++; Seq::Wait(0.005f); }
  avg_x /= 10.0f;
  float arena_x_bias = 10.75f - avg_x;
  _sick_fixed = true;
  APP::monit.LogInfo("[arena_bias]:%.3f", arena_x_bias);
  System.SetPositionBias(Vec3(arena_x_bias, 0.0f, 0.0f));
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
