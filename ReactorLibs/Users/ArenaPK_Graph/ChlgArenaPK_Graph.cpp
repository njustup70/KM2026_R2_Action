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
#include "Area3_Act.hpp"

/// @brief 所有的路径文件
#include "PathCenter.hpp"
#include "ttd_1.hpp"
#include "ttd_2.hpp"
#include "ttd_3.hpp"

#include "a12_1.hpp"
static void Response_Button();
// static void FetchBlockOnTheGround();
StateGraph Arena_PK_Graph("Arena_PK");

// 状态定义
static void Waiting_for_Start(StateCore *core);
static void Put_First_Block(StateCore *core);
static void Waiting_R1_Instru(StateCore *core);
static void Fetch2_From_Mid(StateCore *core);
// 从R1等待位置出发的状态函数
static void Fetch_0_FromWait();
static void Fetch_1_FromWait();

Vec2 Arena_R2_KFS_FetchPos[2] = {
  Vec2(11.1, 3.405), Vec2(11.1, 2.685)
}; // 块之间间隔：0.175 + 0.175 + 0.37 = 0.72 m

Vec2 Arena_R2_PutKFSPos[3] = {
  Vec2(10.21f, 5.15f), Vec2(10.75f, 5.15f), Vec2(11.29f, 5.15f)
}; // 分别对应4、5、6格子

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
  // 回去取 0 1 号块
  StateBlock& s_fetch_0_1_block = Arena_PK_Graph.AddState("fetch_0_1_block");


  /**----------     配置状态行为函数    ----------**/
  s_wait.StateAction = Waiting_for_Start;
  s_put_first_block.StateAction = Put_First_Block;
  s_waiting_R1_instru.StateAction = Waiting_R1_Instru;
  s_fetch_0_1_block.StateAction = Fetch2_From_Mid;

  /**----------     进行状态间链接    ----------**/
  s_wait.LinkTo(&(s_wait.Complete), s_put_first_block);
  s_put_first_block.LinkTo(&(s_put_first_block.Complete), s_fetch_0_1_block);
  s_fetch_0_1_block.LinkTo(&(s_fetch_0_1_block.Complete), s_waiting_R1_instru);

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

  Seq::Wait(1.0f);

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
void SafeFirstPut()
{
  // 检测里边有没有块，如果有，换另一个格子
  if (!APP::get_block.IsBlockPresent())
      return;  // 中间没块，直接放

  APP::monit.LogInfo("Middle block exists, move to right");
  MOVE::MoveToTargGes(Vec3(11.02f, 5.15f, 1.571f));  // 右移

  if (!APP::get_block.IsBlockPresent())
      return;  // 右边没块，直接放

  APP::monit.LogInfo("Right block also exists, move to left");
  MOVE::MoveToTargGes(Vec3(10.48f, 5.15f, 1.571f));  // 左移

  if (APP::get_block.IsBlockPresent())
      APP::monit.LogInfo("All three have blocks, we need combine");
}
/**
 * @brief 这是“九宫藏宝”技能挑战赛，R2出发的第一个动作。先走到九宫格前，然后放块。如果块已经被放置，则换一个方格放块。
 * 
 * @param core 
 */
void Put_First_Block(StateCore *core)
{
  // FetchBlockOnTheGround();
  // 灯带：滚动
  APP::main_loop.test_led.Running(Color::Up_Yellow * 0.75f, 0.5f, 1.0f);

  // 小抬一点腿
  APP::leg.SetFrontPos(-50.0f);
  APP::leg.SetBackPos(-50.0f);
  Seq::Wait(0.3f);

    // 先把块抬起来
  APP::get_block.LiftUp();

  // 首先走上斜坡
  MOVE::MoveToTargGes(Vec3(8.72f, 0.79f, 0.0f));
  MOVE::MoveToTargGes(Vec3(11.38f, 0.77f, 0.0f));

  APP::leg.F_Origin();
  APP::leg.B_Origin();

  MOVE::MoveToTargPos(PATH::A23_2(), Vec3(0.01f, 0.01f, 0.01f));
  MOVE::MoveToTargGes(Vec3(10.75f, 5.20f, 1.571f));

  // 等待，保障稳定性
  Seq::Wait(0.25f);

  // 锁定yaw角为1.571，同时平移直到两路西克激光基本相等
  // APP::comm_center.sick_data_m[0] 约等于 APP::comm_center.sick_data_m[1];
  int sick_cnt = 0;
  const float sick_kd = 0.25f;
  const float sick_d_alpha = 0.7f;
  float last_err_dist = APP::comm_center.sick_data_m[1] - APP::comm_center.sick_data_m[0];
  float d_err_dist_filt = 0.0f;
  uint32_t sick_log_tick = 0;

  while (sick_cnt < 10)
  {
    float err_dist = APP::comm_center.sick_data_m[1] - APP::comm_center.sick_data_m[0];

    if (fabs(err_dist) < 0.004f)
    {
      sick_cnt++;
    }
    else
      sick_cnt = 0;

    float d_err_dist = (err_dist - last_err_dist) / 0.005f;
    d_err_dist_filt = d_err_dist_filt * sick_d_alpha + d_err_dist * (1.0f - sick_d_alpha);
    float err_spd = err_dist * 1.0f - sick_kd * d_err_dist_filt;
    last_err_dist = err_dist;

    Vec2 sick_fix = Vec2(err_spd, 0.0f).Rotate(-System.position.z);

    // 速度保护：限制平移速度不超过0.3m/s
    float sick_speed = sick_fix.Length();
    if (sick_speed > 0.3f)
    {
      sick_fix = sick_fix * (0.3f / sick_speed);
    }

    // 锁定yaw角为 1.5707 rad，不用太强
    float yaw_err = 1.5707f - System.position.z;
    Vec3 targ_body_vec = sick_fix.ToVec3();
    targ_body_vec.z = yaw_err * 0.8f;

    if (++sick_log_tick >= 20)
    {
      sick_log_tick = 0;
      APP::monit.LogInfo("sf:e%.3f v%.3f s0%.3f s1%.3f",
                         err_dist,
                         err_spd,
                         APP::comm_center.sick_data_m[0],
                         APP::comm_center.sick_data_m[1]);
    }

    APP::chas.Move(targ_body_vec, 0.01f);
    Seq::Wait(0.005f); // 200hz更新频率
  }

  APP::monit.LogInfo("Finished sick fix. sick0:%.3f sick1:%.3f",
                     APP::comm_center.sick_data_m[0], APP::comm_center.sick_data_m[1]);
  APP::chas.Move(Vec3(0.0f, 0.0f, 0.0f), 0.25f);
  Seq::Wait(0.5f);

  // 抬起腿
  APP::leg.SetFrontPos(-388);
  APP::leg.SetBackPos(-380);

  // 等腿抬好
  Seq::WaitUntil([](){
    return ( fabs(APP::leg.real_front_pos_mm - APP::leg.front_pos) < 6  && fabs(APP::leg.real_back_pos_mm - APP::leg.back_pos) < 6 );
  });

  // 放块
  APP::get_block.Push();

  // 等待放块完成
  Seq::Wait(1.0f);

  // 等待遥控器的出发指令
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }

  // 走到等待点，等待R1命令
  core->GetCurState().Complete = true;
}


/*******************************************************************************/

/**
 * @brief 假设R2从九宫格中点出发，返回取两个块的逻辑
 * @details 还需要给R1让路
 * @param core 
 */
static void Fetch2_From_Mid(StateCore *core)
{
  // 放完二层块，得先放下腿，再去取块
  APP::leg.F_AlmostOrigin();
  APP::leg.B_AlmostOrigin();
  
  Seq::WaitUntil([](){
    return (fabs(APP::leg.real_front_pos_mm) < 50 && fabs(APP::leg.real_back_pos_mm) < 50);
  });

  // 等待遥控器的出发指令
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }
  // 运行路径跟踪程序
  MOVE::MoveToTargPos(PATH::FFM(), Vec3(0.05f, 0.05f, 0.03f));

  MOVE::MoveToTargGes(Vec3(Arena_R2_KFS_FetchPos[0].x, Arena_R2_KFS_FetchPos[0].y, 0.0));

  // 走到块跟前了
  APP::monit.LogInfo("[Arena] I'm at the block.");

  // 等待遥控器的出发指令
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }
  // 取第一个块
  Area3::FetchBlockOnTheGround();

  // 等待取块完成
  Seq::Wait(1.0f);

  // 等待遥控器的出发指令
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }
  // 移动到第二个块面前
  MOVE::MoveToTargGes(Vec3(Arena_R2_KFS_FetchPos[1].x, Arena_R2_KFS_FetchPos[1].y, 0.0));
  // APP::path_chaser.ChaseGes();

  // 等待到达
  APP::monit.LogInfo("[Arena] I'm at the second block.");

  // 等待遥控器的出发指令
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }
  // 取第二个块
  Area3::FetchBlockOnTheGround();

  // 等待取块完成
  Seq::Wait(1.0f);

  // 返回等待点（光通信朝向 R1）
  MOVE::MoveToTargGes(Vec3(10.1f, 2.1f, -1.571f));

  // 至此：完成所有R2的全自动逻辑，进入R1的指令等待状态
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
      MOVE::MoveToTargPos(PATH::W_TO_N4(), Vec3(0.015f, 0.015f, 0.015f));
      break;
    }
    case 1:
    {
      // 放中间的格子
      MOVE::MoveToTargPos(PATH::W_TO_N5(), Vec3(0.015f, 0.015f, 0.015f));
      break;
    }
    case 2:
    {
      // 放最靠近三区的格子
      MOVE::MoveToTargPos(PATH::W_TO_N6(), Vec3(0.015f, 0.015f, 0.015f));
      break;
    }
  }

  // 等待到达
  APP::monit.LogInfo("Forward is Arena.");

  // 西克校准：根据id确定目标偏移量
  // 几何关系：坐标偏移a时，一边激光近a、另一边远a，差值变化2a
  // N4(左,id=0): X=10.21，偏移0.54m，sick[1]-sick[0]目标为+1.08
  // N5(中,id=1): X=10.75，居中，sick[1]-sick[0]目标为0
  // N6(右,id=2): X=11.29，偏移0.54m，sick[1]-sick[0]目标为-1.08
  float sick_target = 0.0f;
  if (id == 0)
    sick_target = 1.08f;
  else if (id == 2)
    sick_target = -1.08f;

  // 等待，保障稳定性
  Seq::Wait(0.25f);

  // 锁定yaw角为1.571，同时平移直到两路西克激光差值与目标一致
  int sick_cnt = 0;
  const float sick_kd = 0.25f;
  const float sick_d_alpha = 0.7f;
  float last_err_dist = APP::comm_center.sick_data_m[1] - APP::comm_center.sick_data_m[0] - sick_target;
  float d_err_dist_filt = 0.0f;
  uint32_t sick_log_tick = 0;

  while (sick_cnt < 10)
  {
    float err_dist = APP::comm_center.sick_data_m[1] - APP::comm_center.sick_data_m[0] - sick_target;

    if (fabs(err_dist) < 0.004f)
    {
      sick_cnt++;
    }
    else
      sick_cnt = 0;

    float d_err_dist = (err_dist - last_err_dist) / 0.005f;
    d_err_dist_filt = d_err_dist_filt * sick_d_alpha + d_err_dist * (1.0f - sick_d_alpha);
    float err_spd = err_dist * 1.0f - sick_kd * d_err_dist_filt;
    last_err_dist = err_dist;

    Vec2 sick_fix = Vec2(err_spd, 0.0f).Rotate(-System.position.z);

    // 速度保护：限制平移速度不超过0.3m/s
    float sick_speed = sick_fix.Length();
    if (sick_speed > 0.3f)
    {
      sick_fix = sick_fix * (0.3f / sick_speed);
    }

    // 锁定yaw角为 1.5707 rad，不用太强
    float yaw_err = 1.5707f - System.position.z;
    Vec3 targ_body_vec = sick_fix.ToVec3();
    targ_body_vec.z = yaw_err * 0.8f;

    if (++sick_log_tick >= 20)
    {
      sick_log_tick = 0;
      APP::monit.LogInfo("sf:e%.3f v%.3f s0%.3f s1%.3f t%.3f",
                         err_dist,
                         err_spd,
                         APP::comm_center.sick_data_m[0],
                         APP::comm_center.sick_data_m[1],
                         sick_target);
    }

    APP::chas.Move(targ_body_vec, 0.01f);
    Seq::Wait(0.005f); // 200hz更新频率
  }

  APP::monit.LogInfo("Finished sick fix. sick0:%.3f sick1:%.3f id:%d",
                     APP::comm_center.sick_data_m[0], APP::comm_center.sick_data_m[1], id);
  APP::chas.Move(Vec3(0.0f, 0.0f, 0.0f), 0.25f);
  Seq::Wait(0.5f);

  // 抬腿
  APP::leg.SetFrontPos(-388);
  APP::leg.SetBackPos(-380);

  // 等腿抬好
  Seq::WaitUntil([](){
    return ( fabs(APP::leg.real_front_pos_mm - APP::leg.front_pos) < 6  && fabs(APP::leg.real_back_pos_mm - APP::leg.back_pos) < 6 );
  });

  // 放块
  APP::get_block.Put();

  // 等待放块完成
  Seq::Wait(10.0);

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
  auto code = APP::comm_center.Get_R1_InstruCode();
  if (code == Area3ActCode::Idle)
    return;  // 无指令，跳过下面的等待和做动作

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

  switch (code)
  {
    case Area3ActCode::Put_L2_N4_Block:
    {
      PutBlock_L2(0);
      break;
    }
    case Area3ActCode::Put_L2_N5_Block:
    {      
      PutBlock_L2(1);
      break;
    }
    case Area3ActCode::Put_L2_N6_Block:
    {
      PutBlock_L2(2);
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
      Fetch_0_FromWait();
      break;
    }
    case Area3ActCode::Fetch_GroundBlock_1:
    {
      Fetch_1_FromWait();
      break;
    }
    default:
    {
      break;
    }
  }
  // 清除，防止重复执行
  APP::comm_center.Reset_R1_InstruCode();
}

/*******************************************************************************/

void Fetch_0_FromWait()
{
  // 走到第一个块跟前
  MOVE::MoveToTargGes(Vec3(Arena_R2_KFS_FetchPos[0].x, Arena_R2_KFS_FetchPos[0].y, 0.0));

  // 等待到达
  APP::monit.LogInfo("[Arena] I'm at the second block.");

  // 等待遥控器的出发指令
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }
  // 取第二个块
  Area3::FetchBlockOnTheGround();

  // 等待取块完成
  Seq::Wait(1.0f);

  // 返回等待点（光通信朝向 R1）
  MOVE::MoveToTargGes(Vec3(10.1f, 2.1f, -1.571f));
}

void Fetch_1_FromWait()
{
  // 走到第二个块跟前
  MOVE::MoveToTargGes(Vec3(Arena_R2_KFS_FetchPos[1].x, Arena_R2_KFS_FetchPos[1].y, 0.0));

  // 等待到达
  APP::monit.LogInfo("[Arena] I'm at the second block.");

  // 等待遥控器的出发指令
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button();
    Seq::Wait(0.01f);
  }
  // 取第二个块
  Area3::FetchBlockOnTheGround();

  // 等待取块完成
  Seq::Wait(1.0f);

  // 返回等待点（光通信朝向 R1）
  MOVE::MoveToTargGes(Vec3(10.1f, 2.1f, -1.571f));
}






















const static float max_farcon_alpha = 1.8f; // 最大加速度 m/s^2
const static float max_farcon_beta = 1.8f; // 最大角加速度 rad/s^2
void Response_Button()
{
    static float max_velo = 1.5f;
    // if(MOD::farcon.toggle[2] == 0)
    // {
    //   max_velo = 1.5f;
    // }
    // else if(MOD::farcon.toggle[2] == 1)
    // {
    //   max_velo = 0.5f;
    // }

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
        APP::get_block.motor_s2325.SetVel(15.0f);
        is_belt_on = true;
      }
      else
      {
        APP::get_block.motor_2006_fric.SetSpd(0);
        APP::get_block.motor_s2325.SetVel(0);
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
        APP::get_block.motor_2006_feed.SetPos(-1770000.0f);
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
      APP::leg.SetFrontPosSlow(-100.0f, 1.0f);
      APP::leg.SetBackPosSlow(-100.0f, 1.0f);
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
      APP::get_block.Put();
    }

    // if(MOD::farcon.button_first_half[7] == 1)
    // {

    // }

    // 进行 DM 抬升的抬上抬下
    static bool is_dm_lift_up = false;
    static float cooldown_tick = 0.0f;
    if(MOD::farcon.button_first_half[4] == 1  && DWT_GetTimeline_Sec() - cooldown_tick > 1.0f)
    {
      cooldown_tick = DWT_GetTimeline_Sec();
      if (!is_dm_lift_up)
      {
        APP::get_block.LiftUp();
        is_dm_lift_up = true;
      }
      else
      {
        APP::get_block.LiftDown();
        is_dm_lift_up = false;
      }
    }
}

// static void FetchBlockOnTheGround()
// {


//   // 再抬起来一点
//   APP::leg.SetFrontPos(-100);
//   APP::leg.SetBackPos(-100);
//   Seq::WaitUntil([]()
//   {
//     return (fabs(APP::leg.real_back_pos_mm) > 95 && fabs(APP::leg.real_front_pos_mm) > 95);
//   });  

//   // 先把达妙抬起来
//   APP::get_block.dm_lift_pos = -757;  // _lift_vel = 18.0 rad/s

//   // 爪子翻转到竖直，合上爪子
//   APP::vac_claw.pitch_pos = 0.0f;
//   Seq::Wait(1.5f);

//   // 往前走去取物块
//   APP::chas.Move(Vec2(0.1, 0), 3.0);

//   APP::vac_claw.CloseSnap();

//   // 等爪子关上
//   Seq::WaitUntil([]()
//   {
//     return fabs(APP::vac_claw.motor_snap.driver.measure.total_angle - APP::vac_claw.snap_pos) < 1000;
//   });  

//   // 车往前移动一小段吸上物块
//   APP::chas.Move(Vec2(0.05, 0), 7.0);

//   // 开泵
//   APP::get_block.OpenPump();
//   Seq::Wait(1.0f);
//   APP::chas.Move(Vec2(-0.15, 0), 3.0);
//   Seq::Wait(1.3f);
//   // 停下
//   APP::chas.Move(Vec2(0, 0), 3.0);

//   // 开摩擦带
//   APP::get_block._StartBelt();

//   APP::vac_claw.CloseSnap_Hard();
//   // 先翻转一点达妙
//   APP::vac_claw.pitch_pos = 1.7f;
//   Seq::Wait(1.5f);
//   // 再伸出一点
//   APP::vac_claw.stretch_pos = 93000;

//   // // 等爪子伸出
//   // Seq::Wait(0.7f);
//   Seq::WaitUntil([]()
//   {
//     return fabs(APP::vac_claw.motor_stretch.driver.measure.total_angle - APP::vac_claw.stretch_pos) < 100;
//   });  

//   // // 再翻转达妙
//   // APP::vac_claw.pitch_pos = 1.7;

//   // 再等一小会，达妙完全翻回来.这个值必须大一点，否则不知道为什么爪子会先于达妙运动
//   Seq::Wait(2.0f);

//   // 收回
//   APP::vac_claw.stretch_pos = 600.0; // 防堵转，给个略大于0的值
//   // 等着完全收回
//   Seq::WaitUntil([]()
//   {
//     return fabs(APP::vac_claw.motor_stretch.driver.measure.total_angle - APP::vac_claw.stretch_pos) < 5000;
//   });  


//   APP::vac_claw.OpenSnap();
//   APP::get_block.ClosePump();
//   APP::get_block.OpenPRV();

//   APP::get_block.Store();
// }
