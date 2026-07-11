#include "Area1_Act.hpp"
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
#include "CommCenter.hpp"
#include "Get_Block.hpp"
#include "MainLoop.hpp"
#include "PathCenter/PathCenter.hpp"

/// @brief 取矛头时的X方向偏移，由遥控器按键调整，单位m
float fetch_spear_x_bias = 0.0f;

static const float dist_sick_center = 0.34f;
static const float spear_sick_value[6] =
    {0.115f,
     0.315f,
     0.515f,
     0.715f,
     0.915f,
     1.115f};

/********************************************************************************/
namespace OriginTo // 从原点出发的
{
template <size_t N>
void StartSpearChase(const Path<N> &path, Vec3 tolerance)
{
  if (!APP::path_chaser.ChasePath(path, tolerance))
  {
    // 如果路径出问题，拒绝启动，报错，停机
    APP::monit.LogError("[AutoGraph] MoveToTargPos rejected by PathChaser.");
    Seq::Wait(3600.0f);
  }
}

void SpearChase(uint8_t spear_id, Vec3 tolerance)
{
  switch (spear_id)
  {
    case 0:
      StartSpearChase(PATH::sttr1(), tolerance);
      break;
    case 1:
      StartSpearChase(PATH::sttr2(), tolerance);
      break;
    case 2:
      StartSpearChase(PATH::sttr3(), tolerance);
      break;
    case 3:
      StartSpearChase(PATH::sttr4(), tolerance);
      break;
    case 4:
      StartSpearChase(PATH::sttr5(), tolerance);
      break;
    case 5:
      StartSpearChase(PATH::sttr6(), tolerance);
      break;
    default:
      break;
  }
}

void SpearChase_Blue(uint8_t spear_id, Vec3 tolerance)
{
  switch (spear_id)
  {
    case 0:
      StartSpearChase(PATH::sttr_b1(), tolerance);
      break;
    case 1:
      StartSpearChase(PATH::sttr_b2(), tolerance);
      break;
    case 2:
      StartSpearChase(PATH::sttr_b3(), tolerance);
      break;
    case 3:
      StartSpearChase(PATH::sttr_b4(), tolerance);
      break;
    case 4:
      StartSpearChase(PATH::sttr_b5(), tolerance);
      break;
    case 5:
      StartSpearChase(PATH::sttr_b6(), tolerance);
      break;
    default:
      break;
  }
}
}
/********************************************************************************/
namespace DockTo // 从对接位置出发的
{
template <size_t N>
void StartSpearChase(const Path<N> &path, Vec3 tolerance)
{
  if (!APP::path_chaser.ChasePath(path, tolerance))
  {
    // 如果路径出问题，拒绝启动，报错，停机
    APP::monit.LogError("[AutoGraph] MoveToTargPos rejected by PathChaser.");
    Seq::Wait(3600.0f);
  }
}

void DockSpearChase(uint8_t spear_id, Vec3 tolerance)
{
  switch (spear_id)
  {
    case 0:
      StartSpearChase(PATH::DTT_1(), tolerance);
      break;
    case 1:
      StartSpearChase(PATH::DTT_2(), tolerance);
      break;
    case 2:
      StartSpearChase(PATH::DTT_3(), tolerance);
      break;
    default:
    {
      APP::monit.LogError("[A1Act] Red Area Only Can Fetch 0 to 2 !");
      Seq::Wait(3600.0f);
      break;
    }
      
  }
}

void DockSpearChase_Blue(uint8_t spear_id, Vec3 tolerance)
{
  switch (spear_id)
  {
    case 3:
      StartSpearChase(PATH::DTT_B4(), tolerance);
      break;
    case 4:
      StartSpearChase(PATH::DTT_B5(), tolerance);
      break;
    case 5:
      StartSpearChase(PATH::DTT_B6(), tolerance);
      break;
    default:
      APP::monit.LogError("[A1Act] Blue Area Only Can Fetch 3 to 5 !");
      Seq::Wait(3600.0f);
      break;
  }
}
}
/********************************************************************************/
namespace ToDock // 从对接位置出发的
{
template <size_t N>
void StartChase(const Path<N> &path, Vec3 tolerance)
{
  if (!APP::path_chaser.ChasePath(path, tolerance))
  {
    // 如果路径出问题，拒绝启动，报错，停机
    APP::monit.LogError("[AutoGraph] Can't Go to Dock, To Far.");
    Seq::Wait(3600.0f);
  }
}

void LetsGo(uint8_t spear_id, Vec3 tolerance)
{
  switch (spear_id)
  {
    case 0:
      StartChase(PATH::TTD_1(), tolerance);
      break;
    case 1:
      StartChase(PATH::TTD_2(), tolerance);
      break;
    case 2:
      StartChase(PATH::TTD_3(), tolerance);
      break;
    case 3:
      StartChase(PATH::TTD_4(), tolerance);
      break;
    case 4:
      APP::path_chaser.ChaseGes(Vec3(1.3f, 5.00f, 1.57f));
      break;
    case 5:
      APP::path_chaser.ChaseGes(Vec3(1.3f, 5.00f, 1.57f));
      break;
    default:
      break;
  }
}

void LetsGo_Blue(uint8_t spear_id, Vec3 tolerance)
{
  switch (spear_id)
  {
    case 0:
      StartChase(PATH::TTD_B1(), tolerance);
      break;
    case 1:
      StartChase(PATH::TTD_B2(), tolerance);
      break;
    case 2:
      StartChase(PATH::TTD_B3(), tolerance);
      break;
    case 3:
      StartChase(PATH::TTD_B4(), tolerance);
      break;
    case 4:
      StartChase(PATH::TTD_B5(), tolerance);
      break;
    case 5:
      StartChase(PATH::TTD_B6(), tolerance);
      break;
    default:
      break;
  }
}
}
/********************************************************************************/
void Aim_And_Fetch(int spear_id)
{
  int sick_cnt = 0;
  const float sick_kd = 0.1f;
  const float sick_d_alpha = 0.7f;
  float last_err_dist = spear_sick_value[spear_id] - APP::comm_center.sick_data_m[1];
  float d_err_dist_filt = 0.0f;

  // 以西克自己为基准校准，直到西克值满足要求（0.5cm）
  while (sick_cnt < 10)
  {
    if (fabs(spear_sick_value[spear_id] - APP::comm_center.sick_data_m[1]) < 0.004)
    {
      sick_cnt++;
    }
    else
      sick_cnt = 0;

    // 利用西克获取相对误差(m)
    float err_dist = spear_sick_value[spear_id] - APP::comm_center.sick_data_m[1];
    float d_err_dist = (err_dist - last_err_dist) / 0.005f;
    d_err_dist_filt = d_err_dist_filt * sick_d_alpha + d_err_dist * (1.0f - sick_d_alpha);
    float err_spd = err_dist * 2.7f - sick_kd * d_err_dist_filt;
    last_err_dist = err_dist;

    // if (fabs(err_spd) < 0.05f)
    // {
    //   err_spd < 0 ? err_spd = -0.05f : err_spd = 0.05f;
    // }

    Vec2 sick_fix = Vec2(err_spd, 0).Rotate(-System.position.z);

    APP::chas.Move(sick_fix.ToVec3(), 0.01f);
    Seq::Wait(0.005f); // 200hz更新频率
  }

  APP::monit.LogInfo("Finished Aim.");

  APP::chas.Move((0,0,0), 0.25f);
  Seq::Wait(0.25f);

  // 向前走，取矛头
  int walk_cnt_ms = 0;
  while (walk_cnt_ms < 2500) // 0.4s
  {
    // 利用西克获取相对误差(m)
    float err_dist = spear_sick_value[spear_id] - APP::comm_center.sick_data_m[1];
    float err_spd = err_dist * 5.0f;

    // Vec2 sick_fix = Vec2(err_spd, 0).Rotate(-System.position.z);
    // float y_err = 5.51f - System.position.y;
    Vec2 go_ahead = Vec2(0, 0.1f).Rotate(-System.position.z);

    // APP::chas.Move(go_ahead.ToVec3() + sick_fix.ToVec3(), 0.01f);
    APP::chas.Move(go_ahead.ToVec3(), 0.01f);

    Seq::Wait(0.005f); // 200hz更新频率
    walk_cnt_ms += 5;
  }

  APP::monit.LogInfo("Move to target position.");

  APP::jaw.CloseClaw();
  Seq::Wait(0.15);
}

/********************************************************************************/

void Fetch()
{
  APP::monit.LogInfo("Is IR got?%.2f,%.2f,%.2f", System.position.x, System.position.y, System.position.z);
  
  // 检查小黄是否读取到
  while(!APP::main_loop.spear_detector.Read())
  {
    // 小黄没看到，那说明还不够靠前，缓慢向前走。
    Vec2 targ_world_vec_xy = Vec2(0.0f, 0.075f);
    // 转为局部坐标
    Vec2 targ_body_vec_xy = targ_world_vec_xy.Rotate(-System.position.z);

    // 锁定yaw角为 -1.5707 rad 锁着yaw角度，不用太强
    float yaw_err = -1.5707f - System.position.z;
    Vec3 targ_body_vec = targ_body_vec_xy.ToVec3();
    targ_body_vec.z = yaw_err * 0.8f;

    // APP::monit.LogInfo("ir:%d", APP::main_loop.spear_detector.Read() ? 1 : 0);

    // 向前走，直到小黄看到为止
    APP::chas.Move(targ_body_vec, 0.006f);
    Seq::Wait(0.005f); // 200hz更新频率
  }

  APP::monit.LogInfo("IR got, Catch.%.2f,%.2f,%.2f", System.position.x, System.position.y, System.position.z);
  // 距离安全，夹住
  APP::jaw.CloseClaw();
  Seq::Wait(0.15);
}

/********************************************************************************/

void FetchBlue()
{
  APP::monit.LogInfo("Is IR got?%.2f,%.2f,%.2f", System.position.x, System.position.y, System.position.z);
  // 检查小黄是否读取到
  while(!APP::main_loop.spear_detector.Read())
  {
    // 小黄没看到，那说明还不够靠前，缓慢向前走。
    Vec2 targ_world_vec_xy = Vec2(0.0f, -0.07f);
    // 转为局部坐标
    Vec2 targ_body_vec_xy = targ_world_vec_xy.Rotate(-System.position.z);

    // 锁定yaw角为 1.5707 rad 锁着yaw角度，不用太强
    float yaw_err = 1.5707f - System.position.z;
    Vec3 targ_body_vec = targ_body_vec_xy.ToVec3();
    targ_body_vec.z = yaw_err * 0.8f;

    // APP::monit.LogInfo("ir:%d", APP::main_loop.spear_detector.Read() ? 1 : 0);

    // 向前走，直到小黄看到为止
    APP::chas.Move(targ_body_vec, 0.006f);
    Seq::Wait(0.005f); // 200hz更新频率
  }

  APP::monit.LogInfo("IR got, Catch.%.2f,%.2f,%.2f", System.position.x, System.position.y, System.position.z);
  // 距离安全，夹住
  APP::jaw.CloseClaw();
  Seq::Wait(0.15);
}

/**************************************************************/

/**
 * @brief 从原点出发取矛头
 * @param 要取第几个头 0~5
 */
void FetchSpear_FromOrigin(uint8_t spear_id)
{
  APP::jaw.PitchDown();
  APP::jaw.OpenClaw();

  // 应用取矛头X方向偏移
  System.SetPositionBias(Vec3(fetch_spear_x_bias, 0.0f, 0.0f));

  OriginTo::SpearChase(spear_id, Vec3(0.02f, 0.02f, 0.02f));

  uint32_t log_tick = 0;
  // 追逐路径，直到完成
  while (!APP::path_chaser.IsFinished())
  {
    Vec3 speed_vec = APP::path_chaser.GetCmdBody();
    Vec3 w_vec = APP::path_chaser.GetCmdWorld();
    auto tg_p = APP::path_chaser.GetDebugTargetByCurrentT();
    APP::chas.Move(speed_vec, 0.01f);

    if (++log_tick >= 10)
    {
        log_tick = 0;
        APP::monit.LogInfo("pos(%.2f, %.2f, %.2f)",
                           System.position.x, System.position.y, System.position.z);
        APP::monit.LogSpec("v(%.2f, %.2f, %.2f)",
                           w_vec.x, w_vec.y, w_vec.z);
        APP::monit.LogRespond("tp:%.2f,%.2f,%.2f", tg_p.pos.x, tg_p.pos.y, tg_p.yaw);
    }

    Seq::Wait(0.005f); // 200hz更新频率
  }

  APP::monit.LogInfo("Finished Run.");

  Seq::Wait(1.0f);

  // Aim_And_Fetch(spear_id);
  Fetch();

  // 记录抓取位置，以及西克数据
  APP::monit.LogSpec("Catch At:(%.3f,%.3f), sick:%.3f", APP::comm_center.slam_pos.x,
                     APP::comm_center.slam_pos.y, APP::comm_center.sick_data_m[1]);

  // 先往上抬 1.5cm
  APP::jaw.ChangePos(0, 20);
  Seq::Wait(0.25);

  // 然后抬起 Pitch爪子
  APP::jaw.PitchUp();
}

/********************************************************************************/

/**
 * @brief 从对接的位置出发，取矛头
 *
 * @param spear_id 对于红方来说，只能取前三个 (0~2)
 */
void FetchSpear_FromDock(uint8_t spear_id)
{
  APP::jaw.PitchDown();
  APP::jaw.OpenClaw();

  // 应用取矛头X方向偏移
  System.SetPositionBias(Vec3(fetch_spear_x_bias, 0.0f, 0.0f));

  // 根据矛头编号，选择对应的路径（技能挑战赛）
  DockTo::DockSpearChase(spear_id, Vec3(0.015f, 0.015f, 0.02f));

  uint32_t log_tick = 0;

  // 追逐路径，直到完成
  while (!APP::path_chaser.IsFinished())
  {
    Vec3 speed_vec = APP::path_chaser.GetCmdBody();
    Vec3 w_vec = APP::path_chaser.GetCmdWorld();

    APP::chas.Move(speed_vec, 0.01f);

    if (++log_tick >= 20)
    {
      log_tick = 0;
      APP::monit.LogInfo("w:%.2f,%.2f,%.2f", 
        System.position.x, System.position.y, System.position.z);
      APP::monit.LogRespond("e:%.2f", APP::path_chaser.GetRemainDist());
      APP::monit.LogSpec("v:%.2f,%.2f,%.2f", w_vec.x, w_vec.y, w_vec.z);
    }
    Seq::Wait(0.005f); // 200hz更新频率
  }

  APP::monit.LogInfo("Finished Aim.");

  // Aim_And_Fetch(spear_id);
  Fetch();

  // 记录抓取位置，以及西克数据
  APP::monit.LogInfo("Catch At:(%.3f,%.3f), sick:%.3f", APP::comm_center.slam_pos.x,
                     APP::comm_center.slam_pos.y, APP::comm_center.sick_data_m[1]);

  // 先往上抬 1.5cm
  APP::jaw.ChangePos(0, 20);
  Seq::Wait(0.25);

  // 然后抬起 Pitch爪子
  APP::jaw.PitchUp();
}

/********************************************************************************/

/**
 * @brief 原版——红区的版本
 * 
 * @param spear_id 
 */
void GotoDock(uint8_t spear_id)
{
  // 去对接点时清除取矛头偏移，确保对接位置准确
  System.SetPositionBias(Vec3(0.0f, 0.0f, 0.0f));

  ToDock::LetsGo(spear_id, Vec3(0.035f, 0.05f, 0.02f));

  while (!APP::path_chaser.IsFinished())
  {
    Vec3 speed_vec = APP::path_chaser.GetCmdBody();
    APP::chas.Move(speed_vec, 0.01f);

    Seq::Wait(0.005f); // 200hz更新频率
  }

  APP::monit.LogInfo("Move to target position.");
}

/********************************************************************************/

/**
 * @brief 蓝区的版本
 * 
 * @param spear_id 
 */
void GotoDock_Blue(uint8_t spear_id)
{
  ToDock::LetsGo_Blue(spear_id, Vec3(0.025f, 0.05f, 0.02f));

  while (!APP::path_chaser.IsFinished())
  {
    Vec3 speed_vec = APP::path_chaser.GetCmdBody();
    APP::chas.Move(speed_vec, 0.01f);

    Seq::Wait(0.005f); // 200hz更新频率
  }

  APP::monit.LogInfo("Move to target position.");
}

/********************************************************************************/

/**
 * @brief 从对接的位置出发，取矛头（蓝区版本）
 *
 * @param spear_id 对于蓝方来说，只能取后三个 (3~5)
 */
void FetchSpear_FromDock_Blue(uint8_t spear_id)
{
  APP::jaw.PitchDown();
  APP::jaw.OpenClaw();

  // 根据矛头编号，选择对应的路径（技能挑战赛）
  DockTo::DockSpearChase_Blue(spear_id, Vec3(0.015f, 0.015f, 0.02f));

  uint32_t log_tick = 0;

  // 追逐路径，直到完成
  while (!APP::path_chaser.IsFinished())
  {
    Vec3 speed_vec = APP::path_chaser.GetCmdBody();

    APP::chas.Move(speed_vec, 0.01f);

    if (++log_tick >= 20)
    {
      log_tick = 0;
    }
    Seq::Wait(0.005f); // 200hz更新频率
  }

  APP::monit.LogInfo("Finished Aim.");

  // Aim_And_Fetch(spear_id);
  FetchBlue();

  // 记录抓取位置，以及西克数据
  APP::monit.LogInfo("Catch At:(%.3f,%.3f), sick:%.3f", APP::comm_center.slam_pos.x,
                     APP::comm_center.slam_pos.y, APP::comm_center.sick_data_m[1]);

  // 先往上抬 1.5cm
  APP::jaw.ChangePos(0, 20);
  Seq::Wait(0.25);

  // 然后抬起 Pitch爪子
  APP::jaw.PitchUp();
}

/********************************************************************************/

/**
 * @brief 从原点出发取矛头
 * @param 要取第几个头 0~5
 */
void FetchSpear_FromOrigin_Blue(uint8_t spear_id)
{
  APP::jaw.PitchDown();
  APP::jaw.OpenClaw();

  OriginTo::SpearChase_Blue(spear_id, Vec3(0.02f, 0.02f, 0.02f));

  // 追逐路径，直到完成
  while (!APP::path_chaser.IsFinished())
  {
    Vec3 speed_vec = APP::path_chaser.GetCmdBody();
    APP::chas.Move(speed_vec, 0.01f);

    Seq::Wait(0.005f); // 200hz更新频率
  }

  APP::monit.LogInfo("Finished Run.");

  Seq::Wait(1.0f);

  FetchBlue();

  // 记录抓取位置，以及西克数据
  APP::monit.LogSpec("Catch At:(%.3f,%.3f), sick:%.3f", APP::comm_center.slam_pos.x,
                     APP::comm_center.slam_pos.y, APP::comm_center.sick_data_m[1]);

  // 先往上抬 1.5cm
  APP::jaw.ChangePos(0, 20);
  Seq::Wait(0.25);

  // 然后抬起 Pitch爪子
  APP::jaw.PitchUp();
}
