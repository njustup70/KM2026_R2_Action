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

/// @brief 所有的路径文件
// #include "r2_area1_1.hpp"
#include "ttd_3.hpp"

#include "stt_rl_2.hpp"
#include "stt_rl_1.hpp"
#include "stt_r.hpp"
#include "stt_rl_4.hpp"
#include "stt_rl_5.hpp"
#include "stt_rl_6.hpp"

#include "dtt_1.hpp"
#include "dtt_2.hpp"
#include "dtt_3.hpp"

#include "ttd_1.hpp"
#include "ttd_2.hpp"
#include "ttd_3.hpp"

#include "a12_2.hpp"
#include "a23_2.hpp"

static const float dist_sick_center = 0.34f;
static const float spear_sick_value[6] =
    {0.115f,
     0.315f,
     0.515f,
     0.715f,
     0.915f,
     1.115f};

namespace OriginTo    // 从原点出发的
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

void StartSpearChaseById(uint8_t spear_id, Vec3 tolerance)
{
  switch (spear_id)
  {
    case 0:
      StartSpearChase(STT_R1, tolerance);
      break;
    case 1:
      StartSpearChase(STT_R2, tolerance);
      break;
    case 2:
      StartSpearChase(STT_R3, tolerance);
      break;
    case 3:
      StartSpearChase(STT_R4, tolerance);
      break;
    case 4:
      StartSpearChase(STT_R5, tolerance);
      break;
    case 5:
      StartSpearChase(STT_R6, tolerance);
      break;
    default:
      break;
  }
}
}

namespace DockTo  // 从对接位置出发的
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

void StartSpearChaseById(uint8_t spear_id, Vec3 tolerance)
{
  switch (spear_id)
  {
    case 0:
      StartSpearChase(DTT_1, tolerance);
      break;
    case 1:
      StartSpearChase(DTT_2, tolerance);
      break;
    case 2:
      StartSpearChase(DTT_3, tolerance);
      break;
    default:
      break;
  }
}
}

namespace ToDock  // 从对接位置出发的
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
      StartChase(TTD_1, tolerance);
      break;
    case 1:
      StartChase(TTD_2, tolerance);
      break;
    case 2:
      StartChase(TTD_3, tolerance);
      break;
    default:
      break;
  }
}
}

/**
 * @brief 从原点出发取矛头
 * @param 要取第几个头 0~5
 */
void FetchSpear_FromOrigin(uint8_t spear_id)
{
  APP::jaw.PitchDown();
  APP::jaw.OpenClaw();

  OriginTo::StartSpearChaseById(spear_id, Vec3(0.015f, 0.05f, 0.02f));

  uint32_t log_tick = 0;
  // 追逐路径，直到完成
  while (!APP::path_chaser.IsFinished())
  {
    Vec3 speed_vec = APP::path_chaser.GetCmdBody();
    // Vec3 world_vec = APP::path_chaser.GetCmdWorld();

    APP::chas.Move(speed_vec, 0.01f);

    if (++log_tick >= 20)
    {
      log_tick = 0;
      APP::monit.LogInfo("stt_pos(%.2f, %.2f, %.2f),sick:%.3f",
                         System.position.x, System.position.y, System.position.z, APP::comm_center.sick_data_m[1]);
    }
    Seq::Wait(0.005f); // 200hz更新频率
  }

  APP::monit.LogInfo("Finished Aim.");

  // 向前走，取矛头
  int walk_cnt_ms = 0;
  while (walk_cnt_ms < 500) // 0.4s
  {
    // 利用西克获取相对误差(m)
    float err_dist = spear_sick_value[spear_id] - APP::comm_center.sick_data_m[1];
    float err_spd = err_dist * 2.4f;

    Vec2 sick_fix = Vec2(err_spd, 0).Rotate(-System.position.z);
    float y_err = 5.51f - System.position.y;
    Vec2 go_ahead = Vec2(0, y_err * 2.4f + 0.10f).Rotate(-System.position.z);

    APP::chas.Move(go_ahead.ToVec3() + sick_fix.ToVec3(), 0.01f);

    Seq::Wait(0.005f); // 200hz更新频率
    walk_cnt_ms += 5;
  }

  APP::monit.LogInfo("Move to target position.");

  APP::jaw.CloseClaw();
  Seq::Wait(0.15);

  // 记录抓取位置，以及西克数据
  APP::monit.LogInfo("Catch At:(%.3f,%.3f), sick:%.3f", APP::comm_center.slam_pos.x,
                     APP::comm_center.slam_pos.y, APP::comm_center.sick_data_m[1]);

  // 先往上抬 1.5cm
  APP::jaw.ChangePos(0, 20);
  Seq::Wait(0.25);

  // 然后抬起 Pitch爪子
  APP::jaw.PitchUp();
}

/**
 * @brief 从对接的位置出发，取矛头
 *
 * @param spear_id 对于红方来说，只能取前三个 (0~2)
 */
void FetchSpear_FromDock(uint8_t spear_id)
{
  APP::jaw.PitchDown();
  APP::jaw.OpenClaw();

  // 根据矛头编号，选择对应的路径（技能挑战赛）
  DockTo::StartSpearChaseById(spear_id, Vec3(0.02f, 0.05f, 0.02f));

  uint32_t log_tick = 0;

  // 追逐路径，直到完成
  while (!APP::path_chaser.IsFinished())
  {
    Vec3 speed_vec = APP::path_chaser.GetCmdBody();
    // Vec3 world_vec = APP::path_chaser.GetCmdWorld();

    APP::chas.Move(speed_vec, 0.01f);

    if (++log_tick >= 20)
    {
      log_tick = 0;
      APP::monit.LogInfo("stt_pos(%.2f, %.2f, %.2f),sick:%.3f",
                         System.position.x, System.position.y, System.position.z, APP::comm_center.sick_data_m[1]);
    }
    Seq::Wait(0.005f); // 200hz更新频率
  }

  APP::monit.LogInfo("Finished Aim.");

  // 向前走，取矛头
  int walk_cnt_ms = 0;
  while (walk_cnt_ms < 500) // 0.4s
  {
    // 利用西克获取相对误差(m)
    float err_dist = spear_sick_value[spear_id] - APP::comm_center.sick_data_m[1];
    float err_spd = err_dist * 2.4f;

    Vec2 sick_fix = Vec2(err_spd, 0).Rotate(-System.position.z);
    float y_err = 5.51f - System.position.y;
    Vec2 go_ahead = Vec2(0, y_err * 2.4f + 0.10f).Rotate(-System.position.z);

    APP::chas.Move(go_ahead.ToVec3() + sick_fix.ToVec3(), 0.01f);

    Seq::Wait(0.005f); // 200hz更新频率
    walk_cnt_ms += 5;
  }

  APP::monit.LogInfo("Move to target position.");

  APP::jaw.CloseClaw();
  Seq::Wait(0.15);

  // 记录抓取位置，以及西克数据
  APP::monit.LogInfo("Catch At:(%.3f,%.3f), sick:%.3f", APP::comm_center.slam_pos.x,
                     APP::comm_center.slam_pos.y, APP::comm_center.sick_data_m[1]);

  // 先往上抬 1.5cm
  APP::jaw.ChangePos(0, 20);
  Seq::Wait(0.25);

  // 然后抬起 Pitch爪子
  APP::jaw.PitchUp();
}

void GotoDock(uint8_t spear_id)
{
  ToDock::LetsGo(spear_id, Vec3(0.015f, 0.015f, 0.02f));
}