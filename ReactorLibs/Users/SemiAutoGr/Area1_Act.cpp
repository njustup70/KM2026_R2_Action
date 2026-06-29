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
#include "AutoGraph.hpp"
#include "CommCenter.hpp"
#include "Get_Block.hpp"
#include "MainLoop.hpp"

/// @brief 所有的路径文件
// #include "r2_area1_1.hpp"
#include "ttd.hpp"


#include "stt_rl_2.hpp"
#include "stt_rl_1.hpp"
#include "stt_r.hpp"
#include "stt_rl_4.hpp"
#include "stt_rl_5.hpp"
#include "stt_rl_6.hpp"

#include "a12_2.hpp"
#include "a23_2.hpp"

static const float dist_sick_center = 0.34f;
static const float spear_sick_value[6] =
    {0.12f,
     0.32f,
     0.52f,
     0.72f,
     0.92f,
     0.112f};

/**
 * @brief 从原点出发取矛头
 * @param 要取第几个头 0~5
 */
void FetchSpear_FromOrigin(uint8_t spear_id)
{
  APP::jaw.PitchDown();
  APP::jaw.OpenClaw();

  if (spear_id == 2)
  {
    // 尝试开始追逐路径
    if (!APP::path_chaser.ChasePath(STT_R, Vec3(0.025f, 0.025f, 0.02f)))
    {
      // 如果路径出问题，拒绝启动，报错，停机
      APP::monit.LogError("[AutoGraph] MoveToTargPos rejected by PathChaser.");
      Seq::Wait(3600.0f);
    }
  }
  else if (spear_id == 1)
  {
    // 尝试开始追逐路径
    if (!APP::path_chaser.ChasePath(STT_R2, Vec3(0.025f, 0.025f, 0.02f)))
    {
      // 如果路径出问题，拒绝启动，报错，停机
      APP::monit.LogError("[AutoGraph] MoveToTargPos rejected by PathChaser.");
      Seq::Wait(3600.0f);
    }
  }
  else if (spear_id == 0)
  {
    // 尝试开始追逐路径
    if (!APP::path_chaser.ChasePath(STT_R1, Vec3(0.025f, 0.025f, 0.02f)))
    {
      // 如果路径出问题，拒绝启动，报错，停机
      APP::monit.LogError("[AutoGraph] MoveToTargPos rejected by PathChaser.");
      Seq::Wait(3600.0f);
    }
  }
  else if (spear_id == 3)
  {
    // 尝试开始追逐路径
    if (!APP::path_chaser.ChasePath(STT_R4, Vec3(0.025f, 0.025f, 0.02f)))
    {
      // 如果路径出问题，拒绝启动，报错，停机
      APP::monit.LogError("[AutoGraph] MoveToTargPos rejected by PathChaser.");
      Seq::Wait(3600.0f);
    }
  }
  else if (spear_id == 4)
  {
    // 尝试开始追逐路径
    if (!APP::path_chaser.ChasePath(STT_R5, Vec3(0.025f, 0.025f, 0.02f)))
    {
      // 如果路径出问题，拒绝启动，报错，停机
      APP::monit.LogError("[AutoGraph] MoveToTargPos rejected by PathChaser.");
      Seq::Wait(3600.0f);
    }
  }
  else if (spear_id == 5)
  {
    // 尝试开始追逐路径
    if (!APP::path_chaser.ChasePath(STT_R6, Vec3(0.025f, 0.025f, 0.02f)))
    {
      // 如果路径出问题，拒绝启动，报错，停机
      APP::monit.LogError("[AutoGraph] MoveToTargPos rejected by PathChaser.");
      Seq::Wait(3600.0f);
    }
  }



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
    float err_spd = err_dist * 4.8f - sick_kd * d_err_dist_filt;
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

  // 向前走，取矛头
  int walk_cnt_ms = 0;
  while (walk_cnt_ms < 500) // 0.4s
  {
    // 利用西克获取相对误差(m)
    float err_dist = spear_sick_value[spear_id] - APP::comm_center.sick_data_m[1];
    float err_spd = err_dist * 5.0f;

    Vec2 sick_fix = Vec2(err_spd, 0).Rotate(-System.position.z);
    float y_err = 5.51f - System.position.y; // 5.51 是取矛头需要到达的世界坐标系y
    Vec2 go_ahead = Vec2(0, y_err * 3.0f).Rotate(-System.position.z);

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