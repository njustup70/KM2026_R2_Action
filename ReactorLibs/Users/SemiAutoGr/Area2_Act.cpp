#include "Monitor.hpp"
#include "Jaw.hpp"
#include "Farcon.hpp"
#include "Chassis.hpp"
#include "Lift_Leg.hpp"
#include "HostPC.hpp"
#include "PathChaser.hpp"
#include "VacuumClaw.hpp"
#include "CommCenter.hpp"
#include "Get_Block.hpp"
#include "HostPC.hpp"
#include "Area2_Act.hpp"
#include <cstring>
#include "std_math.hpp"
#include "SemiAutoGr.hpp"
#include "MainLoop.hpp"
#include "bsp_dwt.hpp"

MeiLin_BlockPos::MeiLin_BlockPos() = default;
MeiLin_BlockPos& Area2::block_pos = MeiLin_BlockPos::GetInstance();

MeiLinArray::MeiLinArray() = default;
MeiLinArray& Area2::mei_lin_array = MeiLinArray::GetInstance();

static bool IR_Read(InfraredDetector& ir);
static bool NIR_Read(InfraredDetector& ir);

/************************* MeiLin_BlockPos 类 **************************/
const Vec2 MeiHuaForest_Pos[12] = {
  Vec2(3.8, 1.8), Vec2(3.8, 3), Vec2(3.8, 4.2),
  Vec2(5.0, 1.8), Vec2(5.0, 3), Vec2(5.0, 4.2),
  Vec2(6.2, 1.8), Vec2(6.2, 3), Vec2(6.2, 4.2),
  Vec2(7.4, 1.8), Vec2(7.4, 3), Vec2(7.4, 4.2),
};

const Vec2 MeiHuaForest_Pos_Export[3] = {
  Vec2(2.6, 1.8), Vec2(2.6,3.0), Vec2(2.6, 4.2),
};
/*********** 蓝场梅林中心坐标 ***********/
const Vec2 MeiHuaForest_Pos_BLUE[12] = {
  Vec2(3.8, 4.2), Vec2(3.8, 3), Vec2(3.8, 1.8),
  Vec2(5.0, 4.2), Vec2(5.0, 3), Vec2(5.0, 1.8),
  Vec2(6.2, 4.2), Vec2(6.2, 3), Vec2(6.2, 1.8),
  Vec2(7.4, 4.2), Vec2(7.4, 3), Vec2(7.4, 1.8),
};

const Vec2 MeiHuaForest_Pos_Export_BLUE[3] = {
  Vec2(2.6, 4.2), Vec2(2.6,3.0), Vec2(2.6, 1.8),
};

/// @brief 刚进入二区时，定下进入哪个ID
/// @param block_id，由算法组发来
void MeiLin_BlockPos::GetIn(int block_id) 
{
  if(block_id >= 0 && block_id <= 11)
  {
    _cur_block_id = block_id;
    _pending_first_entry = true;  // 标记仍在出口平台，首次 Forward 只翻转标志不前进
  }
}

uint8_t MeiLin_BlockPos::GetCurId()
{
  return _cur_block_id;
}

void MeiLin_BlockPos::Forward()
{
  if (_pending_first_entry)
  {
    // 第一次踏上梅林桩：GetIn 已设好 ID，只需翻转标志确认进入，不更新块 ID
    _pending_first_entry = false;
    return;
  }
  constexpr int COLS = 3;

  int row = _cur_block_id / COLS;
  int col = _cur_block_id % COLS;

  // _forward: 0=+Y(北/前进), 1=+X(东/右), 3=-X(西/左)
  switch (_forward)
  {
    case 0:  if (row < 3) row++; break;  // 前进到下一行
    case 1:  if (col < 2) col++; break;  // 向右列
    case 3:  if (col > 0) col--; break;  // 向左列
  }

  _cur_block_id = row * COLS + col;
}

void MeiLin_BlockPos::TurnLeft()
{
  _forward = (_forward + 1) % 4;
} 

void MeiLin_BlockPos::TurnRight()
{
  _forward = (_forward + 3) % 4;
}

float MeiLin_BlockPos::GetTargetYaw()
{
  // _forward: 0=+x(yaw=0), 1=+y(yaw=PI/2), 2=-x(yaw=PI), 3=-y(yaw=-PI/2)
  switch (_forward)
  {
    case 0:  return 0.0f;
    case 1:  return PI / 2.0f;
    case 2:  return PI;
    case 3:  return -PI / 2.0f;
    default: return 0.0f;
  }
}

/********************** 以下是蓝色半场的方法 ***********************/
// 向左向右转的_forward要改，Forward里面的行列操作也要改

void MeiLin_BlockPos::TurnLeft_BLUE()
{
  _forward = (_forward + 3) % 4;
}

void MeiLin_BlockPos::TurnRight_BLUE()
{
  _forward = (_forward + 1) % 4;
}

float MeiLin_BlockPos::GetTargetYaw_BLUE()
{
  switch (_forward)
  {
    case 0:  return 0.0f;
    case 1:  return -PI / 2.0f;
    case 2:  return PI;
    case 3:  return PI / 2.0f;
    default: return 0.0f;
  }  
}

/************************* MeiLinArray 类 **************************/
void MeiLinArray::Init()
{
  _seq_len = 0;           // 已缓存的动作数
  _seq_ready = false;     // 序列收完标志
  need_fetch_at_entry = false;
  _entry_fetch_plan_locked = false;
  MOD::host_pc.Regist(0x91, Area2ActRxCallback);
}

bool MeiLinArray::LockEntryFetchPlan()
{
  _entry_fetch_plan_locked = true;
  return need_fetch_at_entry;
}

void MeiLinArray::SetAction(const uint8_t *actions, int len)
{
  if (actions == nullptr || len <= 0) return;

  int copy_len = (len > MAX_SEQ) ? MAX_SEQ : len;
  memcpy(_seq, actions, copy_len);
  _seq_len = copy_len;
  _seq_ready = false;

  // 如果最后一条是 AREA_DONE，直接标记就绪
  if (_seq[_seq_len - 1] == Area2Code::AREA_DONE)
    _seq_ready = true;
}

/// @brief 转向指定角度（相对当前朝向）
/// @param delta_rad 正值=逆时针，负值=顺时针
void MeiLinArray::TurnRelative(float delta_rad)
{
    float target_yaw = System.position.z + delta_rad;
    APP::path_chaser.ChaseYaw(target_yaw);

    while (!APP::path_chaser.IsFinished())
    {
        APP::chas.Move(APP::path_chaser.GetCmdBody(), 0.01f);
        Seq::Wait(0.005f);
    }
}

void MeiLinArray::TurnLeft()
{
  TurnRelative(PI / 2.0f);
  

  if(is_blue)
  {
    Area2::block_pos.TurnLeft_BLUE();
    Area2::mei_lin_array.ChaseCurrentGrid_BLUE();
  } 
  else 
  {
    Area2::block_pos.TurnLeft();
    Area2::mei_lin_array.ChaseCurrentGrid();
  }
}

void MeiLinArray::TurnRight()
{
  TurnRelative(-PI / 2.0f);
  if(is_blue)
  {
    Area2::block_pos.TurnRight_BLUE();
    Area2::mei_lin_array.ChaseCurrentGrid_BLUE();
  } 
  else 
  {
    Area2::block_pos.TurnRight();
    Area2::mei_lin_array.ChaseCurrentGrid();
  }
}

/// @brief 根据当前坐标判断在哪个梅林块上，并导航到该块中心
void MeiLinArray::ChaseCurrentGrid()
{
  uint8_t id = Area2::block_pos.GetCurId();
  if (id >= 12) return;

  Vec2 center = MeiHuaForest_Pos[id];
   float target_yaw = Area2::block_pos.GetTargetYaw();

  APP::monit.LogInfo("[MeiLin] ChaseGrid: id=%d center(%.2f,%.2f)",
    id, center.x, center.y);

  APP::monit.LogInfo("[MeiLin] Current Pos: (%.2f,%.2f)", System.position.x, System.position.y);

  APP::path_chaser.ChaseGes(Vec3(center.x, center.y, target_yaw));

  while (!APP::path_chaser.IsFinished())
  {
    APP::chas.Move(APP::path_chaser.GetCmdBody(), 0.01f);
    Seq::Wait(0.005f);
  }

  APP::monit.LogInfo("[MeiLin] ChaseGrid arrived at id=%d", id);
}

void MeiLinArray::ChaseCurrentGrid_BLUE()
{
  uint8_t id = Area2::block_pos.GetCurId();
  if (id >= 12) return;

  Vec2 center = MeiHuaForest_Pos_BLUE[id];
   float target_yaw = Area2::block_pos.GetTargetYaw_BLUE();

  APP::monit.LogInfo("[MeiLin] ChaseGrid: id=%d center(%.2f,%.2f)",
    id, center.x, center.y);

  APP::monit.LogInfo("[MeiLin] Current Pos: (%.2f,%.2f)", System.position.x, System.position.y);

  APP::path_chaser.ChaseGes(Vec3(center.x, center.y, target_yaw));

  while (!APP::path_chaser.IsFinished())
  {
    APP::chas.Move(APP::path_chaser.GetCmdBody(), 0.01f);
    Seq::Wait(0.005f);
  }

  APP::monit.LogInfo("[MeiLin] ChaseGrid arrived at id=%d", id);
}

/// @return 车身系修正向量，x 恒为 0（前向由调用方手动赋值），
///         y=横向闭环修正（世界系位置误差投影到车身左侧），
///         z=偏航闭环修正（rad）
static Vec3 GetAlignErr()
{
  uint8_t id = Area2::block_pos.GetCurId();
  Vec2   tgt;
  float  tyaw;

  if (Area2::block_pos.IsBeforeFirstEntry())
  {
    // 尚未踏上梅林桩，仍处于出口平台，使用出口坐标
    uint8_t export_idx = id;  // GetIn 传入的 id 即出口位索引 (0/1/2)
    if (Area2::mei_lin_array.is_blue)
    {
      tgt  = MeiHuaForest_Pos_Export_BLUE[export_idx];
    }
    else
    {
      tgt  = MeiHuaForest_Pos_Export[export_idx];
    }
    tyaw = 0.0f;  // 出口平台统一面朝 +X
  }
  else
  {
    // 已在梅林内部，使用块中心坐标
    if (Area2::mei_lin_array.is_blue)
    {
      tgt  = MeiHuaForest_Pos_BLUE[id];
      tyaw = Area2::block_pos.GetTargetYaw_BLUE();
    }
    else
    {
      tgt  = MeiHuaForest_Pos[id];
      tyaw = Area2::block_pos.GetTargetYaw();
    }
  }

  float ex = tgt.x - System.position.x;
  float ey = tgt.y - System.position.y;
  float sz = sinf(System.position.z);
  float cz = cosf(System.position.z);

  float yaw_err = tyaw - System.position.z;
  yaw_err = atan2f(sinf(yaw_err), cosf(yaw_err));

  return Vec3(0.0f,
              -sz * ex + cz * ey,
              yaw_err);
}

/// @brief 持续前进，并在每个控制周期刷新梅林桩的横向和 yaw 修正
/// @tparam StopPred 停止条件谓词，返回 true 时退出
template <typename StopPred>
static inline void MoveWhileAlignUntil(float forward_speed,
                                       float lateral_gain,
                                       float yaw_gain,
                                       StopPred&& stop_pred,
                                       float move_duration = 0.01f,
                                       float wait_dt = 0.005f)
{
  int debug_div_cnt = 0;
  while (!stop_pred())
  {
    Vec3 adj = GetAlignErr();
    if ((debug_div_cnt++ % 20) == 0)
    {
      APP::monit.LogInfo("[MeiLin] adj=(%.3f, %.3f, %.3f)", adj.x, adj.y, adj.z);
    }
    APP::chas.Move(Vec3(forward_speed,
                        lateral_gain * adj.y,
                        yaw_gain * adj.z), move_duration);
    Seq::Wait(wait_dt);
  }
}

// 取上200的块之前，先要抬腿抵住
static void FetchUp200()
{
  // 登200时，先放低前后腿
  APP::leg.F_Down_200();
  APP::leg.B_Down_200();

  Seq::WaitUntil([](){
    return (fabs(APP::leg.real_front_pos_mm - APP::leg.front_pos) < 6  
    && fabs(APP::leg.real_back_pos_mm - APP::leg.back_pos) < 6 );
  });

  // 往前缓慢移动，边前进边锁横向+yaw
  float move_start = DWT_GetTimeline_Sec();
  MoveWhileAlignUntil(0.35f, 0.5f, 0.5f, [&]()
  {
    return DWT_GetTimeline_Sec() - move_start >= 0.35f;
  });

  move_start = DWT_GetTimeline_Sec();
  MoveWhileAlignUntil(0.25f, 0.4f, 0.4f, [&]()
  {
    return DWT_GetTimeline_Sec() - move_start >= 0.15f;
  });

  MoveWhileAlignUntil(0.25f, 0.4f, 0.4f, [&]()
  {
    return IR_Read(APP::leg.detector[0]);
  });

  // 再走0.3秒，慢慢走，确保前端抵住（物理接触，不需闭环）
  APP::chas.Move(Vec2(0.2, 0), 1.0f);
  Seq::Wait(0.3f);

  APP::chas.Move(Vec2(0.0f, 0.0f), 1.0f);
}

// 取上400的块之前，先要抬腿抵住
static void FetchUp400()
{
  // 登 400 时，先放低前后腿
  APP::leg.F_Down_400();
  APP::leg.B_Down_400();

  APP::monit.LogInfo("targpos_afterup400:%d,%d", APP::leg.front_pos, APP::leg.back_pos);

  Seq::Wait(0.5f);

  // 往前缓慢移动，边前进边锁横向+yaw
  float move_start = DWT_GetTimeline_Sec();
  MoveWhileAlignUntil(0.35f, 0.5f, 0.5f, [&]()
  {
    return DWT_GetTimeline_Sec() - move_start >= 0.35f;
  });
  move_start = DWT_GetTimeline_Sec();
  MoveWhileAlignUntil(0.25f, 0.4f, 0.4f, [&]()
  {
    return DWT_GetTimeline_Sec() - move_start >= 0.15f;
  });
  MoveWhileAlignUntil(0.25f, 0.4f, 0.4f, [&]()
  {
    return APP::leg.detector[0].Read();
  });

  // 再走0.3秒，慢慢走，确保前端抵住（物理接触，不需闭环）
  APP::chas.Move(Vec2(0.2, 0), 1.0f);
  Seq::Wait(0.3f);

  APP::chas.Move(Vec2(0.0f, 0.0f), 1.0f);
}

static void FetchDown200()
{
  // 往前缓慢移动，边前进边锁横向+yaw
  float move_start = DWT_GetTimeline_Sec();
  MoveWhileAlignUntil(0.25f, 0.5f, 0.5f, [&]()
  {
    return DWT_GetTimeline_Sec() - move_start >= 0.35f;
  });
  move_start = DWT_GetTimeline_Sec();
  MoveWhileAlignUntil(0.15f, 0.4f, 0.4f, [&]()
  {
    return DWT_GetTimeline_Sec() - move_start >= 0.15f;
  });
  MoveWhileAlignUntil(0.15f, 0.4f, 0.4f, [&]()
  {
    return !APP::leg.detector[0].Read();
  });

    // 先缓慢向前走
  APP::chas.Move(Vec2(0.05, 0), 7.0f);

  Seq::Wait(0.6f);

  // // 如果要取块，则伸出爪取块
  // // Seq::Wait(0.8f); // 可能要加延时
  // if(APP::vac_claw.need_to_fetch) FetchBlock();

  // 等待小黄1号检测不到后，减速
  // Seq::WaitUntil([&]()
  // {
  //   return NIR_Read(APP::leg.detector[1]);
  // });

  APP::chas.Move(Vec2(0, 0), 2.0f);
  // APP::leg.F_Down_200();
  Seq::Wait(0.15f);

  // APP::chas.Move(Vec2(0.0f, 0.0f), 1.0f);  
}

/// @brief 伸爪取块
/// @param 
void MeiLinArray::FetchBlock(BlockHeight height)
{
  bool is_fetch_down200 = false;
  const bool preextended = need_fetch_at_entry;
  need_fetch_at_entry = false;

  switch (height)
  {
    case BlockHeight::UP200:
      FetchUp200();
    break;

    case BlockHeight::UP400:
      FetchUp400();
    break;

    case BlockHeight::DOWN200:
      is_fetch_down200 = true;
      FetchDown200();
    break;

    default:
      APP::monit.LogError("Unamed BlockHeight");
    break;
  }

  APP::monit.LogSpec("SkipAppro.");
  _skip_approach = true; // 已经靠上去了，标记跳过下一次 Climb 的重复抵靠动作

  // 抬升先上去；后续 Store() 会从预抬升状态接管
  APP::get_block.LiftUp();

  // /**-----------------    关于小黄对准块的代码    ---------------------**/

  // // 有任何一边小黄能看到，都说明歪了
  // const float side_spd = 0.10f;
  // int align_cnt_ms = 0;
  // while (NIR_Read(APP::main_loop.mouthfetch_ir_left) || NIR_Read(APP::main_loop.mouthfetch_ir_right))
  // {
  //   APP::monit.LogInfo("can't fetch block, need to correct");
  //   bool left_seen = NIR_Read(APP::main_loop.mouthfetch_ir_left);
  //   bool right_seen = NIR_Read(APP::main_loop.mouthfetch_ir_right);
  //   Vec3 body_v = Vec3(0, 0, 0);

  //   // 保持yaw角 为0 （正前方）
  //   float yaw_err = 0.0f - System.position.z;
  //   yaw_err = atan2f(sinf(yaw_err), cosf(yaw_err));
  //   body_v.z = yaw_err * 1.2f;

  //   // 如果左边的小黄（红外）能读取到，向左平移，保持yaw角
  //   if (left_seen && !right_seen)
  //   {
  //     body_v.y = side_spd;  // 向右平移
  //   }
  //   // 如果右边的小黄能读取到，向右平移，保持yaw角
  //   else if (right_seen && !left_seen)
  //   {
  //     body_v.y = -side_spd;  // 向左平移
  //   }
  //   else
  //   {
  //     // 两边同时看到时，先只稳住姿态，避免横向来回抖
  //     body_v.y = 0.0f;
  //   }

  //   APP::chas.Move(body_v, 0.006f);
  //   Seq::Wait(0.005f);

  //   align_cnt_ms += 5;
  //   if (align_cnt_ms >= 2000)   // 最多对准时间：2s
  //   {
  //     APP::monit.LogError("[Area2] mouthfetch align timeout.");
  //     break;
  //   }
  // }

  /**----------------------------------------------------**/

  APP::chas.Move(Vec3(0, 0, 0), 0.01f);
  Seq::Wait(0.05f);

  if (preextended)
  {
    APP::monit.LogInfo("claw already extended at entry");
    Seq::WaitUntil([]()
    {
      return fabs(APP::vac_claw.motor_stretch.driver.measure.total_angle - APP::vac_claw.stretch_pos) < 150;
    });
  }
  else
  {
    // 先开爪
    APP::vac_claw.OpenSnap();
    Seq::Wait(0.7f);

    APP::monit.LogInfo("stretch out to fetch block");
    
    // 再伸出
    APP::vac_claw.StretchOut_First();

    Seq::Wait(1.5f);
  }

  // 关爪子定心
  APP::vac_claw.CloseSnap();

  // 必须等爪子完全合上
  Seq::WaitUntil([]()
  {
    return fabs(APP::vac_claw.motor_snap.driver.measure.total_angle - APP::vac_claw.snap_pos) < 1000;
  });  

  Seq::Wait(0.5f);

  // 等一会再小开爪
  // Seq::Wait(0.8f);
  // APP::vac_claw.OpenSnapMini();

  // 先开泵，然后直接伸出第二段
  APP::get_block.OpenPump();
  APP::vac_claw.StretchOut_Second();

  // 等一会，再翻转物块，收回爪子
  Seq::Wait(0.8f);
  // 后台执行翻块、收爪和入仓；FetchBlock 返回后可立即执行下一次上平台动作。
  APP::vac_claw.ReturnAndStore(is_fetch_down200);

  // APP::get_block.ClosePRV();
}

bool IR_Read(InfraredDetector& ir)
{
  static int read_cnt = 0;
  if (ir.Read())
  {
    read_cnt++;
    if (read_cnt >= 5) // 连续5次检测到，才认为是真的
    {
      read_cnt = 0;
      return true;
    }
  }
  else
  {
    read_cnt = 0;
  }
  return false;
}

bool NIR_Read(InfraredDetector& ir)
{
  static int read_cnt = 0;
  if (!ir.Read())
  {
    read_cnt++;
    if (read_cnt >= 5) // 连续5次检测到，才认为是真的
    {
      read_cnt = 0;
      return true;
    }
  }
  else
  {
    read_cnt = 0;
  }
  return false;
}


/**
 * @brief 二区行为之——上200
 * @details 内部区分需不需要取块
 */
void MeiLinArray::Climb_Up200(bool need_chase)
{
  // 记录起始位置
  APP::monit.LogInfo("Climb200:StartPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);

  if(_skip_approach)
  {
    APP::monit.LogInfo("Leg out skiped");
    _skip_approach = false;
  }
  else
  {
    // 登200时，先放低前后腿
    APP::leg.F_Down_200();
    APP::leg.B_Down_200();
    
    // 等待腿到位
    Seq::WaitUntil([]()-> bool {
      return (fabs(APP::leg.real_front_pos_mm - APP::leg.front_pos) < 10.0f) 
      && (fabs(APP::leg.real_back_pos_mm - APP::leg.back_pos) < 10.0f);
    });
  }
  
  // 往前缓慢移动，边前进边锁横向+yaw
  float move_start = DWT_GetTimeline_Sec();
  MoveWhileAlignUntil(0.5f, 0.5f, 0.5f, [&]()
  {
    return DWT_GetTimeline_Sec() - move_start >= 0.2f;
  });
  move_start = DWT_GetTimeline_Sec();
  MoveWhileAlignUntil(0.4f, 0.4f, 0.4f, [&]()
  {
    return DWT_GetTimeline_Sec() - move_start >= 0.2f;
  });
  move_start = DWT_GetTimeline_Sec();
  MoveWhileAlignUntil(0.28f, 0.3f, 0.3f, [&]()
  {
    return DWT_GetTimeline_Sec() - move_start >= 0.2f;
  });
  MoveWhileAlignUntil(0.14f, 0.2f, 0.2f, [&]()
  {
    return IR_Read(APP::leg.detector[0]);
  });

  // 再走0.3秒，慢慢走，确保前端抵住
  APP::chas.Move(Vec2(0.1, 0), 5.0f);
  Seq::Wait(0.25f);

  // 停稳，抬腿
  APP::chas.Move(Vec2(0, 0), 2.0f);
  APP::leg.F_Origin();

  // 抬腿的同时向前走（抬200mm约需要0.3秒）
  Seq::Wait(0.3f);

  /****   持续衰减的爆发移动速度加成  ****/
  APP::chas.Move(Vec2(1.0, 0), 3.0f);
  Seq::Wait(0.2f);
  APP::chas.Move(Vec2(0.8, 0), 3.0f);
  Seq::Wait(0.15f);
  APP::chas.Move(Vec2(0.6, 0), 3.0f);
  Seq::Wait(0.15f);
  APP::chas.Move(Vec2(0.4, 0), 3.0f);
  Seq::Wait(0.1f);
  APP::chas.Move(Vec2(0.2, 0), 5.0f);

  // 防止小黄2号误检
  // Seq::Wait(0.1f);

  // 等待小黄2号检测到可以抬后腿
  Seq::WaitUntil([&]()
  {
    return IR_Read(APP::leg.detector[2]);
  });
  // Seq::Wait(0.1f);

  // 抬升后腿
  APP::leg.B_Origin();

  // 确认双腿归位
  Seq::WaitUntil([]()-> bool {
      return (fabs(APP::leg.real_front_pos_mm - APP::leg.front_pos) < 10.0f) 
      && (fabs(APP::leg.real_back_pos_mm - APP::leg.back_pos) < 10.0f);
    });
  
  // 往前移动
  APP::chas.Move(Vec2(0.5, 0), 2.0f);

  Seq::Wait(0.45f);
  APP::chas.Move(Vec2(0, 0), 1.0f);

  Area2::block_pos.Forward();

  if (need_chase)
  {
    if(is_blue) Area2::mei_lin_array.ChaseCurrentGrid_BLUE();
    else Area2::mei_lin_array.ChaseCurrentGrid();
  }

  // 记录结束位置
  APP::monit.LogInfo("Climb200:EndPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
}

/**
 * @brief 二区行为之——上400
 * @details 内部区分需不需要取块
 */
void MeiLinArray::Climb_Up400(bool need_chase)
{
  // 记录起始位置
  APP::monit.LogInfo("Climb400:StartPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
  
  if(_skip_approach) 
  {
    APP::monit.LogInfo("Leg out skip");
    _skip_approach = false;
  }
  else
  {
    // 登400时，先放低前后腿
    APP::leg.F_Down_400();
    APP::leg.B_Down_400();

    Seq::WaitUntil([]()-> bool {
      return (fabs(APP::leg.real_front_pos_mm - APP::leg.front_pos) < 10.0f) 
      && (fabs(APP::leg.real_back_pos_mm - APP::leg.back_pos) < 10.0f);
    });
  }

  // 往前缓慢移动，直到抵靠到前端（手K曲线 + 横向+yaw闭环）
  float move_start = DWT_GetTimeline_Sec();
  MoveWhileAlignUntil(0.5f, 0.5f, 0.5f, [&]()
  {
    return DWT_GetTimeline_Sec() - move_start >= 0.2f;
  });
  move_start = DWT_GetTimeline_Sec();
  MoveWhileAlignUntil(0.4f, 0.4f, 0.4f, [&]()
  {
    return DWT_GetTimeline_Sec() - move_start >= 0.2f;
  });
  move_start = DWT_GetTimeline_Sec();
  MoveWhileAlignUntil(0.28f, 0.3f, 0.3f, [&]()
  {
    return DWT_GetTimeline_Sec() - move_start >= 0.2f;
  });
  MoveWhileAlignUntil(0.14f, 0.2f, 0.2f, [&]()
  {
    return IR_Read(APP::leg.detector[0]);
  });

  // 再走0.3秒，慢慢走，确保前端抵住
  APP::chas.Move(Vec2(0.1, 0), 5.0f);
  Seq::Wait(0.25f);

  // 停稳，抬腿
  APP::chas.Move(Vec2(0, 0), 2.0f);
  // 抬升前腿
  APP::leg.F_Origin();

  // 抬腿的同时往前走
  Seq::Wait(0.6f);
  
  /****   持续衰减的爆发移动速度加成  ****/
  APP::chas.Move(Vec2(1.0, 0), 3.0f);
  Seq::Wait(0.2f);
  APP::chas.Move(Vec2(0.8, 0), 3.0f);
  Seq::Wait(0.15f);
  APP::chas.Move(Vec2(0.6, 0), 3.0f);
  Seq::Wait(0.15f);
  APP::chas.Move(Vec2(0.4, 0), 3.0f);
  Seq::Wait(0.1f);
  APP::chas.Move(Vec2(0.2, 0), 5.0f);

  // 防止小黄2号误检
  // Seq::Wait(0.2f);

  // 等待小黄2号检测到可以抬后腿
  Seq::WaitUntil([&]()
  {
    return IR_Read(APP::leg.detector[2]);
  });

  // 抬升后腿
  APP::leg.B_Origin();

  // 确认双腿归位
  Seq::WaitUntil([]()-> bool {
      return (fabs(APP::leg.real_front_pos_mm - APP::leg.front_pos) < 10.0f) 
      && (fabs(APP::leg.real_back_pos_mm - APP::leg.back_pos) < 10.0f);
    });
  
  APP::chas.Move(Vec2(0.5, 0), 2.0f);

  Seq::Wait(0.45f);
  
  APP::chas.Move(Vec2(0, 0), 1.0f);

  // 更新当前梅林ID
  Area2::block_pos.Forward();

  if (need_chase)
  {
    if(is_blue) Area2::mei_lin_array.ChaseCurrentGrid_BLUE();
    else Area2::mei_lin_array.ChaseCurrentGrid();
  }

  // 记录结束位置
  APP::monit.LogInfo("Climb400:EndPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
}

void MeiLinArray::Decend_200(bool need_chase)
{
  // 这里等腿完全放下才允许下200
  Seq::WaitUntil([=]()
  {
    return (APP::leg.real_front_pos_mm > -10.0f) && (APP::leg.real_back_pos_mm > -10.0f); 
  });
  // 伸出妲己
  APP::leg.relay_daji.On();

  // 记录起始位置
  APP::monit.LogInfo("Decend200:StartPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
  
  if(_skip_approach) _skip_approach = false;
  else
  {
    APP::monit.LogInfo("Leg out skip");

    // 先缓慢向前走，边前进边锁横向+yaw
    MoveWhileAlignUntil(0.4f, 0.5f, 0.5f, [&]()
    {
      return NIR_Read(APP::leg.detector[0]);
    });
  }

  // 先缓慢向前走
  APP::chas.Move(Vec2(0.1, 0), 7.0f);

  // 等待小黄1号检测不到后，放腿
  Seq::WaitUntil([&]()
  {
    return NIR_Read(APP::leg.detector[1]);
  });

  APP::chas.Move(Vec2(0, 0), 2.0f);
  APP::leg.F_Down_200();
  Seq::Wait(0.45f);

  // 伸出妲己
  APP::leg.relay_daji.On();

  // 继续向前走。等待小黄2号检测不到后，减速
  APP::chas.Move(Vec2(0.5, 0), 7.0f);
  Seq::WaitUntil([&]()
  {
    return NIR_Read(APP::leg.detector[2]);
  });
  APP::chas.Move(Vec2(0.2, 0), 7.0f);

  // 继续向前走。等待小黄3号检测不到后，放下后腿
  Seq::WaitUntil([&]()
  {
    return NIR_Read(APP::leg.detector[3]);
  });
  APP::chas.Move(Vec2(0, 0), 2.0f);
  APP::leg.B_Down_200();
  Seq::Wait(0.45f);

  // 继续向前走一点，然后放下整个腿
  APP::chas.Move(Vec2(0.4, 0), 7.0f);
  Seq::Wait(0.5f);
  APP::chas.Move(Vec2(0.0, 0), 7.0f);
  APP::leg.F_Origin();
  APP::leg.B_Origin();

  // 收回妲己
  APP::leg.relay_daji.Off();

  Area2::block_pos.Forward();

  if (need_chase)
  {
    if(is_blue) Area2::mei_lin_array.ChaseCurrentGrid_BLUE();
    else Area2::mei_lin_array.ChaseCurrentGrid();
  }

  // 记录结束位置
  APP::monit.LogInfo("Decend200:EndPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
}

void MeiLinArray::Decend_400(bool need_chase)
{
  // 这里等腿完全放下才允许下400
  Seq::WaitUntil([=]()
  {
    return (APP::leg.real_front_pos_mm > -10.0f) && (APP::leg.real_back_pos_mm > -10.0f); 
  });
    // 伸出妲己
  APP::leg.relay_daji.On();

  // 记录起始位置
  APP::monit.LogInfo("Decend400:StartPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
  
  APP::monit.LogInfo("Leg out skip");

  // 先缓慢向前走，边前进边锁横向+yaw
  MoveWhileAlignUntil(0.25f, 0.5f, 0.5f, [&]()
  {
    return !APP::leg.detector[0].Read();
  });

  // 先缓慢向前走
  APP::chas.Move(Vec2(0.1, 0), 7.0f);

  // 等待小黄1号检测不到后，放下前腿
  Seq::WaitUntil([&]()
  {
    return !APP::leg.detector[1].Read();
  });

  APP::chas.Move(Vec2(0, 0), 2.0f);
  APP::leg.F_Down_400();
  Seq::Wait(1.25f);

  // 伸出妲己
  APP::leg.relay_daji.On();

  // 继续向前走。等待小黄3号检测不到后，放下后腿
  APP::chas.Move(Vec2(0.2, 0), 7.0f);
  Seq::Wait(0.5f);
  
  Seq::WaitUntil([&]()
  {
    return !APP::leg.detector[3].Read();
  });

  APP::chas.Move(Vec2(0, 0), 2.0f);
  APP::leg.B_Down_400();
  Seq::Wait(1.25f);

  // 继续向前走一点，然后放下整个腿
  APP::chas.Move(Vec2(0.4, 0), 7.0f);
  Seq::Wait(0.5f);
  APP::chas.Move(Vec2(0.0, 0), 2.0f);
  APP::leg.F_Origin();
  APP::leg.B_Origin();

  // 收回妲己
  APP::leg.relay_daji.Off();

  Area2::block_pos.Forward();

  if (need_chase)
  {
    if(is_blue) Area2::mei_lin_array.ChaseCurrentGrid_BLUE();
    else Area2::mei_lin_array.ChaseCurrentGrid();
  }

  // 记录结束位置
  APP::monit.LogInfo("Decend400:EndPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
}

///@brief 判断_seq[0]和_seq[1]不一样后，进行的平移动作
static void MoveBeforeClimb(Vec2 targ_mei_lin_xy)
{
  APP::path_chaser.ChaseGes(Vec3(targ_mei_lin_xy.x, targ_mei_lin_xy.y, 0));
  while (!APP::path_chaser.IsFinished())
  {
    APP::chas.Move(APP::path_chaser.GetCmdBody(), 0.01f);
    Seq::Wait(0.005f);
  }
}

///@brief 用于处理进入二区前，上位机发来目标梅林后的动作
void MeiLinArray::HandleClimbAct()
{
  // _seq：[0xFA][长度][0x91][0x80/0x81/0x82][0x80/0x81/0x82][后续动作]……[0x6E]
  // 如果_seq的前两个字节( func_code 紧接着的)不一样，说明要先取，然后退后，再去正确的梅林ID，再上去
  if(_is_enter_act)
  {
    switch(_seq[0])
    {
      case Area2Code::ENTER_0:
        Area2::mei_lin_array.FetchBlock(BlockHeight::UP400);
      break;

      case Area2Code::ENTER_1:
        Area2::mei_lin_array.FetchBlock(BlockHeight::UP200);
      break;

      case Area2Code::ENTER_2:
        Area2::mei_lin_array.FetchBlock(BlockHeight::UP400);
      break;

      default:
        APP::monit.LogError("Area2 First ActionCode: 0x%02X Error!", _seq[0]);
      break;
    }
    _is_enter_act = false;
  }
  else
  {
    if(_seq[0] != _seq[1])
    {
      // 先后退一小段
      APP::chas.Move(Vec2(-0.5,0), 0.5f);
      Seq::Wait(0.5);
      APP::chas.Move(Vec2(0,0), 0.5f);
      
      // 再位置闭环一下，到-3，-2，-1的“梅林”中心上
      // 并且，抬腿到对应高度
      switch(_seq[1])
      {
        case Area2Code::ENTER_0:
          APP::leg.F_Down_400();
          APP::leg.B_Down_400();
          if(is_blue)
          {
            MoveBeforeClimb(MeiHuaForest_Pos_Export_BLUE[0]);
          }
          else
          {
            MoveBeforeClimb(MeiHuaForest_Pos_Export[0]);
          }
          break;

        case Area2Code::ENTER_1:
          APP::leg.F_Down_200();
          APP::leg.B_Down_200();
          if(is_blue)
          {
            MoveBeforeClimb(MeiHuaForest_Pos_Export_BLUE[1]);
          }
          else
          {
            MoveBeforeClimb(MeiHuaForest_Pos_Export[1]);
          }        
          break;

        case Area2Code::ENTER_2:
          APP::leg.F_Down_400();
          APP::leg.B_Down_400();
          if(is_blue)
          {
            MoveBeforeClimb(MeiHuaForest_Pos_Export_BLUE[2]);
          }
          else
          {
            MoveBeforeClimb(MeiHuaForest_Pos_Export[2]);
          }
          break;

        default:
          APP::monit.LogError("Area2 Second ActionCode: 0x%02X Error!",_seq[1]);
        break;
      }
    }
  }
}

static void MoveBackAndDecend()
{
  Seq::Wait(0.5);

  // 先后退
  APP::chas.Move(Vec2(-0.25f, 0.0f), 7.0);
  Seq::Wait(0.5);
  // if(Area2::mei_lin_array.is_f_origin_before_turn)
  // {
  //   // 要先收起前腿
  //   APP::leg.F_Origin();
  //   Area2::mei_lin_array.is_f_origin_before_turn = false;
  //   // 等待前腿完全收起
  //   Seq::WaitUntil([=]()
  //   {
  //     return (APP::leg.real_front_pos_mm > -3.0f); 
  //   });
  //   // 再倒退
  //   APP::chas.Move(Vec2(-0.2, 0));
  // }

  // 再根据当前梅林 ID 位置闭环
  if(Area2::mei_lin_array.is_blue) Area2::mei_lin_array.ChaseCurrentGrid_BLUE();
  else Area2::mei_lin_array.ChaseCurrentGrid();

  Seq::Wait(1.0f);

  if (Area2::mei_lin_array._skip_approach) Area2::mei_lin_array._skip_approach = false;

  // 再把腿放下
  APP::leg.F_Origin();
  APP::leg.B_Origin();

  // 确保完全放下才允许下一步转向
  Seq::WaitUntil([=]()
  {
    return (APP::leg.real_front_pos_mm > -10.0f) && (APP::leg.real_back_pos_mm > -10.0f); 
  });
}

void MeiLinArray::ExecuteSequence()
{
  APP::monit.LogInfo("[MeiLin] Waiting for sequence (0x91)...");
  while (!_seq_ready)
  {
    Seq::Wait(0.01f);
  }

  switch (_seq[1])
  {
    case 0x80:
      Area2::block_pos.GetIn(0);
    break;

    case 0x81:
      Area2::block_pos.GetIn(1);
    break;

    case 0x82:
      Area2::block_pos.GetIn(2);
    break;

    default:
      APP::monit.LogError("Error EnterCode: 0x%x02", _seq[1]);
    break;
  }
  

  APP::monit.LogInfo("[MeiLin] Sequence received, %d actions.", _seq_len);

  for (int i = need_extra_foreml_act ? 0 : 2; i < _seq_len; i++)
  {
    /***    厕所    ***/
    if (need_extra_foreml_act && i == 2) continue;

    bool need_chase = true;
    if (i == _seq_len - 2) need_chase = false;

    uint8_t cmd = _seq[i];

    if (cmd == Area2Code::AREA_DONE)
      break;

    APP::monit.LogInfo("[MeiLin] Exec[%d/%d] = 0x%02X", i + 1, _seq_len, cmd);

    switch (cmd)
    {
      case Area2Code::UP_200:        Climb_Up200(need_chase);                         break;
      case Area2Code::DOWN_200:      Decend_200(need_chase);                          break;
      case Area2Code::UP_400:        Climb_Up400(need_chase);                         break;
      case Area2Code::DOWN_400:      Decend_400(need_chase);                          break;
      case Area2Code::TURN_CW:       TurnRight();                    break;
      case Area2Code::TURN_CCW:      TurnLeft();                   break;
      case Area2Code::FETCH_UP200:
        FetchBlock(BlockHeight::UP200); 
        if (i + 1 < _seq_len && (_seq[i+1] == Area2Code::TURN_CW || _seq[i+1] == Area2Code::TURN_CCW))
          MoveBackAndDecend();       
        break;
      case Area2Code::FETCH_UP400:
        FetchBlock(BlockHeight::UP400); 
        if (i + 1 < _seq_len && (_seq[i+1] == Area2Code::TURN_CW || _seq[i+1] == Area2Code::TURN_CCW))
          MoveBackAndDecend();       
        break;
      case Area2Code::FETCH_DOWN200: 
        FetchBlock(BlockHeight::DOWN200);
        is_f_origin_before_turn = true;
        if (i + 1 < _seq_len && (_seq[i+1] == Area2Code::TURN_CW || _seq[i+1] == Area2Code::TURN_CCW))
          MoveBackAndDecend();      
        break;
      case Area2Code::ENTER_0: 
      case Area2Code::ENTER_1: 
      case Area2Code::ENTER_2: HandleClimbAct(); break;
      default:
        APP::monit.LogWarning("[MeiLin] Unknown action 0x%02X, skip.", cmd);
        break;
    }

    // Seq::Wait(1.0f);
    
    // while(MOD::farcon.button_second_half[0] != 1)
    // {
    //   Response_Button_main();
    //   Seq::Wait(0.01f);
    // }
  }

  _seq_len = 0;
  _seq_ready = false;
}

// 接收工控机发来的动作序列
void MeiLinArray::Area2ActRxCallback(uint8_t func_code, const uint8_t *payload, uint8_t payload_len, void *user_ctx)
{
  if(func_code != 0x91 || payload == nullptr || payload_len == 0)
  {
    return;
  }

  MeiLinArray &self = MeiLinArray::GetInstance();

  self._seq_ready = false;

  int copy_len = (payload_len > MAX_SEQ) ? MAX_SEQ : payload_len;
  memcpy(self._seq, payload, copy_len);
  self._seq_len = copy_len;

  self.enter_code = self._seq[0];

  // 检查最后一条是不是 AREA_DONE，是的话直接标记就绪
  if (self._seq_len > 0 && self._seq[self._seq_len - 1] == Area2Code::AREA_DONE)
  {
    self._seq_ready = true;
  }

  // 回复 ACK 告知上位机已收到
  MOD::host_pc.SendAck(Area2Code::ACK_DONE, 0);

  self.need_extra_foreml_act = self._seq_len >= 2 && self._seq[0] != self._seq[1];
  if (!self._entry_fetch_plan_locked)
  {
    self.need_fetch_at_entry = self.need_extra_foreml_act ||
                               (self._seq_len > 2 &&
                                (self._seq[2] == Area2Code::FETCH_UP200 ||
                                 self._seq[2] == Area2Code::FETCH_UP400 ||
                                 self._seq[2] == Area2Code::FETCH_DOWN200));
  }

  self.host_a2act_recved = true;
}
