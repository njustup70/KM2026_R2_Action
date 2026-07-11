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
    _first_move = true;  // 标记下一次 Forward 是"进入"而非"前进"
  }
}

uint8_t MeiLin_BlockPos::GetCurId()
{
  return _cur_block_id;
}

void MeiLin_BlockPos::Forward()
{
  if (_first_move)
  {
    // 第一次：刚进入梅林，已经由 GetIn 设好了 ID，不前进
    _first_move = false;
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
  MOD::host_pc.Regist(0x91, Area2ActRxCallback);
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

// 取上200的块之前，先要抬腿抵住
static void FetchUp200()
{
  // 登200时，先放低前后腿
  APP::leg.F_Down_200();
  APP::leg.B_Down_200();
  Seq::Wait(0.5f);

  // 往前缓慢移动，直到抵靠到前端
  APP::chas.Move(Vec2(0.35, 0), 3.0f);
  Seq::Wait(0.35f);
  APP::chas.Move(Vec2(0.25, 0), 3.0f);
  Seq::Wait(0.15f);

  // 等待小黄0号检测到可以抬前腿
  Seq::WaitUntil([&]()
  {
    return APP::leg.detector[0].Read();
  });
  // 再走0.3秒，慢慢走，确保前端抵住
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

  // 往前缓慢移动，直到抵靠到前端
  APP::chas.Move(Vec2(0.35, 0), 3.0f);
  Seq::Wait(0.35f);
  APP::chas.Move(Vec2(0.25, 0), 3.0f);
  Seq::Wait(0.15f);

  // 等待小黄0号检测到可以抬前腿
  Seq::WaitUntil([&]()
  {
    return APP::leg.detector[0].Read();
  });

  // 再走0.3秒，慢慢走，确保前端抵住
  APP::chas.Move(Vec2(0.2, 0), 1.0f);
  Seq::Wait(0.3f);

  APP::chas.Move(Vec2(0.0f, 0.0f), 1.0f);  
}

static void FetchDown200()
{
  // 往前缓慢移动，直到抵靠到前端
  APP::chas.Move(Vec2(0.25, 0), 7.0f);
  Seq::Wait(0.35f);
  APP::chas.Move(Vec2(0.15, 0), 7.0f);
  Seq::Wait(0.15f);

  // 等待小黄0号检测到就减速
  Seq::WaitUntil([&]()
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

// 取左边上200的块
// void MeiLinArray::FetchLeftSide_UP200()
// {
//   // 判断向左还是向右转

//   // 判断取 上200 的 还是 下200 的

//   Area2::mei_lin_array.TurnLeft();

//   // 等0.5s转过来
//   Seq::Wait(0.5);

//   Area2::mei_lin_array.FetchBlock(BlockHeight::UP200);

//   // 等一小会儿稳定一些，这个可以调短，因为FetchBlock最后调用Store的时候爪子已经伸回来了
//   Seq::Wait(0.5);

//   // 退回当前梅林的中心 
//   Area2::mei_lin_array.ChaseCurrentGrid();

//   // 放下腿
//   APP::leg.F_Origin();
//   APP::leg.B_Origin();

//   // 等腿完全放下来才允许下一步动作
//   Seq::WaitUntil([=]()
//   {
//     return (APP::leg.real_front_pos_mm > -10.0f) && (APP::leg.real_back_pos_mm > -10.0f); 
//   });
// }

// /// @brief 伸爪取块
// /// @param 
// void MeiLinArray::FetchBlock(BlockHeight height)
// {
//   switch (height)
//   {
//     case BlockHeight::UP200:
//       FetchUp200();
//     break;

//     case BlockHeight::UP400:
//       FetchUp400();
//     break;

//     case BlockHeight::DOWN200:
//       FetchDown200();
//     break;

//     default:
//       APP::monit.LogError("Unamed BlockHeight");
//     break;
//   }

//   _skip_approach = true; // 已经靠上去了，标记跳过下一次 Climb 的重复抵靠动作
  
//   // // 等稳定
//   // Seq::Wait(0.5f);
//   // APP::chas.Move(Vec2(0.2,0), 2.0f);
//   // Seq::Wait(1.0f);

//   // 抬升先上去
//   APP::get_block.dm_lift_pos = -750.0f;

//   // 先开爪
//   APP::vac_claw.OpenSnap();
//   Seq::Wait(0.4f);

//   // 再伸出
//   APP::vac_claw.StretchOut();

//   // 等待伸出完成后，闭上爪
//   Seq::Wait(1.2f);
//   APP::vac_claw.CloseSnap();

//   // 微微张开爪子
//   APP::vac_claw.OpenSnapMini();
//   // 往前慢慢走一些
//   APP::chas.Move(Vec2(0.1, 0), 0.25f);
//   Seq::Wait(0.25f);

//   // 打开负压泵
//   APP::get_block.OpenPump();

//   // // 等爪子抓住后
//   // Seq::Wait(2.5f);

//   // 等待吸住后，翻转物块，收爪
//   Seq::Wait(1.0f);
//   APP::vac_claw.StretchBack();

//   // 此时爪还抓着物块，关闭负压泵
//   APP::get_block.ClosePump();

//   APP::get_block._StartBelt();

//   APP::vac_claw.OpenSnap();

//   APP::get_block.OpenPRV();

//   APP::get_block.Store();

//   APP::get_block.ClosePRV();
// }

/// @brief 伸爪取块
/// @param 
void MeiLinArray::FetchBlock(BlockHeight height)
{
  switch (height)
  {
    case BlockHeight::UP200:
      FetchUp200();
    break;

    case BlockHeight::UP400:
      FetchUp400();
    break;

    case BlockHeight::DOWN200:
      FetchDown200();
    break;

    default:
      APP::monit.LogError("Unamed BlockHeight");
    break;
  }

  APP::monit.LogSpec("SkipAppro.");
  _skip_approach = true; // 已经靠上去了，标记跳过下一次 Climb 的重复抵靠动作
  
  // // 等稳定
  // Seq::Wait(0.5f);
  // APP::chas.Move(Vec2(0.2,0), 2.0f);
  // Seq::Wait(1.0f);

  // 抬升先上去；后续 Store() 会从预抬升状态接管
  APP::get_block.LiftUp();

  /**-----------------    关于小黄对准块的代码    ---------------------**/

  // 有任何一边小黄能看到，都说明歪了
  const float side_spd = 0.10f;
  int align_cnt_ms = 0;
  while (IR_Read(APP::main_loop.mouthfetch_ir_left) || IR_Read(APP::main_loop.mouthfetch_ir_right))
  {
    bool left_seen = IR_Read(APP::main_loop.mouthfetch_ir_left);
    bool right_seen = IR_Read(APP::main_loop.mouthfetch_ir_right);

    Vec3 body_v = Vec3(0, 0, 0);

    // 保持yaw角 为0 （正前方）
    float yaw_err = 0.0f - System.position.z;
    yaw_err = atan2f(sinf(yaw_err), cosf(yaw_err));
    body_v.z = yaw_err * 1.2f;

    // 如果左边的小黄（红外）能读取到，向右平移，保持yaw角
    if (left_seen && !right_seen)
    {
      body_v.y = -side_spd;  // 向右平移
    }
    // 如果右边的小黄能读取到，向左平移，保持yaw角
    else if (right_seen && !left_seen)
    {
      body_v.y = side_spd;  // 向左平移
    }
    else
    {
      // 两边同时看到时，先只稳住姿态，避免横向来回抖
      body_v.y = 0.0f;
    }

    APP::chas.Move(body_v, 0.006f);
    Seq::Wait(0.005f);

    align_cnt_ms += 5;
    if (align_cnt_ms >= 2000)   // 最多对准时间：2s
    {
      APP::monit.LogError("[Area2] mouthfetch align timeout.");
      break;
    }
  }

  /**----------------------------------------------------**/

  APP::chas.Move(Vec3(0, 0, 0), 0.01f);
  Seq::Wait(0.05f);

  // 先开爪
  APP::vac_claw.OpenSnap();
  Seq::Wait(0.5f);

  // 再伸出
  APP::vac_claw.StretchOut_First();

  Seq::Wait(1.5f);

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
  APP::vac_claw.StretchBack();

  APP::get_block._StartBelt();
  
  // 此时爪还抓着物块，但是取下200块的时候，泵不开物块会掉下来，所以等爪子收回摩擦带能碰着物块再开夹爪、关泵
  Seq::WaitUntil([]()
  {
    return fabs(APP::vac_claw.motor_stretch.driver.measure.total_angle - APP::vac_claw.stretch_pos) < 150;
  });

  APP::vac_claw.OpenSnap();
  APP::get_block.ClosePump();
  APP::get_block.OpenPRV();

  APP::get_block.Store();

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

  if(_skip_approach) _skip_approach = false;
  else
  {
    APP::monit.LogInfo("Leg out skip");

    // 登200时，先放低前后腿
    APP::leg.F_Down_200();
    APP::leg.B_Down_200();
    Seq::Wait(0.5f);
  }
  
    // 往前缓慢移动，直到抵靠到前端
    APP::chas.Move(Vec2(0.35, 0), 3.0f);
    Seq::Wait(0.35f);
    APP::chas.Move(Vec2(0.15, 0), 3.0f);
    Seq::Wait(0.15f);

  // _skip_approach = false; // 复位标志，下次正常执行上面的抵靠动作


  // 等待小黄0号检测到可以抬前腿
  Seq::WaitUntil([&]()
  {
    return APP::leg.detector[0].Read();
  });
  // 再走0.3秒，慢慢走，确保前端抵住
  APP::chas.Move(Vec2(0.2, 0), 1.0f);
  Seq::Wait(0.3f);

  // // 如果需要取块，那么爪子也要有所行动。要先伸出来
  // if (APP::vac_claw.need_to_fetch)
  // {
  //   // 此时小黄+移动已经确保前端抵住了
  //   FetchBlock();
  // } 

  // 停稳，抬腿
  APP::chas.Move(Vec2(0, 0), 2.0f);
  APP::leg.F_Origin();
  // 抬腿的同时向前走
  Seq::Wait(0.25f);

  /****   持续衰减的爆发移动速度加成  ****/
  APP::chas.Move(Vec2(1.0, 0), 3.0f);
  Seq::Wait(0.2f);
  APP::chas.Move(Vec2(0.5, 0), 3.0f);
  Seq::Wait(0.1f);
  APP::chas.Move(Vec2(0.2, 0), 3.0f);

  // 防止小黄2号误检
  Seq::Wait(0.1f);
  // 等待小黄2号检测到可以抬后腿
  Seq::WaitUntil([&]()
  {
    return APP::leg.detector[2].Read();
  });
  Seq::Wait(0.1f);

  // 抬升后腿并往前移动
  APP::leg.B_Origin();
  APP::chas.Move(Vec2(0.5, 0), 2.0f);

  Seq::Wait(1.25f);
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
      return (APP::leg.real_front_pos_mm < -300.0f) && (APP::leg.real_back_pos_mm < -300.0f);
    });
  }

    // 往前缓慢移动，直到抵靠到前端
    APP::chas.Move(Vec2(0.35, 0), 3.0f);
    Seq::Wait(0.35f);
    APP::chas.Move(Vec2(0.15, 0), 3.0f);
    Seq::Wait(0.15f);

  // 等待小黄0号检测到可以抬前腿
  Seq::WaitUntil([&]()
  {
    return IR_Read(APP::leg.detector[0]);
  });

  // 再走0.3秒，慢慢走，确保前端抵住
  APP::chas.Move(Vec2(0.2, 0), 5.0f);
  Seq::Wait(0.3f);

  // // 如果需要取块，那么爪子也要有所行动。要先伸出来
  // if (APP::vac_claw.need_to_fetch)
  // {
  //   // 此时小黄+移动已经确保前端抵住了
  //   FetchBlock();
  // } 

  // 停稳，抬腿
  APP::chas.Move(Vec2(0, 0), 2.0f);
  // 抬升前腿
  APP::leg.F_Origin();
  // 抬腿的同时往前走
  Seq::Wait(0.25f);
  
  /****   持续衰减的爆发移动速度加成  ****/
  APP::chas.Move(Vec2(1.0, 0), 3.0f);
  Seq::Wait(0.2f);
  APP::chas.Move(Vec2(0.5, 0), 3.0f);
  Seq::Wait(0.1f);
  APP::chas.Move(Vec2(0.2, 0), 5.0f);

  // 防止小黄2号误检
  Seq::Wait(0.2f);

  // 等待小黄2号检测到可以抬后腿
  Seq::WaitUntil([&]()
  {
    return IR_Read(APP::leg.detector[2]);
  });

  // 抬升后腿
  APP::leg.B_Origin();
  
  APP::chas.Move(Vec2(0.5, 0), 2.0f);

  Seq::Wait(1.25f);
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

    // 先缓慢向前走
    APP::chas.Move(Vec2(0.25, 0), 7.0f);

    // // 如果要取块，则伸出爪取块
    // // Seq::Wait(0.8f); // 可能要加延时
    // if(APP::vac_claw.need_to_fetch) FetchBlock();

    // 等待小黄0号检测不到后，放下前腿
    Seq::WaitUntil([&]()
    {
      return NIR_Read(APP::leg.detector[0]);
    });
  }

  // 先缓慢向前走
  APP::chas.Move(Vec2(0.1, 0), 7.0f);

  // // 如果要取块，则伸出爪取块
  // // Seq::Wait(0.8f); // 可能要加延时
  // if(APP::vac_claw.need_to_fetch) FetchBlock();

  // 等待小黄1号检测不到后，减速
  Seq::WaitUntil([&]()
  {
    return NIR_Read(APP::leg.detector[1]);
  });

  APP::chas.Move(Vec2(0, 0), 2.0f);
  APP::leg.F_Down_200();
  Seq::Wait(0.45f);

  // 伸出妲己
  APP::leg.relay_daji.On();

  // 继续向前走。等待小黄3号检测不到后，放下后腿
  APP::chas.Move(Vec2(0.24, 0), 7.0f);
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
  // 先缓慢向前走
  APP::chas.Move(Vec2(0.25, 0), 6.0f);
  Seq::Wait(0.25f);
  
  // 等待小黄0号检测不到后，减速
  Seq::WaitUntil([&]()
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

    Seq::Wait(1.0f);
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

  if (self._seq[0] != self._seq[1]) self.need_extra_foreml_act = true;

  self.host_a2act_recved = true;
}
