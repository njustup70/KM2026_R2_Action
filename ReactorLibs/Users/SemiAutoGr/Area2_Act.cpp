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

MeiLin_BlockPos::MeiLin_BlockPos() = default;
MeiLin_BlockPos& Area2::block_pos = MeiLin_BlockPos::GetInstance();

MeiLinArray::MeiLinArray() = default;
MeiLinArray& Area2::mei_lin_array = MeiLinArray::GetInstance();

/************************* MeiLin_BlockPos 类 **************************/
const Vec2 MeiHuaForest_Pos[12] = {
  Vec2(3.8, 1.8), Vec2(3.8, 3), Vec2(3.8, 4.2),
  Vec2(5.0, 1.8), Vec2(5.0, 3), Vec2(5.0, 4.2),
  Vec2(6.2, 1.8), Vec2(6.2, 3), Vec2(6.2, 4.2),
  Vec2(7.4, 1.8), Vec2(7.4, 3), Vec2(7.4, 4.2),
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
    float target_yaw = APP::chas.chas_odom.pos.z + delta_rad;
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
  Area2::block_pos.TurnLeft();
}

void MeiLinArray::TurnRight()
{
  TurnRelative(-PI / 2.0f);
  Area2::block_pos.TurnRight();
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
  APP::chas.Move(Vec2(0.5, 0), 3.0f);
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
  // 登200时，先放低前后腿
  APP::leg.F_Down_400();
  APP::leg.B_Down_400();
  Seq::Wait(0.5f);

  // 往前缓慢移动，直到抵靠到前端
  APP::chas.Move(Vec2(0.5, 0), 3.0f);
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
  APP::chas.Move(Vec2(0.5, 0), 3.0f);
  Seq::Wait(0.35f);
  APP::chas.Move(Vec2(0.25, 0), 3.0f);
  Seq::Wait(0.15f);

  // 等待小黄0号检测到就停车
  Seq::WaitUntil([&]()
  {
    return !APP::leg.detector[0].Read();
  });

  APP::chas.Move(Vec2(0.0f, 0.0f), 1.0f);  
}

void MeiLinArray::FetchSideBlock()
{
  // 判断向左还是向右转

  // 判断取 上200 的 还是 下200 的
}

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

  _skip_approach = true; // 已经靠上去了，标记跳过下一次 Climb 的重复抵靠动作
  
  // // 等稳定
  // Seq::Wait(0.5f);
  // APP::chas.Move(Vec2(0.2,0), 2.0f);
  // Seq::Wait(1.0f);

  // 抬升先上去
  APP::get_block.dm_lift_pos = -750.0f;

  // 先开爪
  APP::vac_claw.OpenSnap();
  Seq::Wait(0.4f);

  // 再伸出
  APP::vac_claw.StretchOut_First();

  Seq::Wait(0.8f);

  // 关爪子定心
  APP::vac_claw.CloseSnap();

  // 等一会再小开爪
  Seq::Wait(0.8f);
  APP::vac_claw.OpenSnapMini();

  // 先开泵，然后直接伸出第二段
  APP::get_block.OpenPump();
  APP::vac_claw.StretchOut_Second();

  // 等一会，再翻转物块，收回爪子
  Seq::Wait(0.8f);
  APP::vac_claw.StretchBack();

  APP::get_block._StartBelt();

  Seq::Wait(0.2f);

  APP::vac_claw.OpenSnap();
    // 此时爪还抓着物块，关闭负压泵
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



/**
 * @brief 二区行为之——上200
 * @details 内部区分需不需要取块
 */
void MeiLinArray::Climb_Up200()
{
  // 记录起始位置
  APP::monit.LogInfo("Climb200:StartPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);

  if(!_skip_approach)
  {
    // 登200时，先放低前后腿
    APP::leg.F_Down_200();
    APP::leg.B_Down_200();
    Seq::Wait(0.5f);

    // 往前缓慢移动，直到抵靠到前端
    APP::chas.Move(Vec2(0.5, 0), 3.0f);
    Seq::Wait(0.35f);
    APP::chas.Move(Vec2(0.25, 0), 3.0f);
    Seq::Wait(0.15f);
  }
  _skip_approach = false; // 复位标志，下次正常执行上面的抵靠动作


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

  // 记录结束位置
  APP::monit.LogInfo("Climb200:EndPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
}

/**
 * @brief 二区行为之——上400
 * @details 内部区分需不需要取块
 */
void MeiLinArray::Climb_Up400()
{
  // 记录起始位置
  APP::monit.LogInfo("Climb400:StartPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
  
  // 登400时，先放低前后腿
  APP::leg.F_Down_400();
  APP::leg.B_Down_400();

  // // 如果需要取块，那么爪子也要有所行动。要先伸出来
  // if (APP::vac_claw.need_to_fetch) FetchBlock();

  // 往前缓慢移动，直到抵靠到前端
  APP::chas.Move(Vec2(0.5, 0), 3.0f);
  Seq::Wait(0.35f);
  APP::chas.Move(Vec2(0.25, 0), 3.0f);
  Seq::Wait(0.15f);

  // 等待小黄0号检测到可以抬前腿
  Seq::WaitUntil([&]()
  {
    return APP::leg.detector[0].Read();
  });
  // 再走0.3秒，慢慢走，确保前端抵住
  APP::chas.Move(Vec2(0.2, 0), 2.0f);
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
  APP::chas.Move(Vec2(0.2, 0), 3.0f);

  // 防止小黄2号误检
  Seq::Wait(0.1f);
  // 等待小黄2号检测到可以抬后腿
  Seq::WaitUntil([&]()
  {
    return APP::leg.detector[2].Read();
  });

  // 抬升后腿
  APP::leg.B_Origin();
  
  APP::chas.Move(Vec2(0.5, 0), 2.0f);

  Seq::Wait(1.25f);
  APP::chas.Move(Vec2(0, 0), 1.0f);

  // 更新当前梅林ID
  Area2::block_pos.Forward();

  // 记录结束位置
  APP::monit.LogInfo("Climb400:EndPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
}

void MeiLinArray::Decend_200()
{
  // 这里等腿完全放下才允许下200
  Seq::WaitUntil([=]()
  {
    return (APP::leg.real_front_pos_mm > -10.0f) && (APP::leg.real_back_pos_mm > -10.0f); 
  });

  // 记录起始位置
  APP::monit.LogInfo("Decend200:StartPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
  
  // 先缓慢向前走
  APP::chas.Move(Vec2(0.25, 0), 3.0f);

  // // 如果要取块，则伸出爪取块
  // // Seq::Wait(0.8f); // 可能要加延时
  // if(APP::vac_claw.need_to_fetch) FetchBlock();
  // 等待小黄0号检测不到后，放下前腿
  Seq::WaitUntil([&]()
  {
    return !APP::leg.detector[0].Read();
  });

  // 先缓慢向前走
  APP::chas.Move(Vec2(0.1, 0), 3.0f);

  // // 如果要取块，则伸出爪取块
  // // Seq::Wait(0.8f); // 可能要加延时
  // if(APP::vac_claw.need_to_fetch) FetchBlock();

  // 等待小黄1号检测不到后，放下前腿
  Seq::WaitUntil([&]()
  {
    return !APP::leg.detector[1].Read();
  });
  APP::chas.Move(Vec2(0, 0), 2.0f);
  APP::leg.F_Down_200();
  Seq::Wait(0.45f);

  // 伸出妲己
  APP::leg.relay_daji.On();

  // 继续向前走。等待小黄3号检测不到后，放下后腿
  APP::chas.Move(Vec2(0.24, 0), 3.0f);
  Seq::WaitUntil([&]()
  {
    return !APP::leg.detector[3].Read();
  });
  APP::chas.Move(Vec2(0, 0), 2.0f);
  APP::leg.B_Down_200();
  Seq::Wait(0.45f);

  // 继续向前走一点，然后放下整个腿
  APP::chas.Move(Vec2(0.4, 0), 2.0f);
  Seq::Wait(0.5f);
  APP::chas.Move(Vec2(0.0, 0), 2.0f);
  APP::leg.F_Origin();
  APP::leg.B_Origin();

  // 收回妲己
  APP::leg.relay_daji.Off();

  Area2::block_pos.Forward();

  // 记录结束位置
  APP::monit.LogInfo("Decend200:EndPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
}

void MeiLinArray::Decend_400()
{
  // 这里等腿完全放下才允许下400
  Seq::WaitUntil([=]()
  {
    return (APP::leg.real_front_pos_mm > -10.0f) && (APP::leg.real_back_pos_mm > -10.0f); 
  });

  // 记录起始位置
  APP::monit.LogInfo("Decend400:StartPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
  
  // 先缓慢向前走
  APP::chas.Move(Vec2(0.2, 0), 6.0f);
  Seq::Wait(0.25f);
  
  // 等待小黄0号检测不到后，减速
  Seq::WaitUntil([&]()
  {
    return !APP::leg.detector[0].Read();
  });

  // 先缓慢向前走
  APP::chas.Move(Vec2(0.1, 0), 3.0f);

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
  APP::chas.Move(Vec2(0.2, 0), 6.0f);
  Seq::Wait(0.5f);
  
  Seq::WaitUntil([&]()
  {
    return !APP::leg.detector[3].Read();
  });

  APP::chas.Move(Vec2(0, 0), 2.0f);
  APP::leg.B_Down_400();
  Seq::Wait(1.25f);

  // 继续向前走一点，然后放下整个腿
  APP::chas.Move(Vec2(0.4, 0), 2.0f);
  Seq::Wait(0.5f);
  APP::chas.Move(Vec2(0.0, 0), 2.0f);
  APP::leg.F_Origin();
  APP::leg.B_Origin();

  // 收回妲己
  APP::leg.relay_daji.Off();

  Area2::block_pos.Forward();

  // 记录结束位置
  APP::monit.LogInfo("Decend400:EndPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
}

void MeiLinArray::ExecuteSequence()
{
  APP::monit.LogInfo("[MeiLin] Waiting for sequence (0x91)...");
  while (!_seq_ready)
  {
    Seq::Wait(0.01f);
  }

  APP::monit.LogInfo("[MeiLin] Sequence received, %d actions.", _seq_len);

  for (int i = 0; i < _seq_len; i++)
  {
    uint8_t cmd = _seq[i];

    if (cmd == Area2Code::AREA_DONE)
      break;

    APP::monit.LogInfo("[MeiLin] Exec[%d/%d] = 0x%02X", i + 1, _seq_len, cmd);

    switch (cmd)
    {
      case Area2Code::UP_200:        Climb_Up200();                         break;
      case Area2Code::DOWN_200:      Decend_200();                          break;
      case Area2Code::UP_400:        Climb_Up400();                         break;
      case Area2Code::DOWN_400:      Decend_400();                          break;
      case Area2Code::TURN_CW:       TurnLeft();                    break;
      case Area2Code::TURN_CCW:      TurnRight();                   break;
      case Area2Code::FETCH_UP200:   FetchBlock(BlockHeight::UP200);        break;
      case Area2Code::FETCH_UP400:   FetchBlock(BlockHeight::UP400);        break;
      case Area2Code::FETCH_DOWN200: FetchBlock(BlockHeight::DOWN200);      break;
      case Area2Code::FETCH_LEFT_UP_200: FetchSideBlock(); break;
      case Area2Code::FETCH_RIGHT_UP_200: FetchSideBlock(); break;
      case Area2Code::FETCH_LEFT_DOWN_200: FetchSideBlock(); break;
      case Area2Code::FETCH_RIGHT_DOWN_200: FetchSideBlock(); break;
      default:
        APP::monit.LogWarning("[MeiLin] Unknown action 0x%02X, skip.", cmd);
        break;
    }
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

  self._seq_len = 0;
  self._seq_ready = false;

  int copy_len = (payload_len > MAX_SEQ) ? MAX_SEQ : payload_len;
  memcpy(self._seq, payload, copy_len);
  self._seq_len = copy_len;

  // 检查最后一条是不是 AREA_DONE，是的话直接标记就绪
  if (self._seq_len > 0 && self._seq[self._seq_len - 1] == Area2Code::AREA_DONE)
  {
    self._seq_ready = true;
  }

  // 回复 ACK 告知上位机已收到
  MOD::host_pc.SendAck(Area2Code::ACK_DONE, 0);
}