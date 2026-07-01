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

MeiLinArray::MeiLinArray() = default;
MeiLinArray& Area2::mei_lin_array = MeiLinArray::GetInstance();

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
}

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
  }
  
  // 等稳定
  Seq::Wait(0.5f);
  // APP::chas.Move(Vec2(0.2,0), 2.0f);
  // Seq::Wait(1.0f);

  // 伸出夹爪
  APP::vac_claw.StretchOut();

  Seq::Wait(0.5f);

  APP::get_block.OpenPump();

  Seq::Wait(1.5f);

  APP::vac_claw.StretchBack();

  Seq::Wait(0.5f);

  // 这里应该加负压泵的阀，快速放块

  APP::get_block.ClosePump();

  APP::vac_claw.need_to_fetch = false; // 复位，准备取下一个物块

  APP::get_block.Store();
}

const Vec2 MeiHuaForest_Pos[12] = {
  Vec2(3.8, 1.8), Vec2(3.8, 3), Vec2(3.8, 4.2),
  Vec2(5.0, 1.8), Vec2(5.0, 3), Vec2(5.0, 4.2),
  Vec2(6.2, 1.8), Vec2(6.2, 3), Vec2(6.2, 4.2),
  Vec2(7.4, 1.8), Vec2(7.4, 3), Vec2(7.4, 4.2),
};

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

  // 记录结束位置
  APP::monit.LogInfo("Climb400:EndPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
}

void MeiLinArray::Decend_200()
{
  // 记录起始位置
  APP::monit.LogInfo("Decend200:StartPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
  
  // 先缓慢向前走
  APP::chas.Move(Vec2(0.25, 0), 3.0f);

  // 等待小黄0号检测不到后，减速
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

  // 记录结束位置
  APP::monit.LogInfo("Decend200:EndPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
}

void MeiLinArray::Decend_400()
{
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
      case Area2Code::TURN_CW:       TurnRelative(PI/2);                    break;
      case Area2Code::TURN_CCW:      TurnRelative(-PI/2);                   break;
      case Area2Code::FETCH_UP200:   FetchBlock(BlockHeight::UP200);        break;
      case Area2Code::FETCH_UP400:   FetchBlock(BlockHeight::UP400);        break;
      case Area2Code::FETCH_DOWN200: FetchBlock(BlockHeight::DOWN200);      break;
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