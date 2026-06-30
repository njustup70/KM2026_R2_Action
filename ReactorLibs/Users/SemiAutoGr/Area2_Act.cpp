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

#include "Area2_Act.hpp"

/// @brief 伸爪取块
/// @param 取多少高度的块
static void FetchBlock()
{
  // 等 1s 稳定
  Seq::Wait(1.0f);

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
void Climb_Up200()
{
  // 记录起始位置
  APP::monit.LogInfo("Climb200:StartPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);

  // 登200时，先放低前后腿
  APP::leg.F_Down_200();
  APP::leg.B_Down_200();
  Seq::Wait(0.5f);

  // 如果需要取块，那么爪子也要有所行动。要先伸出来
  if (APP::vac_claw.need_to_fetch) FetchBlock();

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
void Climb_Up400()
{
  // 记录起始位置
  APP::monit.LogInfo("Climb400:StartPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
  
  // 登400时，先放低前后腿
  APP::leg.F_Down_400();
  APP::leg.B_Down_400();

  // 如果需要取块，那么爪子也要有所行动。要先伸出来
  if (APP::vac_claw.need_to_fetch) FetchBlock();

  // 往前缓慢移动，直到抵靠到前端
  APP::chas.Move(Vec2(0.3, 0), 2.0f);

  Seq::Wait(1.0f);

  // 等待小黄0号检测到可以抬前腿
  Seq::WaitUntil([&]()
  {
    return APP::leg.detector[0].Read();
  });

  // 抬升前腿
  APP::leg.F_Origin();
  
  // 往前移动
  APP::chas.Move(Vec2(0.5, 0), 2.0f);

  // 等待小黄2号检测到可以抬后腿
  Seq::WaitUntil([&]()
  {
    return APP::leg.detector[2].Read();
  });

  // 抬升后腿
  APP::leg.B_Origin();
  
  // 往前移动一点
  APP::chas.Move(Vec2(0.5, 0), 2.0f);
  Seq::Wait(1.0f);
  
  // 结束
  APP::chas.Move(Vec2(0.0f, 0.0f), 2.0f);

  // 记录结束位置
  APP::monit.LogInfo("Climb400:EndPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
}

void Decend_200()
{
  // 记录起始位置
  APP::monit.LogInfo("Decend200:StartPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
  
  // 先缓慢向前走
  APP::chas.Move(Vec2(0.24, 0), 3.0f);

  // 如果要取块，则伸出爪取块
  // Seq::Wait(0.8f); // 可能要加延时
  if(APP::vac_claw.need_to_fetch) FetchBlock();

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

void Decend_400()
{
  // 记录起始位置
  APP::monit.LogInfo("Decend400:StartPos:(%.2f,%.2f)",
     APP::comm_center.slam_pos.x, APP::comm_center.slam_pos.y);
  
  // 先缓慢向前走
  APP::chas.Move(Vec2(0.2, 0), 6.0f);
  Seq::Wait(0.5f);

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