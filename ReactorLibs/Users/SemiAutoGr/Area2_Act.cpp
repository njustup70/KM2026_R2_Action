#include "Monitor.hpp"
#include "Jaw.hpp"
#include "Farcon.hpp"
#include "Chassis.hpp"
#include "Lift_Leg.hpp"
#include "HostPC.hpp"
#include "PathChaser.hpp"
#include "VacuumClaw.hpp"

#include "Area2_Act.hpp"

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
  // 登200时，先放低前后腿
  APP::leg.F_Down_200();
  APP::leg.B_Down_200();

  // 如果需要取块，那么爪子也要有所行动。要先伸出来
  // if (need_to_fetch) APP::vac_claw.StretchOut();

  // 往前缓慢移动，直到抵靠到前端
  APP::chas.Move(Vec2(0.3, 0), 2.0f);

  Seq::Wait(1.0f);

  // 等待小黄0号检测到可以抬前腿
  Seq::WaitUntil([&]()
  {
    return APP::leg.detector[0].Read();
  });

  APP::chas.Move(Vec2(0, 0), 2.0f);

  Seq::Wait(0.5f);

  // 抬升前腿
  APP::leg.F_Origin();

  Seq::Wait(0.5f);
  
  // 往前移动
  APP::chas.Move(Vec2(0.5, 0), 2.0f);
  
  Seq::Wait(0.5f);

  // 等待小黄2号检测到可以抬后腿
  Seq::WaitUntil([&]()
  {
    return APP::leg.detector[2].Read();
  });

  APP::chas.Move(Vec2(0, 0), 2.0f);
  
  Seq::Wait(0.5f);

  // 抬升后腿
  APP::leg.B_Origin();

  // 往前移动
  APP::chas.Move(Vec2(0.5, 0), 2.0f);

  Seq::Wait(0.5f);

  APP::chas.Move(Vec2(0, 0), 2.0f);
}

/**
 * @brief 二区行为之——上400
 * @details 内部区分需不需要取块
 */
void Climb_Up400()
{
  // 登400时，先放低前后腿
  APP::leg.F_Down_400();
  APP::leg.B_Down_400();

  // 如果需要取块，那么爪子也要有所行动。要先伸出来
  // if (need_to_fetch) APP::vac_claw.StretchOut();

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
}

void Decend_200()
{
  // 先缓慢向前走
  APP::chas.Move(Vec2(0.3, 0), 2.0f);

  // 等待小黄1号检测不到后，放下前腿
  Seq::WaitUntil([&]()
  {
    return !APP::leg.detector[1].Read();
  });
  APP::chas.Move(Vec2(0, 0), 2.0f);
  APP::leg.F_Down_200();
  Seq::Wait(0.245f);

  // 继续向前走。等待小黄3号检测不到后，放下后腿
  APP::chas.Move(Vec2(0.3, 0), 2.0f);
  Seq::WaitUntil([&]()
  {
    return !APP::leg.detector[3].Read();
  });
  APP::chas.Move(Vec2(0, 0), 2.0f);
  APP::leg.B_Down_200();
  Seq::Wait(0.245f);

  // 继续向前走一点，然后放下整个腿
  APP::chas.Move(Vec2(0.4, 0), 2.0f);
  Seq::Wait(0.5f);
  APP::chas.Move(Vec2(0.0, 0), 2.0f);
  APP::leg.F_Origin();
  APP::leg.B_Origin();
}

void Decend_400()
{
  // 先缓慢向前走
  APP::chas.Move(Vec2(0.15, 0), 2.0f);

  // 等待小黄1号检测不到后，放下前腿
  Seq::WaitUntil([&]()
  {
    return !APP::leg.detector[1].Read();
  });
  APP::chas.Move(Vec2(0, 0), 2.0f);
  APP::leg.F_Down_400();
  Seq::Wait(0.45f);

  // 继续向前走。等待小黄3号检测不到后，放下后腿
  APP::chas.Move(Vec2(0.15, 0), 2.0f);
  Seq::WaitUntil([&]()
  {
    return !APP::leg.detector[3].Read();
  });
  APP::chas.Move(Vec2(0, 0), 2.0f);
  APP::leg.B_Down_400();
  Seq::Wait(0.45f);

  // 继续向前走一点，然后放下整个腿
  APP::chas.Move(Vec2(0.4, 0), 2.0f);
  Seq::Wait(0.5f);
  APP::chas.Move(Vec2(0.0, 0), 2.0f);
  APP::leg.F_Origin();
  APP::leg.B_Origin();
}