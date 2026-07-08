#include "Area3_Act.hpp"
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
#include <cstring>
#include "std_math.hpp"
#include "SemiAutoGr.hpp"

void Area3::FetchBlockOnTheGround()
{
  // 再抬起来一点
  APP::leg.SetFrontPos(-100);
  APP::leg.SetBackPos(-100);
  Seq::WaitUntil([]()
  {
    return (fabs(APP::leg.real_back_pos_mm) > 95 && fabs(APP::leg.real_front_pos_mm) > 95);
  });  

  // 先把达妙抬起来
  APP::get_block.dm_lift_pos = -757;  // _lift_vel = 18.0 rad/s

  // 爪子翻转到竖直，合上爪子
  APP::vac_claw.PitchDown();
  Seq::Wait(1.5f);

  // 往前走去取物块
  APP::chas.Move(Vec2(0.1, 0), 3.0);

  APP::vac_claw.CloseSnap();

  // 等爪子关上
  Seq::WaitUntil([]()
  {
    return fabs(APP::vac_claw.motor_snap.driver.measure.total_angle - APP::vac_claw.snap_pos) < 1000;
  });  

  // 车往前移动一小段吸上物块
  APP::chas.Move(Vec2(0.05, 0), 7.0);

  // 开泵
  APP::get_block.OpenPump();
  Seq::Wait(1.0f);
  APP::chas.Move(Vec2(-0.15, 0), 3.0);
  Seq::Wait(1.3f);
  // 停下
  APP::chas.Move(Vec2(0, 0), 3.0);

  // 开摩擦带
  APP::get_block._StartBelt();

  APP::get_block.Store();

  APP::vac_claw.CloseSnap_Hard();
  // 先翻转一点达妙
  APP::vac_claw.PitchOrigin();
  Seq::Wait(1.2f);
  // 再伸出一点
  APP::vac_claw.stretch_pos = 93000;

  // // 等爪子伸出
  // Seq::Wait(0.7f);
  Seq::WaitUntil([]()
  {
    return fabs(APP::vac_claw.motor_stretch.driver.measure.total_angle - APP::vac_claw.stretch_pos) < 100;
  });  

//   // 再等一小会，达妙完全翻回来.这个值必须大一点，否则不知道为什么爪子会先于达妙运动
//   Seq::Wait(2.0f);

  // 收回
  APP::vac_claw.stretch_pos = 600.0; // 防堵转，给个略大于0的值
  // 等着完全收回
  Seq::WaitUntil([]()
  {
    return fabs(APP::vac_claw.motor_stretch.driver.measure.total_angle - APP::vac_claw.stretch_pos) < 5000;
  });  


  APP::vac_claw.OpenSnap();
  APP::get_block.ClosePump();
  APP::get_block.OpenPRV();

  APP::get_block.Store();
}
