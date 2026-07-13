#pragma once
#include "StateCore.hpp"
#include "infrared_detector.hpp"
#include "PathChaser.hpp"
#include "Chassis.hpp"
#include "Lift_Leg.hpp"
#include <cstdint>
#include "VacuumClaw.hpp"

extern const Vec2 MeiHuaForest_Pos[12];
enum class BlockHeight : uint8_t
{
  UP200,   // 取上 200 的块
  UP400,   // 取上 400 的块
  DOWN200, /// 取下 200 的块
};

// 动作码

enum Area2Code
{
  UP_200 = 0x64,
  DOWN_200 = 0x65,
  UP_400 = 0x66,
  DOWN_400 = 0x67,
  TURN_CW = 0x69,
  TURN_CCW = 0x68,
  FETCH_UP200 = 0x6A,
  FETCH_UP400 = 0x6B,
  FETCH_DOWN200 = 0x6C,
  AREA_DONE = 0x6E,
  ACK_DONE = 0x6F,
  // FETCH_LEFT_UP_200 = 0x70,
  // FETCH_LEFT_DOWN_200 = 0x71,
  // FETCH_RIGHT_UP_200 = 0x72,
  // FETCH_RIGHT_DOWN_200 = 0x73,

  ENTER_0 = 0x80,
  ENTER_1 = 0x81,
  ENTER_2 = 0x82,
};

// 用于管理 R2 二区动作的类
class MeiLinArray
{
  SINGLETON(MeiLinArray);

public:
  // 进入二区的动作码，默认是 0
  Area2Code enter_code = Area2Code::ENTER_0; // 进入二区的动作码，默认是 0

  /// @brief 是否收到工控机发来的动作序列，安全标准位
  bool host_a2act_recved = false; 

  /// @brief 判断是否有进入梅林前的动作
  bool need_extra_foreml_act = false;

  /// @brief 是否需要在梅林入口处取块，由下一次 FetchBlock 消费
  bool need_fetch_at_entry = false;

  /// @brief 取下200的块后之后如果要进行转向，则要先收回前腿
  bool is_f_origin_before_turn = false; 

  bool is_blue = false; // 区分红蓝半场，默认红场

  void Init(); // 初始化，注册回调函数

  /// @brief 手动设置动作序列（调试用），最后一条需为 AREA_DONE
  void SetAction(const uint8_t *actions, int len);
  
  void ExecuteSequence();

  void HandleClimbAct();

  /****************  R2 在二区的单个封装动作，公开，可以留给外面调试用 ******************/
  /**
   * @brief 二区行为之 上200
   * @details 内部区分需不需要取块
   */
  void Climb_Up200(bool need_chase = true);

  /**
   * @brief 二区行为之 上400
   * @details 内部区分需不需要取块
   */
  void Climb_Up400(bool need_chase = true);

  void Decend_200(bool need_chase = true);

  void Decend_400(bool need_chase = true);

  // void FetchSideBlock();

  void FetchBlock(BlockHeight height);

  void TurnRelative(float delta_rad);

  void TurnLeft();

  void TurnRight();

  /// @brief 根据当前坐标判断在哪个梅林块上，并导航到该块中心
  void ChaseCurrentGrid();
  void ChaseCurrentGrid_BLUE();

  static void Area2ActRxCallback(uint8_t func_code, const uint8_t *payload, uint8_t payload_len, void *user_ctx);
  bool _skip_approach = false; // 是否跳过接近动作
private:
  static constexpr int MAX_SEQ = 64;
  Area2Code _seq[MAX_SEQ] = {AREA_DONE}; // 动作缓存

  int _seq_len = 0;            // 已缓存的动作数
  bool _seq_ready = false;     // 序列收完标志

  

  bool _is_enter_act = true; // 第一次接收0x80/0x81/0x82，进行对_seq[0]的判断，第二次进入就执行判断_seq[0]和_seq[1]是否相同的逻辑

};

class MeiLin_BlockPos
{
  SINGLETON(MeiLin_BlockPos);
public:
  void GetIn(int block_id);
  void Forward(); // 内部根据逻辑更新_cur_block_id
  // void Backward(); // 应该不存在后退的情况
  void TurnLeft(); 
  void TurnRight();
  float GetTargetYaw();
  void TurnLeft_BLUE();
  void TurnRight_BLUE();
  float GetTargetYaw_BLUE();
  uint8_t GetCurId(); // 获取当前梅林ID，供外部进行姿态校准

  /// @brief 机器人是否尚未真正踏上第一块梅林桩（仍在出口平台）
  /// @details GetIn() 后为 true，第一次 Forward() 后变为 false
  bool IsBeforeFirstEntry() const { return _pending_first_entry; }

private:
  bool IR_Read(InfraredDetector& ir);
  bool NIR_Read(InfraredDetector& ir);
  
  uint8_t _forward = 0;
  uint8_t _left = 0;
  bool _pending_first_entry = true;  // true=仍在出口平台未踏上梅林桩，首次 Forward() 只翻转标志不前进
  uint8_t _cur_block_id = 0; // 当前所在的梅林块编号
  // uint8_t _cur_block_id[3][4];
};

namespace Area2
{
  /// @brief 进入二区的逻辑，会在路径的中点之后开始抬腿，预期1.5s完成
  template <size_t N>
  static bool RunIntoArea2(const Path<N>& path, Vec3 tolerance, bool is_height400 = false, bool need_fetch_at_entry = false)
  {
      if (!APP::path_chaser.ChasePath(path, tolerance))
      {
          APP::monit.LogError("RunIntoArea2 rejected by PathChaser.");
          return false;
      }
      uint32_t log_tick = 0;

      bool leg_lifted = false;
      bool vac_claw_extended = false;

      if (need_fetch_at_entry)
      {
          // 路径执行前，就先开爪
          APP::vac_claw.OpenSnap();
          // 放下爪子
          APP::vac_claw.PitchDown();
      }

      while (!APP::path_chaser.IsFinished())
      {
          Vec3 speed_vec = APP::path_chaser.GetCmdBody();
          // Vec3 world_vec = APP::path_chaser.GetCmdWorld();
          APP::chas.Move(speed_vec, 0.01f);

          if (need_fetch_at_entry)
          {
            // 路径开始执行35%后，开始伸出爪子
            if (!vac_claw_extended && APP::path_chaser.GetProgress() >= 0.35f)
            {
              vac_claw_extended = true;
              // 伸出爪子。预期1s完成
              APP::vac_claw.StretchSlow(VacuumClaw::StretchPos::OUT_FIRST, 1.0f);
            }
          }

          // 路径过半后开始缓慢抬腿，预期1.5s完成
          if (!leg_lifted && APP::path_chaser.GetProgress() >= 0.5f)
          {
            leg_lifted = true;
            int16_t lift_target = is_height400 ? -407 : -207;
            APP::leg.SetFrontPosSlow(lift_target, 1.25f);
            APP::leg.SetBackPosSlow(lift_target, 1.25f);
          }

          if (++log_tick >= 20)
          {
            log_tick = 0;
            // APP::monit.LogInfo("world_vec(%.3f, %.3f, %.3f)",
            //                    world_vec.x, world_vec.y, world_vec.z);
          }
          Seq::Wait(0.005f); // 200hz更新频率
      }

      APP::monit.LogOK("RunIntoArea2 completed.");
      return true;
  }
  
  extern MeiLinArray &mei_lin_array;
  extern MeiLin_BlockPos &block_pos;
}