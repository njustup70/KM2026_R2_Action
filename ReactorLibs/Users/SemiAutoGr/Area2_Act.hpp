#pragma once
#include "StateCore.hpp"
#include "PathChaser.hpp"
#include "Chassis.hpp"
#include <cstdint>

extern const Vec2 MeiHuaForest_Pos[12];
enum class BlockHeight : uint8_t
{
  UP200,   // 取上 200 的块
  UP400,   // 取上 400 的块
  DOWN200, /// 取下 200 的块
};

// 动作码
namespace Area2Code
{
enum : uint8_t
{
  UP_200 = 0x64,
  DOWN_200 = 0x65,
  UP_400 = 0x66,
  DOWN_400 = 0x67,
  TURN_CW = 0x68,
  TURN_CCW = 0x69,
  FETCH_UP200 = 0x6A,
  FETCH_UP400 = 0x6B,
  FETCH_DOWN200 = 0x6C,
  AREA_DONE = 0x6E,
  ACK_DONE = 0x6F,
};
}

// 用于管理 R2 二区动作的类
class MeiLinArray
{
  SINGLETON(MeiLinArray);

public:
  void Init(); // 初始化，注册回调函数

  /// @brief 手动设置动作序列（调试用），最后一条需为 AREA_DONE
  void SetAction(const uint8_t *actions, int len);
  
  void ExecuteSequence();

  /****************  R2 在二区的单个封装动作，公开，可以留给外面调试用 ******************/
  /**
   * @brief 二区行为之 上200
   * @details 内部区分需不需要取块
   */
  void Climb_Up200();

  /**
   * @brief 二区行为之 上400
   * @details 内部区分需不需要取块
   */
  void Climb_Up400();

  void Decend_200();

  void Decend_400();

  void FetchBlock(BlockHeight height);

  void TurnRelative(float delta_rad);

  static void Area2ActRxCallback(uint8_t func_code, const uint8_t *payload, uint8_t payload_len, void *user_ctx);

private:
  static constexpr int MAX_SEQ = 64;
  uint8_t _seq[MAX_SEQ] = {0}; // 动作缓存
  int _seq_len = 0;            // 已缓存的动作数
  bool _seq_ready = false;     // 序列收完标志

  bool _skip_approach = false; // 是否跳过接近动作
};

namespace Area2
{
extern MeiLinArray &mei_lin_array;
}