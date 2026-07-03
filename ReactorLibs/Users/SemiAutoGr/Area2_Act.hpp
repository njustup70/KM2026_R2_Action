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

enum Area2Code
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
  FETCH_LEFT_UP_200 = 0x70,
  FETCH_LEFT_DOWN_200 = 0x71,
  FETCH_RIGHT_UP_200 = 0x72,
  FETCH_RIGHT_DOWN_200 = 0x73,

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
  void Climb_Up200();

  /**
   * @brief 二区行为之 上400
   * @details 内部区分需不需要取块
   */
  void Climb_Up400();

  void Decend_200();

  void Decend_400();

  void FetchSideBlock();

  void FetchBlock(BlockHeight height);

  void TurnRelative(float delta_rad);

  void TurnLeft();

  void TurnRight();

  /// @brief 根据当前坐标判断在哪个梅林块上，并导航到该块中心
  void ChaseCurrentGrid();

  static void Area2ActRxCallback(uint8_t func_code, const uint8_t *payload, uint8_t payload_len, void *user_ctx);

private:
  static constexpr int MAX_SEQ = 64;
  Area2Code _seq[MAX_SEQ] = {Area2Code::AREA_DONE}; // 动作缓存

  int _seq_len = 0;            // 已缓存的动作数
  bool _seq_ready = false;     // 序列收完标志

  bool _skip_approach = false; // 是否跳过接近动作

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
  uint8_t GetCurId(); // 获取当前梅林ID，供外部进行姿态校准
  float GetTargetYaw();

private:
  uint8_t _forward = 0;
  uint8_t _left = 0;
  bool _first_move = true;  // ← 首次 Forward 只落位不前进
  uint8_t _cur_block_id = 0; // 当前所在的梅林块编号
  // uint8_t _cur_block_id[3][4];
};

namespace Area2
{
extern MeiLinArray &mei_lin_array;
extern MeiLin_BlockPos &block_pos;
}