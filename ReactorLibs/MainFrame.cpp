#include "MainFrame.hpp"
#include "Monitor.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "Jaw.hpp"
#include "Get_Block.hpp"
#include "HostPC.hpp"
#include "Interboardcomm.hpp"
#include "farcon.hpp"
#include "Jaw.hpp"
// #include "ManualGraph.hpp"
#include "AutoGraph.hpp"
#include "Lift_Leg.hpp"
#include "Jaw.hpp"
#include "ManualGraph.hpp"
#include "board.hpp"

StateGraph graph("LoopGraph");

void SlamRxCallback(uint8_t func_code, const uint8_t *payload, uint8_t payload_len, void *user_ctx);

// void Config_StateGraph();

// void State1Action(StateCore *core)
// {
//   APP::path_chaser.ChasePath(GeneratedPath);
//   Seq::Wait(1);
//   core->GetCurState().Complete = true;
// }

// void State2Action(StateCore *core)
// {
//   Vec3 cmd_body = APP::path_chaser.GetCmdBody();
//   APP::chas.Move(cmd_body);
//   // core->GetCurState().Complete = true;
// }

// void SetUp()
// {
//   StateBlock &state1 = graph.AddState("state1");
//   StateBlock &state2 = graph.AddState("state2");

//   state1.StateAction = State1Action;
//   state2.StateAction = State2Action;

//   // 状态1完成后 -> 状态2
//   state1.LinkTo(&state1.Complete, state2);

//   // 状态2完成后 -> 状态2自身（循环）
//   // state2.LinkTo(&state2.Complete, state2);

//   StateCore &core = StateCore::GetInstance();
//   core.RegistGraph(graph);
//   core.Enable(0);
// }

/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
  // 初始化板间通信
  MOD::board_can.Init(Hardware::hcan_sub, 0x240, false);
  // 初始化工控机通信
  MOD::host_pc.Init(Hardware::huart_host);
  // 初始化遥控器
  MOD::farcon.init(Hardware::huart_farcon);

  // 注册工控机接收SLAM的回调
  MOD::host_pc.Regist(HostPC::FuncCode::slam_pos, SlamRxCallback, nullptr);

    // 启动矛头控制应用
      System.RegistApp(APP::jaw);
    
    // 启动伸缩腿控制
    // System.RegistApp(APP::leg);

    // 底盘启动
    System.RegistApp(APP::chas);
    // APP::chas.Enable();

    Config_AutoStateGraph();
}

/***********    一些不通用的回调函数    ***********/
/**
 * @brief 接收SLAM位置信息的回调函数
 */
void SlamRxCallback(uint8_t func_code, const uint8_t *payload, uint8_t payload_len, void *user_ctx)
{
    if (func_code == HostPC::FuncCode::slam_pos)
    {
        Vec3 slam_pos;
        slam_pos.FromBytes(payload);

        // 更新系统位姿
        System.position = slam_pos;

        // 发送SLAM坐标数据转发给底盘
        uint8_t data[6];
        const float scale = 1000.0f;

        StdMath::PackFloatToInt16(&data[0], slam_pos.x, scale);
        StdMath::PackFloatToInt16(&data[2], slam_pos.y, scale);
        StdMath::PackFloatToInt16(&data[4], slam_pos.z, scale);

        MOD::board_can.SendTask(MOD::C_sub_ctrl_board_id, 0, data, 6, false);
    }
}
