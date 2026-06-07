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
#include "CommCenter.hpp"
#include "SickPC.hpp"

StateGraph graph("LoopGraph");

// void SlamRxCallback(uint8_t func_code, const uint8_t *payload, uint8_t payload_len, void *user_ctx);

// void Config_StateGraph();

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

  // // 注册工控机接收SLAM的回调
  // MOD::host_pc.Regist(HostPC::FuncCode::slam_pos, SlamRxCallback, nullptr);

  System.SetPositionSource(APP::comm_center.slam_pos);
  // System.SetPositionSource(APP::chas.chas_odom.pos); // yaw太不准了，用不了

  // 启动通信中心
  System.RegistApp(APP::comm_center);

  // 启动矛头控制应用s
  System.RegistApp(APP::jaw);

  // 启动伸缩腿控制
  // System.RegistApp(APP::leg);

  // 启动路径跟踪
  System.RegistApp(APP::path_chaser);

  // 底盘启动s
  System.RegistApp(APP::chas);
  APP::chas.Enable();

  System.RegistApp(APP::leg);

  System.RegistApp(APP::get_block);

  System.RegistApp(APP::sick_pc);

  // Config_AutoStateGraph(); // Auto：0
  Config_StateGraph(); // Manul：1
}

/***********    一些不通用的回调函数    ***********/
