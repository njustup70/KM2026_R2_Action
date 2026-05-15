#include "MainFrame.hpp"
#include "Monitor.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "Jaw.hpp"
#include "Get_Block.hpp"
#include "HostPC.hpp"
#include "Interboardcomm.hpp"
#include "farcon.hpp"

void SlamRxCallback(uint8_t func_code, const uint8_t* payload, uint8_t payload_len, void* user_ctx);

// 手动遥控的状态机
StateGraph ManualGraph("Manual");


void Action_of_Dege(StateCore *core);

/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
  // 初始化板间通信
  MOD::board_can.Init(Hardware::hcan_sub, 0x220, false);
  // 初始化工控机通信
  MOD::host_pc.Init(Hardware::huart_host);
  // 初始化遥控器
  MOD::farcon.init(Hardware::huart_farcon);

  MOD::host_pc.Regist(HostPC::FuncCode::slam_pos, SlamRxCallback, nullptr);

  // 底盘启动
  // System.RegistApp(chas);
  // chas.Enable();

  

  // System.RegistApp(jaw);
  // System.RegistApp(key);
  // System.RegistApp(hand);
  
  // 配置状态图

  // 向状态机核心注册
  APP::state_core.RegistGraph(ManualGraph);
  APP::state_core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
}

void Waiting_for_Start(StateCore *core)
{
    // 等待
    
}

void SlamRxCallback(uint8_t func_code, const uint8_t* payload, uint8_t payload_len, void* user_ctx)
{
    if(func_code == HostPC::FuncCode::slam_pos)
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

        MOD::board_can.SendTask(0x210, 0, data, 6, false);
    }
}