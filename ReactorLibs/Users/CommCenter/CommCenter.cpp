#include "CommCenter.hpp"
#include "HostPC.hpp"   
#include "Sick.hpp"
#include "optical_comm.hpp"
#include "string.h"

CommCenter& APP::comm_center = CommCenter::GetInstance();

void SlamRxCallback(uint8_t func_code, const uint8_t *payload, uint8_t payload_len, void *user_ctx);
void OpticalRxCallback(uint8_t *data, uint8_t len);

void CommCenter::Start()
{
    // 注册工控机接收SLAM的回调
    MOD::host_pc.Regist(HostPC::FuncCode::slam_pos, SlamRxCallback, nullptr);

    // 注册sick串口
    MOD::sick.Init(Hardware::huart_sick, 0x03); // 使用通道0和1
    // 初始化光通信模块
    MOD::optical_comm.Init(Hardware::huart_optical);
    // // 注册光通信数据回调（如果需要）
    MOD::optical_comm.Regist(0xC2, OpticalRxCallback); // kfs数据回调
    // MOD::optical_comm.Regist();
}

void CommCenter::Update()
{
    /*************** sick ****************/
    static int send_cnt = 0;
    if (++send_cnt >= 4) // 50hz
    {
        send_cnt = 0;
        // 发送SICK数据给工控机
        _sick_ch0_data = MOD::sick.GetSingleChannel(0); // 留作调试时看
        MOD::host_pc.SendSickData(MOD::sick.GetData().raw_frame);
    }
    // 发送SICK纠正指令
    if(_is_send_sick_correction)
    {
        MOD::host_pc.SendSlamCorrectionCmd();
        _is_send_sick_correction = false; // 重置标志位
    }

    /*************** KFS values ****************/
    if(_send_kfs)
    {
        // 这里可以添加发送KFS的逻辑，例如：
        // Vec3 kfs_values = ...; // 获取KFS数据
        // uint8_t payload[12];
        // kfs_values.ToBytes(payload);
    }
}

/**
 * @brief 触发sick纠正指令发送
 */
void CommCenter::SlamCorrection()
{
    _is_send_sick_correction = true;
}

/**
 * @brief 接收SLAM位置信息的回调函数
 */
void CommCenter::SlamRxCallback(uint8_t func_code, const uint8_t *payload, uint8_t payload_len, void *user_ctx)
{
    using namespace APP;
    if (func_code == HostPC::FuncCode::slam_pos)
    {
        // Vec3 slam_pos;
        comm_center.slam_pos.FromBytes(payload);

        // 车体坐标系->世界坐标系转换直接用偏移了
        // Vec3 bias{0.39f, 4.610f, 0.0f};

        // comm_center.slam_pos.x += bias.x;
        // comm_center.slam_pos.y += bias.y;

        // // 更新系统位姿
        // System.position = comm_center.slam_pos;

        // // 发送SLAM坐标数据转发给底盘
        // uint8_t data[6];
        // const float scale = 1000.0f;

        // StdMath::PackFloatToInt16(&data[0], slam_pos.x, scale);
        // StdMath::PackFloatToInt16(&data[2], slam_pos.y, scale);
        // StdMath::PackFloatToInt16(&data[4], slam_pos.z, scale);

        // MOD::board_can.SendTask(MOD::C_sub_ctrl_board_id, 0, data, 6, false);
    }
}

void CommCenter::OpticalRxCallback(uint8_t func_code, const uint8_t* payload, uint8_t payload_len, void* user_ctx)
{
    using namespace APP;
    // uint8_t kfs_values[12];
    memcpy(comm_center.kfs_values, payload, payload_len > 12 ? 12 : payload_len);

    // 发送给上位机
    MOD::host_pc.SendKFSData(comm_center.kfs_values);
}