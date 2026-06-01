#include "CommCenter.hpp"
#include "HostPC.hpp"   

CommCenter& APP::comm_center = CommCenter::GetInstance();

void SlamRxCallback(uint8_t func_code, const uint8_t *payload, uint8_t payload_len, void *user_ctx);

void CommCenter::Start()
{
    // 注册工控机接收SLAM的回调
    MOD::host_pc.Regist(HostPC::FuncCode::slam_pos, SlamRxCallback, nullptr);
}

void CommCenter::Update()
{

}

/**
 * @brief 接收SLAM位置信息的回调函数
 */
void CommCenter::SlamRxCallback(uint8_t func_code, const uint8_t *payload, uint8_t payload_len, void *user_ctx)
{
    using namespace APP;
    if (func_code == HostPC::FuncCode::slam_pos)
    {
        Vec3 slam_pos;
        slam_pos.FromBytes(payload);

        // 更新系统位姿
        System.position = slam_pos;

        // // 发送SLAM坐标数据转发给底盘
        // uint8_t data[6];
        // const float scale = 1000.0f;

        // StdMath::PackFloatToInt16(&data[0], slam_pos.x, scale);
        // StdMath::PackFloatToInt16(&data[2], slam_pos.y, scale);
        // StdMath::PackFloatToInt16(&data[4], slam_pos.z, scale);

        // MOD::board_can.SendTask(MOD::C_sub_ctrl_board_id, 0, data, 6, false);
    }
}
