#include "CommCenter.hpp"
#include "HostPC.hpp"   
#include "Sick.hpp"
#include "optical_comm.hpp"
#include "Jaw.hpp"
#include "string.h"

CommCenter& APP::comm_center = CommCenter::GetInstance();

// 代替static修饰符，这些作用域都局域化
namespace
{
// 一次光通信触发按 2mm 计算修正
constexpr float kOpticDockBiasStepMm = 2.0f;

// 对接时的 光通信命令码定义
enum DockOpticPayload : uint8_t
{
    DockCodeA_DockOK = 0x01,    // 完成对接
    DockCodeA_Vert = 0x02,      // 垂直方向矛头移动
    DockCodeA_Hori = 0x03,      // 水平方向矛头移动
    DockCodeA_GiveUp = 0x04,    // 放弃对接

    DockCodeB_DockOK = 0x02,
    DockCodeB_SpearUp = 0x10,
    DockCodeB_SpearDown = 0x11,
    DockCodeB_SpearLeft = 0x12,
    DockCodeB_SpearRight = 0x13,
    DockCodeB_GiveUp = 0x14,
};

// 检验payload的第三位是否满足验证要求
bool IsValidDockPayload(const uint8_t* payload, uint8_t payload_len)
{
    if (payload == nullptr || payload_len < 3)
    {
        return false;
    }

    return static_cast<uint8_t>(payload[0] + payload[1]) == payload[2];
}
}

void CommCenter::Start()
{
    // 注册工控机接收 SLAM的回调
    MOD::host_pc.Regist(HostPC::FuncCode::slam_pos, SlamRxCallback, nullptr);
    // 注册工控机接收 分辨 红蓝半场 与 一三区 反馈的回调
    MOD::host_pc.Regist(HostPC::FuncCode::field_area_feedback, FieldToggleRxCb, nullptr);

    // 注册sick串口
    MOD::sick.Init(Hardware::huart_sick, 0x03); // 使用通道0和1
    // 初始化光通信模块
    MOD::optical_comm.Init(Hardware::huart_optical);
    // // 注册光通信数据回调（如果需要）
    MOD::optical_comm.Regist(0xC2, OpticalRxCallback); // kfs数据回调
    MOD::optical_comm.Regist(0xC3, DockRxCallback);
}

void CommCenter::Update()
{
    /*************** sick ****************/
    static int send_cnt = 0;
    if (++send_cnt >= 4) // 50hz
    {
        send_cnt = 0;
        // 发送SICK数据给工控机
        sick_data_m[0] = MOD::sick.GetSingleChannel(0); // 留作调试时看，以及取矛头时校准
        sick_data_m[1] = MOD::sick.GetSingleChannel(1);
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

void CommCenter::ClearDockFlag()
{
    _dock_ok = false;
    _dock_give_up = false;
    APP::jaw.ClearOpticDockBias();
}

/**
 * @brief 接收SLAM位置信息的回调函数
 */
void CommCenter::SlamRxCallback(uint8_t func_code, const uint8_t *payload, uint8_t payload_len, void *user_ctx)
{
    using namespace APP;
    if (func_code == HostPC::FuncCode::slam_pos)
    {
        comm_center.slam_pos.FromBytes(payload);
    }
}

/**
 * @brief 接收SLAM位置信息的回调函数
 * @details payload[0] = 红蓝半场选择, payload[1] = 一三区重试选择
 */
void CommCenter::FieldToggleRxCb(uint8_t func_code, const uint8_t *payload, uint8_t payload_len, void *user_ctx)
{
    using namespace APP;
    if (func_code == HostPC::FuncCode::field_area_feedback)
    {
        if (payload_len < 2)
        {
            APP::monit.LogError("[HostPC] FieldToggleRxCb: payload too short.");
            return;
        }

        uint8_t field = payload[0];
        uint8_t area_retry = payload[1];

        // 首先，判断工控机当前认为的红蓝区
        if (field == static_cast<uint8_t>(HostPC::Field::RED))
        {
            comm_center.is_blue_field = false;
        }
        else if (field == static_cast<uint8_t>(HostPC::Field::BLUE))
        {
            comm_center.is_blue_field = true;
        }
        else
        {
            APP::monit.LogError("[comm] Unknown field value: %d", field);
        } 

        // 接着，判断工控机当前认为的一三区重试选择
        if (area_retry == static_cast<uint8_t>(HostPC::AreaRetry::FIRST))
        {
            comm_center.at_first_area = true;
        }
        else if (area_retry == static_cast<uint8_t>(HostPC::AreaRetry::THIRD))
        {
            comm_center.at_first_area = false;
        }
        else
        {
            APP::monit.LogError("[comm] Unknown area retry value: %d", area_retry);
        }
    }
}

void CommCenter::OpticalRxCallback(uint8_t func_code, const uint8_t* payload, uint8_t payload_len, void* user_ctx)
{
    using namespace APP;
    
    switch (func_code) 
    {
        case 0xC2: // KFS数据
        {
            // R1 发来的顺序：逆序。12->1
            uint8_t len = payload_len > 12 ? 12 : payload_len;      // 限制payload长度，最多12字节

            for (uint8_t i = 0; i < len; i++)
            {
                comm_center.kfs_values[i] = payload[len - 1 - i];
            }

            // 发送给上位机
            MOD::host_pc.SendKFSData(comm_center.kfs_values, len);
            break;
        }
    }
}

void CommCenter::DockRxCallback(uint8_t func_code, const uint8_t* payload, uint8_t payload_len, void* user_ctx)
{
    (void)user_ctx;

    if(func_code == 0xC3)
    {
        // 0.5s内最多处理一次，避免对接偏置被过快重复触发
        static float last_trigger_time = -0.5f;
        const float now = DWT_GetTimeline_Sec();
        if (now - last_trigger_time < 0.5f)
        {
            return;
        }

        // 验证，至少要有东西传过来，长度符合实际，并且满足相加校验
        if (!IsValidDockPayload(payload, payload_len))
        {
            APP::monit.LogError("[Optic] Invalid dock payload.");
            return;
        }

        last_trigger_time = now;

        // 解析payload，对接这里使用两位 内容码
        const uint8_t code_a = payload[0];
        const uint8_t code_b = payload[1];

        // 0x01, 0x02: 对接完成
        if (code_a == DockCodeA_DockOK && code_b == DockCodeB_DockOK)
        {
            APP::monit.LogRespond("[Optic] Docking complete.");
            APP::comm_center._dock_ok = true;
            return;
        }

        // 0x02, 0x10: 垂直方向上矛头上移
        if (code_a == DockCodeA_Vert && code_b == DockCodeB_SpearUp)
        {
            APP::jaw.AddOpticDockBias(0.0f, kOpticDockBiasStepMm);
        }
        // 0x02, 0x11: 垂直方向上矛头下移
        else if (code_a == DockCodeA_Vert && code_b == DockCodeB_SpearDown)
        {
            APP::jaw.AddOpticDockBias(0.0f, -kOpticDockBiasStepMm);
        }
        // 0x03, 0x12: 水平方向上矛头左移
        else if (code_a == DockCodeA_Hori && code_b == DockCodeB_SpearLeft)
        {
            APP::jaw.AddOpticDockBias(-kOpticDockBiasStepMm, 0.0f);
        }
        // 0x03, 0x13: 水平方向上矛头
        else if (code_a == DockCodeA_Hori && code_b == DockCodeB_SpearRight)
        {
            APP::jaw.AddOpticDockBias(kOpticDockBiasStepMm, 0.0f);
        }
        // 0x04, 0x14: 对接放弃
        else if (code_a == DockCodeA_GiveUp && code_b == DockCodeB_GiveUp)
        {
            APP::monit.LogRespond("[Optic] GiveUp Docking...");
            // 标志位置位
            APP::comm_center._dock_give_up = true;
            APP::comm_center._dock_ok = true;
            return;
        }
        else
        {
            APP::monit.LogError("[Optic] Unknown dock cmd: 0x%02X 0x%02X", code_a, code_b);
            return;
        }

        Vec2 bias = APP::jaw.GetOpticDockBias();
        APP::monit.LogRespond("[Optic] Dock bias: x=%.1fmm y=%.1fmm", bias.x, bias.y);
    }
}
