#pragma once
#include "System.hpp"

// 后续有关通信的内容可放这里，方便管理用的

enum Area3ActCode : uint8_t
{
    Idle = 0x00,            // 空闲状态，请保持等待，什么都不要做！！
    Put_L2_N4_Block = 0x11, // 将块放在第二层的四号位置上
    Put_L2_N5_Block = 0x12, // 将块放在第二层的五号位置上
    Put_L2_N6_Block = 0x13, // 将块放在第二层的六号位置上
    Put_L3 = 0x14,          // R2已经合体，无需在意位置，由R1接管

    Combine = 0x20, // 准备与 R1 合体

    Fetch_GroundBlock_0 = 0x30, // 从地面 取块0
    Fetch_GroundBlock_1 = 0x31, // 从地面 取块1
};

class CommCenter : public Application
{
    SINGLETON(CommCenter) : Application("CommCenter") {};

    APPLICATION_OVERRIDE

private:
    bool _send_kfs; // 是否发送KFS给上位机
    bool _is_send_sick_correction; // 是否发送SICK纠正指令
    bool _dock_ok = false;  // 是否完成对接
    Area3ActCode _area3_act_code; // 三区光通信，接收到的指令码

    // float _sick_ch0_data; // sick测距值（ch0）


public:
    Vec3 slam_pos;
    float _sick_ch0_data; // sick测距值（ch0）
    uint8_t kfs_values[12];
    float sick_data_m[4]; // sick测距值（ch0 - ch3），单位m

    bool IsDocked() const { return _dock_ok; }
    void ClearDockFlag() { _dock_ok = false; }
    void SlamCorrection(); // 触发Sick纠正
    static void SlamRxCallback(uint8_t func_code, const uint8_t *payload, uint8_t payload_len, void *user_ctx);
    static void OpticalRxCallback(uint8_t func_code, const uint8_t* payload, uint8_t payload_len, void* user_ctx);
    static void DockRxCallback(uint8_t func_code, const uint8_t* payload, uint8_t payload_len, void* user_ctx);
    
    Area3ActCode Get_R1_InstruCode() const { return _area3_act_code; }
    
    /// @warning 享受完指令后，请务必调用 Reset_R1_InstruCode() 来清除指令码，避免重复执行
    void Reset_R1_InstruCode() { _area3_act_code = Area3ActCode::Idle; }

    /**
     * @brief 三区光通信，接收来自R1的指令，路由至不同状态
     * 
     */
    static void Area3_OpticRxCb(uint8_t func_code, const uint8_t* payload, uint8_t payload_len, void* user_ctx);
};

namespace APP
{
    extern CommCenter& comm_center;
}