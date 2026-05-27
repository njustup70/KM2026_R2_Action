#include "SickPC.hpp"
#include "HostPC.hpp"
#include "Sick.hpp"

SickPC& APP::sick_pc = SickPC::GetInstance();

void SickPC::Start()
{
    // 初始化sick模块
    MOD::sick.Init(Hardware::huart_sick, 0x03); // 使用通道0和1
}

void SickPC::Update()
{
    // 50hz发送四个通道数据
    static int send_cnt = 0;
    if (++send_cnt >= 4) // 50hz
    {
        send_cnt = 0;
        // 发送SICK数据给工控机
        MOD::host_pc.SendSickData(MOD::sick.GetData().raw_frame);
    }
}