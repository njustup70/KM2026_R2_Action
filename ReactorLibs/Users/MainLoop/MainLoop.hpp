#pragma once
#include "System.hpp"
#include "PathChaser.hpp"
#include "Chassis.hpp"
#include "led_ws2812.hpp"
#include "IR_RX.hpp"
#include "IR_TX.hpp"

class MainLoop : public Application
{
    SINGLETON(MainLoop) : Application("MainLoop") {};
    APPLICATION_OVERRIDE

    public:
    LedWs2812<13> test_led;
    int cnt = 0;

    IR_RX  ir_rx;
    IR_TX  ir_tx;

    /// @brief 红外发送（带序号日志）
    void SendIR(uint8_t address, uint8_t command);

private:
    uint16_t _ir_tx_seq      = 0;   // 发送序号
    uint16_t _ir_rx_seq      = 0;   // 接收序号
    uint8_t  _ir_tx_last_addr = 0;  // DMA回调用：最近发送的地址
    uint8_t  _ir_tx_last_cmd  = 0;  // DMA回调用：最近发送的命令

    static void _IrTxDoneCb(BSP::TIM::TimID tim, BSP::TIM::Channel ch, void *ctx);
};

namespace APP
{
    extern MainLoop& main_loop;
}

/**************  移动的封装  **************/ 
namespace MOVE
{
    /// @brief 沿预设路径移动并等待完成
    /// @param path_data 路径数据
    /// @return true-到达终点, false-失败

    template <size_t N>
    bool MoveToTargPos(const Path<N>& path)
    {
        if (!APP::path_chaser.ChasePath(path))
        {
            APP::monit.LogError("[AutoGraph] MoveToTargPos rejected by PathChaser.");
            return false;
        }
        uint32_t log_tick = 0;
        
        while (!APP::path_chaser.IsFinished())
        {
            Vec3 speed_vec = APP::path_chaser.GetCmdBody();
            Vec3 world_vec = APP::path_chaser.GetCmdWorld();
            APP::chas.Move(speed_vec, 0.01f);

            if (++log_tick >= 20)
            {
                log_tick = 0;
                APP::monit.LogInfo("world_vec(%.3f, %.3f, %.3f)",
                                   world_vec.x, world_vec.y, world_vec.z);
            }
            Seq::Wait(0.005f); // 200hz更新频率
        }
        
        APP::monit.LogInfo("Move to target position.");
        return true;
    }

    /// @brief 普通跑点移动并等待完成
    /// @param targ_ges 目标位姿，世界系(x, y, yaw)，单位m/m/rad
    /// @return true-到达目标, false-失败
    inline bool MoveToTargGes(Vec3 targ_ges)
    {
        APP::path_chaser.ChaseGes(targ_ges);
        uint32_t log_tick = 0;

        while (!APP::path_chaser.IsFinished())
        {
            Vec3 speed_vec = APP::path_chaser.GetCmdBody();
            Vec3 world_vec = APP::path_chaser.GetCmdWorld();
            APP::chas.Move(speed_vec, 0.01f);

            if (++log_tick >= 20)
            {
                log_tick = 0;
                APP::monit.LogInfo("pt_world_vec(%.3f, %.3f, %.3f)",
                                   world_vec.x, world_vec.y, world_vec.z);
            }
            Seq::Wait(0.005f); // 200hz更新频率
        }

        APP::monit.LogInfo("Move to target gesture.");
        return true;
    }
}