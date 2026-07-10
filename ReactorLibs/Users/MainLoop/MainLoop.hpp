#pragma once
#include "System.hpp"
#include "PathChaser.hpp"
#include "Chassis.hpp"
#include "led_ws2812.hpp"
#include "infrared_detector.hpp"
#include "IR_RX.hpp"
#include "IR_TX.hpp"

class MainLoop : public Application
{
    SINGLETON(MainLoop) : Application("MainLoop") {};
    APPLICATION_OVERRIDE

    public:
    LedWs2812<13> test_led;

    // 取矛头的小黄
    InfraredDetector spear_detector;
    bool spear_detected = false;

    // 重试/切场按钮，当小黄处理了
    InfraredDetector button_0;
    InfraredDetector button_1;
    InfraredDetector button_2;
    InfraredDetector button_3;
    bool button_pressed[4] = {false, false, false, false};

    /*--------------        切区逻辑        ----------------*/
    /// @brief 所有切换红蓝区的逻辑，都封装在里边
    void ToggleAreaLogics();
    /// @brief 刷新灯带红蓝区显示：前半段为STM32选区，后半段为工控机回传选区
    void RefreshFieldLed();
    /// @brief 刷新灯带第 3 段显示重试类型
    void RefreshRetryLed();

    // 所选的区域（分红区蓝区）
    const static uint8_t AREA_RED = 0;
    const static uint8_t AREA_BLUE = 1;
    uint8_t targ_area = AREA_RED;   // 默认红区
    bool graph_started = false;     // 状态机是否已经启动

    // 启动的坐标（崇武探幽+竞技赛启动坐标 / 九宫藏宝启动坐标）
    const static uint8_t AREA_ONE = 0;
    const static uint8_t AREA_THREE = 1;
    uint8_t targ_start_area = AREA_ONE; // 默认崇武探幽+竞技赛

    /****    常用的状态图   ****/
    /// @brief "崇武探幽" 一二区技能赛
    uint8_t Challenge_Explore_RedGr_id = 0;
    uint8_t Challenge_Explore_BlueGr_id = 0;
    
    /// @brief "九宫藏宝" 三区技能赛
    uint8_t Challenge_Treasure_RedGr_id = 0;
    uint8_t Challenge_Treasure_BlueGr_id = 0;

    /// @brief "武林探秘" 竞技赛正赛
    uint8_t Compete_KM_RedGr_id = 0;
    uint8_t Compete_KM_BlueGr_id = 0;

    /*******        重试标志位      *******/
    void JudgeRetryFlags();
    uint8_t retry_id = 0; // 区分重试类型
    uint8_t max_retry_id = 3; // 最大重试类型编号

    int cnt = 0;

    uint16_t _ir_tx_seq       = 0;   // 发送序号

    IR_RX  ir_rx;
    IR_TX  ir_tx;

    /// @brief 红外发送（带序号日志）
    void SendIR(uint8_t address, uint8_t command);

private:
   
    uint16_t _ir_rx_seq       = 0;   // 接收序号
    uint8_t  _ir_tx_last_addr = 0;   // DMA回调用：最近发送的地址
    uint8_t  _ir_tx_last_cmd  = 0;   // DMA回调用：最近发送的命令

    bool     _ir_rx_pending   = false;   // 收到新帧标记
    uint8_t  _ir_rx_last_addr = 0;   // 最近接收的地址
    uint8_t  _ir_rx_last_cmd  = 0;   // 最近接收的命令

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

    /// @brief 沿预设路径移动并等待完成，使用本次路径单独的终点容差
    /// @param path_data 路径数据
    /// @param tolerance 本次路径单独使用的终点容差(x/y/yaw)，单位m/m/rad
    /// @return true-到达终点, false-失败
    template <size_t N>
    bool MoveToTargPos(const Path<N>& path, Vec3 tolerance)
    {
        if (!APP::path_chaser.ChasePath(path, tolerance))
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
                // APP::monit.LogInfo("world_vec(%.3f, %.3f, %.3f)",
                //                    world_vec.x, world_vec.y, world_vec.z);
            }
            Seq::Wait(0.005f); // 200hz更新频率
        }

        APP::monit.LogInfo("Move to target position.");
        return true;
    }

    template <size_t N>
    bool MoveToTargPos(const Path<N>& path)
    {
        return MoveToTargPos(path, Vec3(-1.0f, -1.0f, -1.0f));
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
                // APP::monit.LogInfo("pt_world_vec(%.3f, %.3f, %.3f)",
                //                    world_vec.x, world_vec.y, world_vec.z);
            }
            Seq::Wait(0.005f); // 200hz更新频率
        }

        APP::monit.LogInfo("Move to target gesture.");
        return true;
    }
}
