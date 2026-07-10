/**
 * @file IR_RX.hpp
 * @brief 红外接收模块 (NEC 协议)
 * @note 使用 GPIO EXTI 捕获边沿，状态机解码，支持多实例
 */

#pragma once

#include "bsp_gpio.hpp"
#include "bsp_dwt.hpp"
#include <stdint.h>
#include "string.h"

/// @brief 最大同时存在的 IR_RX 实例数（用于 EXTI 静态路由表）
static constexpr uint8_t IRRX_MAX_INSTANCES = 8;

// 前向声明
class IR_RX;

/// @brief NEC 帧接收回调
/// @param rx       触发回调的 IR_RX 实例指针（区分来自哪个接收头）
/// @param address  8-bit 地址
/// @param command  8-bit 命令
/// @param is_repeat 是否为重复码
/// @param user_ctx 用户上下文
using IRRxHandler = void (*)(const IR_RX *rx,
                              uint8_t address, uint8_t command,
                              bool is_repeat, void *user_ctx);

/**
 * @class IR_RX
 * @brief 红外接收模块 (NEC 协议)，仅接收
 *
 * 使用方式:
 * @code
 * IR_RX ir_front;
 * IR_RX ir_back;
 *
 * ir_front.Init(Pin{'C', 0});
 * ir_front.RegisterRxHandler(OnNECFrame, nullptr);
 *
 * ir_back.Init(Pin{'C', 1});
 * ir_back.RegisterRxHandler(OnNECFrame, nullptr);
 *
 * // 主循环中调用
 * ir_front.Handle();
 * ir_back.Handle();
 * @endcode
 */
class IR_RX
{
public:
    IR_RX();
    ~IR_RX();

    IR_RX(const IR_RX &) = delete;
    IR_RX &operator=(const IR_RX &) = delete;

    /**
     * @brief 初始化
     * @param rx_pin 红外接收头输出引脚（需支持 EXTI）
     */
    void Init(Pin rx_pin);

    /**
     * @brief 注册接收回调
     * @param cb       回调函数
     * @param user_ctx 用户上下文
     */
    void RegisterRxHandler(IRRxHandler cb, void *user_ctx = nullptr);

    /**
     * @brief 周期性调用（驱动解码状态机 + 超时检测）
     * @note 建议 1kHz 以上频率调用
     */
    void Handle();

    // ==================== 公共可配置参数 ====================

    /// @brief 本机地址过滤: 只触发 Addr 匹配的回调, 0xFF = 接收全部
    uint8_t local_addr = 0;

    // ---- NEC 协议时序参数 (单位: us) ----
    uint16_t leader_on_us  = 9000;
    uint16_t leader_off_us = 4500;
    uint16_t repeat_off_us = 2250;
    uint16_t bit_on_us     = 563;
    uint16_t bit_0_off_us  = 563;
    uint16_t bit_1_off_us  = 1687;

    /// @brief 时序容差 (0.0 ~ 1.0)，默认 0.30
    float timing_tolerance = 0.50f;

private:
    // ==================== EXTI 处理 ====================
    static void _ExtiRouter(Pin pin);
    void _OnExtiEvent(uint32_t duration_us, bool is_rising);
    bool _TimingInRange(uint32_t duration, uint32_t target) const;

    // ==================== NEC 接收状态机 ====================
    enum class RxState : uint8_t
    {
        Idle,
        LeaderOn,
        LeaderOff,
        BitOn,
        BitOff,
        RepeatWait,
    };

    struct
    {
        RxState  state         = RxState::Idle;
        uint32_t last_edge_us  = 0;
        uint32_t data          = 0;
        uint8_t  bit_count     = 0;
        uint8_t  address       = 0;
        uint8_t  command       = 0;
    } _rx;

    struct
    {
        IRRxHandler cb       = nullptr;
        void       *user_ctx = nullptr;
    } _rx_handler;

    bool            _initialized = false;
    BSP::GPIO::Inst _rx_gpio;
    Pin             _rx_pin = {0, 0};

    // ==================== 静态 EXTI 路由表 ====================
    struct ExtiRoute
    {
        Pin    pin;
        IR_RX *instance;
        bool   used;
    };
    static ExtiRoute _exti_routes[IRRX_MAX_INSTANCES];
    static uint8_t   _exti_route_count;
};
