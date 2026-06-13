#pragma once
#include "stdint.h"
#include "bsp_uart.hpp"

// 光通信库
class OpticalComm
{
public:
    /// @brief 分发表容量
    static constexpr uint8_t MAX_RX_HANDLERS = 16;
    /// @brief 未注册帧告警去重表容量
    static constexpr uint8_t MAX_UNREGISTERED_KEYS = 32;

    /// @brief 接收回调签名（App层注册）
    using RxHandler = void(*)(uint8_t func_code, const uint8_t* payload, uint8_t payload_len, void* user_ctx);

    static OpticalComm& GetInstance()
    {
        static OpticalComm instance;
        return instance;
    }

    OpticalComm(const OpticalComm&) = delete;
    OpticalComm& operator=(const OpticalComm&) = delete;

    void Init(BSP::UART::UartID uart_id);

    /// @brief 注册指定功能码的回调
    /// @param func_code 功能码
    /// @param cb        回调函数
    /// @param user_ctx  用户上下文指针，通常传 App 实例 this
    /// @retval true  注册成功（同功能码重复注册覆盖旧回调）
    /// @retval false 注册失败（空回调或表满）
    bool Regist(uint8_t func_code, RxHandler cb, void* user_ctx = nullptr);

    /// @brief 发送一帧数据
    /// @param func_code 功能码
    /// @param payload   数据载荷
    /// @param len       载荷长度
    void SendData(uint8_t func_code, uint8_t* payload, uint8_t len);

    /// @brief 预设一键配置
    struct RecvConfig
    {
        uint8_t     func_code;
        RxHandler   cb;
        void*       user_ctx;
    };

private:
    static constexpr uint8_t MAX_FRAME_SIZE = 24;
    static constexpr uint8_t MAX_DATA_SIZE  = MAX_FRAME_SIZE - 3;
    static const uint8_t frame_head = 0xFF;

    /// @brief 分发表项
    struct RxRouteEntry
    {
        uint8_t     func_code;
        RxHandler   cb;
        void*       user_ctx;
        bool        used;
    };

    /// @brief 未注册帧告警去重键
    struct FrameKey
    {
        uint8_t func_code;
        bool    used;
    };

    OpticalComm() = default;

    /// @brief 串口接收静态入口，转发到 ProcessFrame
    static void _RxCallback(BSP::UART::UartID id, uint8_t* rxData, uint8_t size);
    /// @brief 帧解析与分发（仅解析 + 路由，无业务逻辑）
    void ProcessFrame(uint8_t* data, uint8_t size);

    /// @brief 在分发表中查找功能码
    int FindRouteIndex(uint8_t func_code) const;
    /// @brief 查询未注册功能码是否已告警
    bool IsUnknownReported(uint8_t func_code) const;
    /// @brief 标记未注册功能码已告警
    void MarkUnknownReported(uint8_t func_code);

    BSP::UART::Handler _uart;
    RxRouteEntry _routes[MAX_RX_HANDLERS]        = {};
    FrameKey     _unknown_once[MAX_UNREGISTERED_KEYS] = {};
};

namespace MOD
{
    extern OpticalComm& optical_comm;
}
