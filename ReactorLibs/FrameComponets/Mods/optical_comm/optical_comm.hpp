#pragma once
#include "stdint.h"
#include "bsp_uart.hpp"

// 光通信库
class OpticalComm
{
private:
    // 最大帧长度
    constexpr static uint8_t MAX_FRAME_SIZE = 24;
    // 数据载荷最大长度
    constexpr static uint8_t MAX_DATA_SIZE = MAX_FRAME_SIZE - 3;

    // 发送帧头
    const static uint8_t frame_head = 0xFF;
    // 发送帧类型
    const static uint8_t frame_type = 0xC2;

    BSP::UART::Handler _uart;
    uint8_t _rx_buffer[MAX_DATA_SIZE];
    uint8_t _rx_len;
    bool    _new_data_flag;

    static void _RxCallback(BSP::UART::UartID id, uint8_t *rxData, uint8_t size);

public:
    // 数据接收回调函数类型
    using DataCallback = void (*)(uint8_t *data, uint8_t len);

    OpticalComm(){};

    static OpticalComm& GetInstance()
    {
        static OpticalComm instance;
        return instance;
    }

    OpticalComm(const OpticalComm&) = delete;
    OpticalComm& operator=(const OpticalComm&) = delete;

    void Init(BSP::UART::UartID uart_id);
    void SendData(uint8_t* data, uint8_t len);

    /// @brief 注册数据接收回调（收到完整一帧时在中断中触发）
    void RegisterDataCallback(DataCallback callback);

    /// @brief 获取最新接收的数据（轮询方式）
    /// @param[out] buf     输出缓冲区
    /// @param[in]  max_len 缓冲区最大长度
    /// @return 实际拷贝的数据长度
    uint8_t GetReceivedData(uint8_t* buf, uint8_t max_len);

    /// @brief 检查是否有新数据到达
    bool HasNewData() const { return _new_data_flag; }

private:
    DataCallback _data_callback;
};

namespace MOD
{
    extern OpticalComm& optical_comm;
}
