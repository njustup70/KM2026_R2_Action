#include "optical_comm.hpp"
#include "string.h"
#include "bsp_log.hpp"

OpticalComm& MOD::optical_comm = OpticalComm::GetInstance();

void OpticalComm::Init(BSP::UART::UartID uart_id)
{
    // 清空路由表与去重表
    memset(_routes, 0, sizeof(_routes));
    memset(_unknown_once, 0, sizeof(_unknown_once));

    _uart = BSP::UART::Apply(uart_id);
    if (_uart.IsValid())
    {
        _uart.RegisterRx(MAX_FRAME_SIZE, _RxCallback);
    }
}

bool OpticalComm::Regist(uint8_t func_code, RxHandler cb, void* user_ctx)
{
    if (cb == nullptr)
    {
        BspLog_LogError("[OpticalComm] Regist failed: cb is nullptr, func=0x%02X", func_code);
        return false;
    }

    // 已存在同功能码 -> 覆盖
    int index = FindRouteIndex(func_code);
    if (index >= 0)
    {
        _routes[index].cb       = cb;
        _routes[index].user_ctx = user_ctx;
        return true;
    }

    // 找空槽
    for (uint8_t i = 0; i < MAX_RX_HANDLERS; ++i)
    {
        if (!_routes[i].used)
        {
            _routes[i].func_code = func_code;
            _routes[i].cb        = cb;
            _routes[i].user_ctx  = user_ctx;
            _routes[i].used      = true;
            return true;
        }
    }

    BspLog_LogError("[OpticalComm] Regist failed: route table full, func=0x%02X", func_code);
    return false;
}

void OpticalComm::SendData(uint8_t func_code, uint8_t* payload, uint8_t len)
{
    uint8_t buf[MAX_FRAME_SIZE] = {0};
    buf[0] = frame_head;
    buf[1] = func_code;
    buf[2] = len > MAX_DATA_SIZE ? MAX_DATA_SIZE : len;
    memcpy(&buf[3], payload, buf[2]);
    _uart.Transmit(buf, MAX_FRAME_SIZE);
}

// ── 串口接收桥接 ────────────────────────────────────────────────

void OpticalComm::_RxCallback(BSP::UART::UartID id, uint8_t* rxData, uint8_t size)
{
    (void)id;
    GetInstance().ProcessFrame(rxData, size);
}

// ── 帧解析与分发 ────────────────────────────────────────────────

void OpticalComm::ProcessFrame(uint8_t* data, uint8_t size)
{
    if (data == nullptr || size < 3)
        return;

    if (data[0] != frame_head)
    {
        BspLog_LogError("[OpticalComm] Invalid frame head: 0x%02X", data[0]);
        return;
    }

    uint8_t func_code   = data[1];
    uint8_t payload_len = data[2];
    if (payload_len > MAX_DATA_SIZE)
        payload_len = MAX_DATA_SIZE;

    if (size < 3 + payload_len)
    {
        BspLog_LogError("[OpticalComm] Frame too short: need %u, got %u", 3 + payload_len, size);
        return;
    }

    const uint8_t* payload = (payload_len > 0) ? &data[3] : nullptr;

    // 分发到已注册回调
    int index = FindRouteIndex(func_code);
    if (index >= 0)
    {
        _routes[index].cb(func_code, payload, payload_len, _routes[index].user_ctx);
        return;
    }

    // 未注册功能码仅首条报错
    if (!IsUnknownReported(func_code))
    {
        BspLog_LogError("[OpticalComm] Unregistered func_code=0x%02X, len=%u", func_code, size);
        MarkUnknownReported(func_code);
    }
}

// ── 路由查找 ────────────────────────────────────────────────────

int OpticalComm::FindRouteIndex(uint8_t func_code) const
{
    for (uint8_t i = 0; i < MAX_RX_HANDLERS; ++i)
    {
        if (_routes[i].used && _routes[i].func_code == func_code)
            return i;
    }
    return -1;
}

// ── 未注册帧去重 ────────────────────────────────────────────────

bool OpticalComm::IsUnknownReported(uint8_t func_code) const
{
    for (uint8_t i = 0; i < MAX_UNREGISTERED_KEYS; ++i)
    {
        if (_unknown_once[i].used && _unknown_once[i].func_code == func_code)
            return true;
    }
    return false;
}

void OpticalComm::MarkUnknownReported(uint8_t func_code)
{
    // 已存在则跳过
    for (uint8_t i = 0; i < MAX_UNREGISTERED_KEYS; ++i)
    {
        if (_unknown_once[i].used && _unknown_once[i].func_code == func_code)
            return;
    }
    // 找空槽
    for (uint8_t i = 0; i < MAX_UNREGISTERED_KEYS; ++i)
    {
        if (!_unknown_once[i].used)
        {
            _unknown_once[i].func_code = func_code;
            _unknown_once[i].used      = true;
            return;
        }
    }
}