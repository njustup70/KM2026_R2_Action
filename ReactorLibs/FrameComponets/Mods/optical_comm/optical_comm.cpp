#include "optical_comm.hpp"
#include "string.h"
#include "bsp_log.hpp"

OpticalComm& MOD::optical_comm = OpticalComm::GetInstance();

void OpticalComm::Init(BSP::UART::UartID uart_id)
{
    _uart = BSP::UART::Apply(uart_id);
    if (_uart.IsValid())
    {
        // 注册UART接收回调，每次期望接收一帧完整数据
        _uart.RegisterRx(MAX_FRAME_SIZE, _RxCallback);
    }
    _rx_len = 0;
    _new_data_flag = false;
    _data_callback = nullptr;
}

void OpticalComm::SendData(uint8_t fun_code, uint8_t* payload, uint8_t len)
{
    uint8_t buf[MAX_FRAME_SIZE] = {0};
    buf[0] = frame_head;
    buf[1] = fun_code;
    buf[2] = len;
    memcpy(&buf[3], payload, len > MAX_DATA_SIZE ? MAX_DATA_SIZE : len);
    _uart.Transmit(buf, sizeof(buf));
}

void OpticalComm::RegisterDataCallback(DataCallback callback)
{
    _data_callback = callback;
}

uint8_t OpticalComm::GetReceivedData(uint8_t* buf, uint8_t max_len)
{
    if (!_new_data_flag) return 0;

    uint8_t copy_len = (_rx_len < max_len) ? _rx_len : max_len;
    memcpy(buf, _rx_buffer, copy_len);
    _new_data_flag = false;
    _rx_len = 0;
    return copy_len;
}

void OpticalComm::_RxCallback(BSP::UART::UartID id, uint8_t *rxData, uint8_t size)
{
    (void)id;
    auto& self = GetInstance();

    // 检查帧头、帧类型
    if (size < 3 || rxData[0] != frame_head)
    {
        BspLog_LogError("[OpticalComm] Invalid frame received\n");
        return;
    }

    // 提取数据长度
    uint8_t data_len = rxData[2];
    if (data_len > MAX_DATA_SIZE) data_len = MAX_DATA_SIZE;

    // 拷贝有效数据
    memcpy(self._rx_buffer, &rxData[3], data_len);
    self._rx_len = data_len;
    self._new_data_flag = true;

    // 触发用户回调
    if (self._data_callback)
    {
        self._data_callback(self._rx_buffer, data_len);
    }
}