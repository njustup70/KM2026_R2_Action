/**
 * @file IR_RX.cpp
 * @brief 红外接收模块实现
 */

#include "IR_RX.hpp"
#include "bsp_log.hpp"
#include "string.h"

// ==================== 静态路由表 ====================
IR_RX::ExtiRoute IR_RX::_exti_routes[IRRX_MAX_INSTANCES] = {};
uint8_t IR_RX::_exti_route_count = 0;

// ==================== 构造 / 析构 ====================

IR_RX::IR_RX() {}

IR_RX::~IR_RX()
{
    for (uint8_t i = 0; i < _exti_route_count; i++)
    {
        if (_exti_routes[i].instance == this)
        {
            _exti_routes[i].used = false;
            for (uint8_t j = i; j < _exti_route_count - 1; j++)
                _exti_routes[j] = _exti_routes[j + 1];
            memset(&_exti_routes[_exti_route_count - 1], 0, sizeof(ExtiRoute));
            _exti_route_count--;
            break;
        }
    }
}

// ==================== 初始化 ====================

void IR_RX::Init(Pin rx_pin)
{
    _rx_pin = rx_pin;

    _rx_gpio = BSP::GPIO::Inst(rx_pin);
    if (!_rx_gpio.IsValid())
    {
        BspLog_LogError("[IR_RX] Init failed: invalid pin %c%d",
                        rx_pin.port, rx_pin.number);
        _initialized = false;
        return;
    }

    // 注册到静态路由表
    if (_exti_route_count < IRRX_MAX_INSTANCES)
    {
        _exti_routes[_exti_route_count].pin      = rx_pin;
        _exti_routes[_exti_route_count].instance = this;
        _exti_routes[_exti_route_count].used     = true;
        _exti_route_count++;

        if (_exti_route_count == 1)
            _rx_gpio.RegisterExtiHandler(_ExtiRouter);
    }
    else
    {
        BspLog_LogError("[IR_RX] Init failed: route table full (max %d)",
                        IRRX_MAX_INSTANCES);
        _initialized = false;
        return;
    }

    memset(&_rx, 0, sizeof(_rx));
    _rx.state        = RxState::Idle;
    _rx.last_edge_us = DWT_GetTimeline_USec();

    _initialized = true;
    BspLog_LogInfo("[IR_RX] Init OK, pin=%c%d, addr=0x%02X",
                   rx_pin.port, rx_pin.number, local_addr);
}

// ==================== 回调注册 ====================

void IR_RX::RegisterRxHandler(IRRxHandler cb, void *user_ctx)
{
    _rx_handler.cb       = cb;
    _rx_handler.user_ctx = user_ctx;
}

// ==================== EXTI 路由 ====================

void IR_RX::_ExtiRouter(Pin pin)
{
    for (uint8_t i = 0; i < _exti_route_count; i++)
    {
        if (_exti_routes[i].used &&
            _exti_routes[i].pin.port   == pin.port &&
            _exti_routes[i].pin.number == pin.number)
        {
            IR_RX *inst = _exti_routes[i].instance;
            if (inst && inst->_initialized)
            {
                uint64_t now_us = DWT_GetTimeline_USec();
                uint32_t dur    = (uint32_t)(now_us - inst->_rx.last_edge_us);
                bool     rising = inst->_rx_gpio.Read();
                inst->_rx.last_edge_us = now_us;
                inst->_OnExtiEvent(dur, rising);
            }
            break;
        }
    }
}

// ==================== 时序匹配 ====================

bool IR_RX::_TimingInRange(uint32_t duration, uint32_t target) const
{
    if (target == 0) return (duration < 100);
    uint32_t lower = (uint32_t)((float)target * (1.0f - timing_tolerance));
    uint32_t upper = (uint32_t)((float)target * (1.0f + timing_tolerance));
    return (duration >= lower && duration <= upper);
}

// ==================== NEC 解码状态机 ====================

void IR_RX::_OnExtiEvent(uint32_t duration_us, bool is_rising)
{
    switch (_rx.state)
    {
    case RxState::Idle:
        if (!is_rising)
            _rx.state = RxState::LeaderOn;
        break;

    case RxState::LeaderOn:
        if (is_rising)
        {
            if (_TimingInRange(duration_us, leader_on_us))
                _rx.state = RxState::LeaderOff;
            else
                _rx.state = RxState::Idle;
        }
        break;

    case RxState::LeaderOff:
        if (!is_rising)
        {
            if (_TimingInRange(duration_us, leader_off_us))
            {
                _rx.data      = 0;
                _rx.bit_count = 0;
                _rx.state     = RxState::BitOn;
            }
            else if (_TimingInRange(duration_us, repeat_off_us))
                _rx.state = RxState::RepeatWait;
            else
                _rx.state = RxState::Idle;
        }
        break;

    case RxState::BitOn:
        if (is_rising)
        {
            if (_TimingInRange(duration_us, bit_on_us))
                _rx.state = RxState::BitOff;
            else
                _rx.state = RxState::Idle;
        }
        break;

    case RxState::BitOff:
        if (!is_rising)
        {
            uint8_t bit;
            if (_TimingInRange(duration_us, bit_0_off_us))
                bit = 0;
            else if (_TimingInRange(duration_us, bit_1_off_us))
                bit = 1;
            else
            {
                _rx.state = RxState::Idle;
                break;
            }

            _rx.data |= ((uint32_t)bit << _rx.bit_count);
            _rx.bit_count++;

            if (_rx.bit_count >= 32)
            {
                uint8_t addr  =  _rx.data        & 0xFF;
                uint8_t n_addr = (_rx.data >> 8)  & 0xFF;
                uint8_t cmd   = (_rx.data >> 16)  & 0xFF;
                uint8_t n_cmd  = (_rx.data >> 24) & 0xFF;

                if (addr == (uint8_t)~n_addr && cmd == (uint8_t)~n_cmd)
                {
                    _rx.address = addr;
                    _rx.command = cmd;

                    if (local_addr == 0xFF || addr == local_addr)
                    {
                        if (_rx_handler.cb)
                            _rx_handler.cb(this, addr, cmd, false, _rx_handler.user_ctx);
                    }
                }
                _rx.state = RxState::Idle;
            }
            else
            {
                _rx.state = RxState::BitOn;
            }
        }
        break;

    case RxState::RepeatWait:
        if (is_rising)
        {
            if (_TimingInRange(duration_us, bit_on_us))
            {
                if (_rx_handler.cb &&
                    (local_addr == 0xFF || _rx.address == local_addr))
                {
                    _rx_handler.cb(this, _rx.address, _rx.command, true, _rx_handler.user_ctx);
                }
            }
            _rx.state = RxState::Idle;
        }
        break;
    }
}

// ==================== 周期处理 ====================

void IR_RX::Handle()
{
    if (!_initialized) return;

    if (_rx.state != RxState::Idle)
    {
        uint64_t elapsed = DWT_GetTimeline_USec() - _rx.last_edge_us;
        uint32_t max_frame_us = leader_on_us + leader_off_us +
                                32 * (bit_on_us + bit_1_off_us) + 10000;
        if (elapsed > max_frame_us)
            _rx.state = RxState::Idle;
    }
}
