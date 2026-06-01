#pragma once
#include "System.hpp"
#include "FreeRTOS.h"
#include "queue.h"

// 后续有关通信的内容可放这里，方便管理用的

class CommCenter : public Application
{
    SINGLETON(CommCenter) : Application("GetBlock") {};

    APPLICATION_OVERRIDE

private:

public:
    static void SlamRxCallback(uint8_t func_code, const uint8_t *payload, uint8_t payload_len, void *user_ctx);
};

namespace APP
{
    extern CommCenter& comm_center;
}