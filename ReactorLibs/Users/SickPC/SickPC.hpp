#pragma once

#include "bsp_uart.hpp"
#include "System.hpp"

class SickPC : public Application
{
    SINGLETON(SickPC) : Application("sick_pc") {};
    APPLICATION_OVERRIDE
public:
};

namespace APP
{
    extern SickPC& sick_pc; 
}