#pragma once
#include "StateCore.hpp"
#include "PathChaser.hpp"
#include "Chassis.hpp"
#include <cstdint>

void FetchSpear_FromOrigin(uint8_t spear_id);
void FetchSpear_FromDock(uint8_t spear_id);
void GotoDock(uint8_t spear_id);

/**
 * @brief 蓝场的版本
 * 
 * @param spear_id 
 */
 
void FetchSpear_FromOrigin_Blue(uint8_t spear_id);
void FetchSpear_FromDock_Blue(uint8_t spear_id);
void GotoDock_Blue(uint8_t spear_id);