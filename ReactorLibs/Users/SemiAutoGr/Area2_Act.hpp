#pragma once
#include "StateCore.hpp"
#include "PathChaser.hpp"
#include "Chassis.hpp"
#include <cstdint>

/**
 * @brief 二区行为之 上200
 * @details 内部区分需不需要取块
 */
void Climb_Up200();

/**
 * @brief 二区行为之 上400
 * @details 内部区分需不需要取块
 */
void Climb_Up400();

void Decend_200();

void Decend_400();