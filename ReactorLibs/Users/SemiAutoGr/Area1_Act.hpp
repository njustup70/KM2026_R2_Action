#pragma once
#include "StateCore.hpp"
#include "PathChaser.hpp"
#include "Chassis.hpp"
#include <cstdint>

void FetchSpear_FromOrigin(uint8_t spear_id);
void FetchSpear_FromDock(uint8_t spear_id);
void GotoDock(uint8_t spear_id);