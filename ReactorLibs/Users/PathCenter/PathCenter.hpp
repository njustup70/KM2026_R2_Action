#pragma once

#include "PathChaser.hpp"

namespace PATH
{
/// @brief 蓝区：矛台到对接点 TableToDock
const Path<16>& TTD_B6();
const Path<16>& TTD_B5();
const Path<16>& TTD_B4();
const Path<18>& TTD_B3();
const Path<21>& TTD_B2();
const Path<22>& TTD_B1();

/// @brief 蓝区：原点到矛台
const Path<44>& sttr_b1();
const Path<47>& sttr_b2();
const Path<54>& sttr_b3();
const Path<60>& sttr_b4();
const Path<68>& sttr_b5();
const Path<75>& sttr_b6();

/// @brief 跨区域路径
const Path<108>& A12_2();
const Path<147>& A12_3();
const Path<200>& A23_2_first();
}
