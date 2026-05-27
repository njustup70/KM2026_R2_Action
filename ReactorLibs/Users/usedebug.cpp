#include "HostPC.hpp"
#include "Farcon.hpp"
#include "Jaw.hpp"
#include "Lift_Leg.hpp"
#include "Chassis.hpp"
#include "Get_Block.hpp"
#include "SickPC.hpp"

HostPC& host_pc_debug = HostPC::GetInstance();
Farcon& farcon_debug = Farcon::GetInstance();
Jaw& jaw_debug = Jaw::GetInstance();
Lift_Leg& leg_debug = Lift_Leg::GetInstance();
ChassisType& chas_debug = ChassisType::GetInstance();
GetBlock& get_block_debug = GetBlock::GetInstance();
SickPC& sick_pc_debug = SickPC::GetInstance();