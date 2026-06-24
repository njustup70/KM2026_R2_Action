#include "HostPC.hpp"
#include "Farcon.hpp"
#include "Jaw.hpp"
#include "Lift_Leg.hpp"
#include "Chassis.hpp"
#include "Get_Block.hpp"
#include "PathChaser.hpp"
#include "CommCenter.hpp"
#include "StateCore.hpp"
#include "SickPC.hpp"
#include "InterBoardComm.hpp"

HostPC& host_pc_debug = HostPC::GetInstance();
Farcon& farcon_debug = Farcon::GetInstance();
Jaw& jaw_debug = Jaw::GetInstance();
Lift_Leg& leg_debug = Lift_Leg::GetInstance();
ChassisType& chas_debug = ChassisType::GetInstance();
GetBlock& get_block_debug = GetBlock::GetInstance();
SickPC& sick_pc_debug = SickPC::GetInstance();
PathChaserType& path_chaser_debug = PathChaserType::GetInstance();
CommCenter& comm_center_debug = CommCenter::GetInstance();
StateCore& state_core_debug = StateCore::GetInstance();
BoardComm& board_can_debug = BoardComm::GetInstance();