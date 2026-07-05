#include "MainFrame.hpp"
#include "Monitor.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "Jaw.hpp"
#include "Get_Block.hpp"
#include "HostPC.hpp"
#include "Interboardcomm.hpp"
#include "farcon.hpp"
#include "Jaw.hpp"
// #include "ManualGraph.hpp"
// #include "AutoGraph.hpp"
#include "Lift_Leg.hpp"
#include "Jaw.hpp"
#include "ManualGraph.hpp"
#include "board.hpp"
#include "CommCenter.hpp"
#include "SickPC.hpp"
#include "optical_comm.hpp"
#include "VacuumClaw.hpp"
#include "SemiAutoGr.hpp"
#include "ChallengeGraph.hpp"
#include "ChallengeGraphBlue.hpp"

#include "MainLoop.hpp"
#include "Area2_Act.hpp"


StateGraph graph("LoopGraph");
void Response_Button_main();

// void SlamRxCallback(uint8_t func_code, const uint8_t *payload, uint8_t payload_len, void *user_ctx);

// void Config_StateGraph();

void Action_of_Dege(StateCore *core)
{
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button_main();
    Seq::Wait(0.01f);
  }

  // 抬升先上去
  APP::get_block.motor_dm_lift.SetPosVel(-750, 15);

  // 先开爪
  APP::vac_claw.OpenSnap();
  Seq::Wait(0.4f);

  // 再伸出
  APP::vac_claw.StretchOut();

  // 等待伸出完成后，闭上爪
  Seq::Wait(1.5f);
  APP::vac_claw.CloseSnap();

  // 等爪子抓住后
  Seq::Wait(2.5f);

  // 打开负压泵
  APP::get_block.OpenPump();

  // 等待吸住后，翻转物块，收爪
  Seq::Wait(1.0f);
  APP::vac_claw.StretchBack();

  // 此时爪还抓着物块，关闭负压泵
  APP::get_block.ClosePump();

  APP::get_block._StartBelt();

  APP::vac_claw.OpenSnap();

  APP::get_block.OpenPRV();

  APP::get_block.Store();

  // Seq::WaitUntil([]()
  // {
  //   return APP::get_block._sensor[0].Read();
  // });

  // Seq::Wait(0.5f);

  // APP::get_block.motor_dm_lift.SetPosVel(-5.0f, 15);

  // Seq::Wait(3.0f);

  // APP::get_block.motor_dm_lift.SetPosVel(-750.0f, 15);

  // APP::get_block._StopBelt();


  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button_main();
    Seq::Wait(0.01f);
  }

  // APP::get_block.Store();

  core->GetCurState().Complete = true;
}

void Action_of_Dege_Fetch_UP200(StateCore *core)
{
  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button_main();
    Seq::Wait(0.01f);
  }  
  
  Area2::mei_lin_array.FetchBlock(BlockHeight::DOWN200);
  

  // Area2::block_pos.GetIn(1);
  // Area2::mei_lin_array.Climb_Up200();

  while(MOD::farcon.button_second_half[0] != 1)
  {
    Response_Button_main();
    Seq::Wait(0.01f);
  }  
  // Area2::mei_lin_array.FetchBlock(BlockHeight::UP200);

  // Area2::mei_lin_array.Climb_Up200();

  // while(MOD::farcon.button_second_half[0] != 1)
  // {
  //   Response_Button_main();
  //   Seq::Wait(0.01f);
  // }   
  
  // Area2::mei_lin_array.Climb_Up200();

  // while(MOD::farcon.button_second_half[0] != 1)
  // {
  //   Response_Button_main();
  //   Seq::Wait(0.01f);
  // } 

  // Area2::mei_lin_array.FetchBlock(BlockHeight::UP200);  

  // Area2::mei_lin_array.Climb_Up200();

  // // while(MOD::farcon.button_second_half[0] != 1)
  // // {
  // //   Response_Button_main();
  // //   Seq::Wait(0.01f);
  // // }   

  // Area2::mei_lin_array.Decend_200();

  // // Area2::mei_lin_array.FetchBlock(BlockHeight::DOWN200);

  // // while(MOD::farcon.button_second_half[0] != 1)
  // // {
  // //   Response_Button_main();
  // //   Seq::Wait(0.01f);
  // // }  
  
  // Area2::mei_lin_array.Decend_400();

  // while(MOD::farcon.button_second_half[0] != 1)
  // {
  //   Response_Button_main();
  //   Seq::Wait(0.01f);
  // }    

  // APP::get_block.Put();

  // while(MOD::farcon.button_second_half[0] != 1)
  // {
  //   Response_Button_main();
  //   Seq::Wait(0.01f);
  // }    

  // APP::get_block.Put();  

    core->GetCurState().Complete = true;
}

/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
  // 初始化板间通信
  MOD::board_can.Init(Hardware::hcan_sub, 0x240, false);
  // 初始化工控机通信
  MOD::host_pc.Init(Hardware::huart_host);
  // 初始化遥控器
  MOD::farcon.init(Hardware::huart_farcon);

  System.SetPositionSource(APP::comm_center.slam_pos);

  APP::monit.Track(System.position.x);

  System.RegistApp(APP::main_loop);

  // 启动通信中心
  System.RegistApp(APP::comm_center);

  // 启动矛头控制应用
  System.RegistApp(APP::jaw);

  // 底盘启动
  System.RegistApp(APP::chas);
  APP::chas.Enable();

  // 启动伸缩腿控制
  System.RegistApp(APP::leg);
  APP::leg.Enable();

  // 启动路径跟踪
  System.RegistApp(APP::path_chaser);

  System.RegistApp(APP::get_block);

  System.RegistApp(APP::vac_claw);
  APP::vac_claw.Enable();
  // System.RegistApp(APP::sick_pc);

  Area2::mei_lin_array.Init();

  // Config_AutoStateGraph();
  // Config_SemiAutoStateGraph();
  // Config_R2_Challenge_StateGraph();
  Config_R2_Challenge_Blue_StateGraph();

  /*简并模式测试*/
  // graph.Degenerate(Action_of_Dege_Fetch_UP200);
  // APP::state_core.RegistGraph(graph);
  // APP::state_core.Enable(0); // 启动状态机核心，指定初始状态图为0号图

}



// const static float max_farcon_alpha = 1.8f; // 最大加速度 m/s^2
// const static float max_farcon_beta = 1.8f; // 最大角加速度 rad/s^2
/***********    一些不通用的回调函数    ***********/
// void Response_Button_main()
// {
//     static float max_velo = 1.5f;
//     if(MOD::farcon.toggle[2] == 0)
//     {
//       max_velo = 1.5f;
//     }
//     else if(MOD::farcon.toggle[2] == 1)
//     {
//       max_velo = 0.5f;
//     }

//     // 计算遥控器死区：0-15不算，将15-100映射到原来的0.0-1.0
//     constexpr float deadzone = 15.0f;
//     auto map_axis = [deadzone](int v) -> float {
//       float a = (v >= 0) ? float(v) : -float(v);
//       if (a <= deadzone) return 0.0f;
//       return (v > 0 ? 1.0f : -1.0f) * ((a - deadzone) / (100.0f - deadzone));
//     };

//     // 计算目标速度（摇杆直接映射到的期望速度）
//     Vec3 target_vel;
//     target_vel.x = -map_axis(MOD::farcon.jys_value[3]) * max_velo;
//     target_vel.y = -map_axis(MOD::farcon.jys_value[2]) * max_velo;
//     target_vel.z = -map_axis(MOD::farcon.jys_value[0]) * max_velo;

//     // 加速度限制：平滑过渡到目标速度
//     static Vec3 last_vel(0, 0, 0);
//     static float last_accel_time = DWT_GetTimeline_Sec();

//     float now = DWT_GetTimeline_Sec();
//     float dt = now - last_accel_time;
//     // 限制最大 dt，防止长时间阻塞后速度跳变
//     if (dt > 0.05f) dt = 0.05f;
//     if (dt < 0.0f)  dt = 0.0f;  // 防止时间回绕
//     last_accel_time = now;

//     // 限幅函数
//     auto limit_delta = [](float delta, float max_delta) -> float {
//         if (delta > max_delta) return max_delta;
//         if (delta < -max_delta) return -max_delta;
//         return delta;
//     };

//     Vec3 vel;
//     vel.x = last_vel.x + limit_delta(target_vel.x - last_vel.x, max_farcon_alpha * dt);
//     vel.y = last_vel.y + limit_delta(target_vel.y - last_vel.y, max_farcon_alpha * dt);
//     vel.z = last_vel.z + limit_delta(target_vel.z - last_vel.z, max_farcon_beta  * dt);
//     last_vel = vel;

//     if (vel.Length() > 0.03f)
//     {
//       // 向底盘发送移动指令
//       APP::chas.Move(vel, 0.2f);
//     }
//     else
//     {
//       // 如果摇杆没有动作，则发送零速度指令
//       APP::chas.Move(Vec3(0, 0, 0), 0.2f);
//     }

//     static float last_trigger_time = 0;
//     if(MOD::farcon.button_second_half[7] == 1 && DWT_GetTimeline_Sec() - last_trigger_time > 1.0f)
//     {
//       static bool is_daji_enable = false;
//       if(is_daji_enable)
//       {
//         // 打开后边小轮(妲己)
//         APP::leg.relay_daji.Off();
//       }
//       else
//       {
//         APP::leg.relay_daji.On();
//       }
 
//       is_daji_enable = !is_daji_enable;
//       last_trigger_time = DWT_GetTimeline_Sec();
//     }

//     // 处理泄压阀的开关
//     static float last_prv_time = 0;
//     if(MOD::farcon.button_second_half[1] == 1 && DWT_GetTimeline_Sec() - last_prv_time > 1.0f)
//     {
//       static bool is_PRV_open = false;
//       if(!is_PRV_open)
//       {
//         APP::get_block.OpenPRV();
//         is_PRV_open = true;
//       }
//       else
//       {
//         APP::get_block.ClosePRV();
//         is_PRV_open = false;
//       }
//       last_prv_time = DWT_GetTimeline_Sec();
//     }

//     // 处理抬升平台的升降
//     static float last_lift_time = 0;
//     if(MOD::farcon.button_second_half[3] == 1 && DWT_GetTimeline_Sec() - last_lift_time > 1.0f)
//     {
//       static bool is_lift_up = false;
//       if(!is_lift_up)
//       {
//         APP::get_block.motor_dm_lift.SetPosVel(-760, 15);
//         is_lift_up = true;
//       }
//       else
//       {
//         APP::get_block.motor_dm_lift.SetPosVel(-10, 15);
//         is_lift_up = false;
//       }
//       last_lift_time = DWT_GetTimeline_Sec();
//     }

//     // 处理摩擦带的启停
//     static float last_belt_time = 0;
//     if(MOD::farcon.button_second_half[2] == 1 && DWT_GetTimeline_Sec() - last_belt_time > 1.0f)
//     {
//       static bool is_belt_on = false;
//       if(!is_belt_on)
//       {
//         APP::get_block.motor_2006_fric.SetSpd(15.0f * MotorDJIConst::redu_M2006 * 60.0f / (2 * PI));
//         APP::get_block.motor_s2325.SetVel(15.0f);
//         is_belt_on = true;
//       }
//       else
//       {
//         APP::get_block.motor_2006_fric.SetSpd(0);
//         APP::get_block.motor_s2325.SetVel(0);
//         is_belt_on = false;
//       }
//       last_belt_time = DWT_GetTimeline_Sec();
//     }

//     // 处理放块
//     static float last_put_time = 0;
//     if(MOD::farcon.button_second_half[4] == 1 && DWT_GetTimeline_Sec() - last_put_time > 1.0f)
//     {
//       static bool is_put = false;
//       if(!is_put)
//       {
//         APP::get_block.motor_2006_feed.SetPos(-1780000.0f);
//         is_put = true;
//       }
//       else
//       {
//         APP::get_block.motor_2006_feed.SetPos(-10000.0f);
//         is_put = false;
//       }
//       last_put_time = DWT_GetTimeline_Sec();
//     }    

//     // 同时放低前后腿，准备登R1
//     static float last_taitui_time = 0;
//     if(MOD::farcon.button_first_half[0] == 1 && DWT_GetTimeline_Sec() - last_taitui_time > 1.0f)
//     {
//       APP::leg.SetFrontPos(-400.0f);
//       APP::leg.SetBackPos(-400.0f);
//       last_taitui_time = DWT_GetTimeline_Sec();
//     }

//     static float last_shouqiantui_time = 0;
//     if(MOD::farcon.button_first_half[1] == 1 && DWT_GetTimeline_Sec() - last_shouqiantui_time > 1.0f)
//     {
//       APP::leg.SetFrontPos(-10.0f);
//       // APP::leg.SetBackPos(-380.0f);
//       last_shouqiantui_time = DWT_GetTimeline_Sec();
//     }  

//     static float last_shouhoutui_time = 0;
//     if(MOD::farcon.button_first_half[2] == 1 && DWT_GetTimeline_Sec() - last_shouhoutui_time > 1.0f)
//     {
//       // APP::leg.SetFrontPos(-5.0f);
//       APP::leg.SetBackPos(-10.0f);
//       last_shouhoutui_time = DWT_GetTimeline_Sec();
//     }

//     if(MOD::farcon.button_first_half[6] == 1)
//     {
//       APP::vac_claw.OpenSnap();
//     }

//     if(MOD::farcon.button_first_half[7] == 1)
//     {
//       APP::vac_claw.CloseSnap();
//     }

//     // 处理梅林的上下
//     if(MOD::farcon.button_middle[0][0] == 1) // KFS0
//     {
//       Area2::mei_lin_array.Climb_Up200();
//     }
//     if(MOD::farcon.button_middle[0][1] == 1) // KFS1
//     {
//       Area2::mei_lin_array.Decend_200();
//     }
//     if(MOD::farcon.button_middle[0][2] == 1) // KFS2
//     {
//       Area2::mei_lin_array.Climb_Up400();
//     }
//     if(MOD::farcon.button_middle[1][0] == 1) // KFS3
//     {
//       Area2::mei_lin_array.Decend_400();
//     }
//     if(MOD::farcon.button_middle[1][1])
//     {
//       Area2::mei_lin_array.TurnLeft();
//     }
//     if(MOD::farcon.button_middle[1][2])
//     {
//       Area2::mei_lin_array.TurnRight();
//     }

//     // 进行梅林上的位置闭环
//     if(MOD::farcon.button_first_half[4] == 1)
//     {
//       Area2::mei_lin_array.ChaseCurrentGrid();
//     }
// }
