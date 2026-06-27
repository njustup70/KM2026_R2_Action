#include "Get_Block.hpp"

GetBlock& APP::get_block = GetBlock::GetInstance();

// ==================== 超时间隔 ====================
static constexpr float T_BELT = 10.0f;
static constexpr float T_LIFT = 10.0f;
static constexpr float T_FEED = 10.0f;
static constexpr float BELT_RPM    = 15.0f;
static constexpr float DM_BELT_VEL = 15.0f;

// ==================== Start ====================
void GetBlock::Start()
{
    /**** 下摩擦带 DM (hcan_sub, ID 9) ****/
    motor_s2325.Init(Hardware::hcan_sub, 9, 19, VEL, DM_S2325);
    motor_s2325.SetFeedbackRange(3.14159f, 10.0f, 5.0f);
    motor_s2325.SetVelLimit(38);
    motor_s2325.SetAutoEnable(true, 1000); // user_want_enable 可禁止重使能
    motor_s2325.Enable();

    /**** 上摩擦带 2006 (hcan_main, ID 4) ****/
    motor_2006_fric.Init(Hardware::hcan_main, 4, DJI_C610);
    motor_2006_fric.ConfigADRC()
        .AsSpeedC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(1.5e-4f, 0.18f)
        .ADRC_Limit(8.0f)
        .SpdLimit(8000.0f)
        .ADRC_MaxPlannedVel(5000.0f)
        .ADRC_SOTF(0.2f)
        .Apply();
    motor_2006_fric.driver.Enable();

    /****  抬升 DM (hcan_sub, ID 10) ，向上为负，总行程 0 - 400 rad 左右****/
    motor_dm_lift.Init(Hardware::hcan_sub, 10, 20, POSANDVEL, DM_S2325);
    motor_dm_lift.SetFeedbackRange(3.14159f, 10.0f, 5.0f);
    motor_dm_lift.SetPosLimit(1500.0f, -1500.0f);
    motor_dm_lift.SetVelLimit(15.0f);
    motor_dm_lift.Enable();

    /****  送块 2006 (hcan_main, ID 5) ，送出去是负值，总行程 1780000 ****/
    motor_2006_feed.Init(Hardware::hcan_main, 5, DJI_C610);
    motor_2006_feed.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(1.0e-4f, 0.18f)
        .ADRC_Limit(8.0f)
        .SpdLimit(8000.0f)
        .ADRC_MaxPlannedVel(5000.0f)
        .ADRC_SOTF(0.6f)
        .Apply();
    motor_2006_feed.driver.Enable();

    /****  两个传感器（下仓、上仓） ****/
    _pin_sensor[0] = PIN_SENSOR_LOWER;
    _pin_sensor[1] = PIN_SENSOR_UPPER;
    for (int i = 0; i < 2; i++)
    {
        _sensor[i].Init(_pin_sensor[i]);
    }

    /****  负压泵 ****/
    pin_pump = PIN_PUMP;
    relay_pump.Init(pin_pump, Relay::HIGH_ON);
    

    /**** 初始位置 ****/
    motor_dm_lift.SetPosVel(_lift_pos_lower, _lift_vel);
    motor_2006_feed.SetPos(_feed_pos_retract);

    /**** 初始扫描 ****/
    _ReadSensors();
    _upper = _grid[1];
    _lower = _grid[0];

    _fsm = FSM::IDLE;
    _fsm_busy = false;
}

// ==================== Update ====================
void GetBlock::Update()
{
    if (System.out_from_debugmode)
    {
        _StopBelt();
        motor_2006_fric.driver.Disable();
        motor_dm_lift.Disable();
        motor_2006_feed.driver.Disable();
        _fsm = FSM::IDLE;
        _fsm_busy = false;
        return;
    }

    _ReadSensors();

    if(is_debug)
    {
        // motor_dm_lift.SetPosVel(-_lift_pos_upper, _lift_vel);
        // motor_2006_feed.SetPos(-_feed_pos_extend);
        // relay_pump.On();

    }
    else
    {
        relay_pump.Off();     
    }

    if(_belt_enable)
    {
        _StartBelt();
    }
    else
    {
        _StopBelt();
    }

    if(_store_debug)
    {
        _store_debug = false;
        Store();
    }
    if(_put_debug)
    {
        _put_debug = false;
        Put();
    }

    _UpdateFSM();
}

// ==================== 传感器 ====================
void GetBlock::_ReadSensors()
{
    // _sensor[0]=下仓  [1]=上仓
    _grid[0] = _sensor[0].Read();   // 下仓
    _grid[1] = _sensor[1].Read();   // 上仓
}

// ==================== 辅助 ====================
bool GetBlock::_IsTimeout(float sec) const
{
    return (System.runtime_tick - _action_tick) > sec;
}

void GetBlock::_StartBelt()
{
    motor_2006_fric.SetSpd(BELT_RPM * MotorDJIConst::redu_M2006 * 60.0f / (2 * PI));
    motor_s2325.SetVel(DM_BELT_VEL);
}

void GetBlock::_StopBelt()
{
    motor_2006_fric.SetSpd(0);
    motor_s2325.SetVel(0);
}

// ==================== 存块 ====================
void GetBlock::Store()
{
    // 在等待块2状态 → 直接入料块2
    if (_fsm_busy && _fsm == FSM::STORE_WAIT_SECOND)
    {
        _fsm = FSM::STORE_BELT_IN;
        _storing_b2 = true;
        _action_tick = System.runtime_tick;
        APP::monit.LogInfo("[GetBlock] Store #2: belt in");
        return;
    }

    // 正常排队
    if (_fsm_busy)
    {
        _store_pending = true;
        APP::monit.LogInfo("[GetBlock] Store queued");
        return;
    }

    _ReadSensors();

    if (!_upper && !_lower)
    {
        // 空仓 → 块1
        _fsm = FSM::STORE_BELT_IN;
        _storing_b2 = false;
        APP::monit.LogInfo("[GetBlock] Store #1: belt in");
    }
    else if (_upper && !_lower)
    {
        // 上仓有块、下仓空 → 块2
        _fsm = FSM::STORE_BELT_IN;
        _storing_b2 = true;
        APP::monit.LogInfo("[GetBlock] Store #2: belt in");
    }
    else if (_upper && _lower)
    {
        APP::monit.LogWarning("[GetBlock] Full");
        return;
    }
    else
    {
        // _lower && !_upper  异常恢复
        _fsm = FSM::STORE_LIFT_UP;
        _storing_b2 = false;
    }

    _fsm_busy = true;
    _action_tick = System.runtime_tick;
}

// ==================== 放块 ====================
void GetBlock::Put()
{
    if (_fsm_busy)
    {
        // 在存块流程中等待Put → 直接开始推出块1
        if (_fsm == FSM::STORE_HOLD_PUT)
        {
            _fsm = FSM::PUT_FEED_PUSH;
            _action_tick = System.runtime_tick;
            APP::monit.LogInfo("[GetBlock] Put during store hold: push block1");
            return;
        }

        _put_pending = true;
        APP::monit.LogInfo("[GetBlock] Put queued");
        return;
    }

    _ReadSensors();

    if (!_upper && !_lower)
    {
        APP::monit.LogWarning("[GetBlock] Empty, nothing to put");
        return;
    }

    if (_upper)
        _fsm = FSM::PUT_FEED_PUSH;
    else // _lower && !_upper
        _fsm = FSM::PUT_LIFT_UP;

    _fsm_busy = true;
    _action_tick = System.runtime_tick;
    APP::monit.LogInfo("[GetBlock] Put begin, upper=%d lower=%d", _upper, _lower);
}

// ==================== 内部状态机 ====================
void GetBlock::_UpdateFSM()
{
    if (!_fsm_busy) return;

    switch (_fsm)
    {
    // ═══════════════════ 存块: 入料 ═══════════════════
    case FSM::STORE_BELT_IN:
    {
        _StartBelt();

        if (_grid[0])  // 下仓检测到块
        {
            _StopBelt();
            _lower = true;
            _action_tick = System.runtime_tick;

            if (_storing_b2)
            {
                // 块2已到抬升平台，但挡片还托着上仓的块1→不能降
                // 保持上位，等Put()先推出块1
                _fsm = FSM::STORE_HOLD_PUT;
                APP::monit.LogInfo("[GetBlock] Block2 at lower, waiting for Put()");
            }
            else
            {
                // 块1已到，挡片托着块1上升→到上仓
                _fsm = FSM::STORE_LIFT_UP;
                APP::monit.LogInfo("[GetBlock] Block1 at lower, lift up");
            }
        }
        else if (_IsTimeout(T_BELT))
        {
            _StopBelt();
            APP::monit.LogError("[GetBlock] Belt timeout");
            _fsm = FSM::ABORT;
        }
        break;
    }

    // ═══════════════════ 存块: 抬升 ↑ (挡片托着块到上仓) ═══════════════════
    case FSM::STORE_LIFT_UP:
    {
        // 抬升达妙向上 rad 为负
        motor_dm_lift.SetPosVel(-_lift_pos_upper, _lift_vel);

        if (_grid[1])  // 上仓检测到块（被挡片托住触发）
        {
            _upper = true;
            _lower = false;

            _action_tick = System.runtime_tick;

            if (_storing_b2)
            {
                // 块2已到上仓 → 完成
                _fsm = FSM::COMPLETE;
                APP::monit.LogInfo("[GetBlock] Block2 stored at upper");
            }
            else
            {
                // 块1到上仓 → 保持抬升↑，等待块2 
                _fsm = FSM::STORE_WAIT_SECOND;
                APP::monit.LogInfo("[GetBlock] Block1 at upper, waiting for block2");
            }
        }
        else if (_IsTimeout(T_LIFT))
        {
            APP::monit.LogError("[GetBlock] Lift up timeout");
            _fsm = FSM::ABORT;
        }
        break;
    }

    // ═══════════════════ 存块: 等待块2 (挡片托着块1) ═══════════════════
    case FSM::STORE_WAIT_SECOND:
    {
        // 抬升机构保持在上位
        motor_dm_lift.SetPosVel(-_lift_pos_upper, _lift_vel);
        // 无超时 — 外部调用 Store() 会捕获此状态并转入 STORE_BELT_IN
        break;
    }

    // ═══════════════════ 存块: 等待Put推出块1 (挡片不能降) ═══════════════════
    case FSM::STORE_HOLD_PUT:
    {
        // 块2已在下仓，但挡片托着块1——不能降抬升(块1会掉)
        // 保持上位，等待外部 Put() 先推出块1，再降抬升处理块2
        motor_dm_lift.SetPosVel(-_lift_pos_upper, _lift_vel);
        // 无超时 — 外部调用 Put() 会捕获此状态并转入 PUT_FEED_PUSH
        break;
    }

    // ═══════════════════ 存块: 抬升 ↓ (挡片穿过块2) ═══════════════════
    case FSM::STORE_LIFT_DOWN:
    {
        // 下降时机：块1已被推出，块2在下仓
        // 挡片从上往下穿过块2→被块2顶开→通过后在块2下方闭合
        motor_dm_lift.SetPosVel(_lift_pos_lower, _lift_vel);

        if (_IsTimeout(T_LIFT))
        {
            _action_tick = System.runtime_tick;
            _fsm = FSM::STORE_LIFT_BOTTOM;
            APP::monit.LogInfo("[GetBlock] Lift at bottom, flap closed below block2");
        }
        break;
    }

    // ═══════════════════ 存块: 挡片在块2下方闭合 ═══════════════════
    case FSM::STORE_LIFT_BOTTOM:
    {
        // 抬升到最底，挡片已穿过块2并在其下方重新闭合
        // 现在挡片托住了块2（相当于块2在"下仓"）
        if (_storing_b2)
        {
            // 块1已推出，挡片托住块2 → 抬升↑带块2到上仓
            _lower = true;
            _fsm = FSM::STORE_LIFT_UP;
            APP::monit.LogInfo("[GetBlock] Block2 on flap, lift up");
        }
        else
        {
            // 块1存好，挡片复位，等待块2
            _lower = false;
            _fsm = FSM::COMPLETE;
            APP::monit.LogInfo("[GetBlock] Block1 stored, flap ready for block2");
        }
        break;
    }

    // ═══════════════════ 放块: 送块推出 ═══════════════════
    case FSM::PUT_FEED_PUSH:
    {
        motor_2006_feed.SetPos(-_feed_pos_extend);

        if (_FeedNear(-_feed_pos_extend))  // 送块到达头即认为成功
        {
            _upper = false;
            _action_tick = System.runtime_tick;
            _fsm = FSM::PUT_FEED_RETRACT;
            APP::monit.LogInfo("[GetBlock] Block ejected, feed retract");
        }
        else if (_IsTimeout(T_FEED))
        {
            _upper = false;
            APP::monit.LogWarning("[GetBlock] Push timeout, force retract");
            _fsm = FSM::PUT_FEED_RETRACT;
        }
        break;
    }

    // ═══════════════════ 放块: 送块收回 ═══════════════════
    case FSM::PUT_FEED_RETRACT:
    {
        motor_2006_feed.SetPos(_feed_pos_retract);

        if (_FeedNear(_feed_pos_retract))
        {
            if (_storing_b2)
            {
                // 块1已推出，挡片不再托块 → 可以下降穿过块2了
                _fsm = FSM::STORE_LIFT_DOWN;
                APP::monit.LogInfo("[GetBlock] Push done, lift descend through block2");
            }
            else if (_lower)
            {
                _fsm = FSM::PUT_LIFT_UP;
                APP::monit.LogInfo("[GetBlock] Lower has block, lift to upper");
            }
            else
            {
                _fsm = FSM::COMPLETE;
            }
            _action_tick = System.runtime_tick;
        }
        else if (_IsTimeout(T_FEED))
        {
            if (_storing_b2)
                _fsm = FSM::STORE_LIFT_DOWN;
            else if (_lower)
                _fsm = FSM::PUT_LIFT_UP;
            else
                _fsm = FSM::COMPLETE;
        }
        break;
    }

    // ═══════════════════ 放块: 补料抬升 ═══════════════════
    case FSM::PUT_LIFT_UP:
    {
        motor_dm_lift.SetPosVel(-_lift_pos_upper, _lift_vel);

        if (_grid[1])  // 上仓检测到块
        {
            _upper = true;
            _lower = false;
            _action_tick = System.runtime_tick;
            _fsm = FSM::COMPLETE;
            APP::monit.LogInfo("[GetBlock] Block lifted to upper, done");
        }
        else if (_IsTimeout(T_LIFT))
        {
            APP::monit.LogError("[GetBlock] Put lift up timeout");
            _fsm = FSM::ABORT;
        }
        break;
    }

    // ═══════════════════ 放块: 抬升回位 ═══════════════════
    case FSM::PUT_LIFT_DOWN:
    {
        motor_dm_lift.SetPosVel(_lift_pos_lower, _lift_vel);

        if (_IsTimeout(T_LIFT))
        {
            _action_tick = System.runtime_tick;
            if (_lower)
            {
                _fsm = FSM::PUT_LIFT_UP;
                APP::monit.LogInfo("[GetBlock] Lift at bottom, lift next block");
            }
            else
            {
                _fsm = FSM::COMPLETE;
                APP::monit.LogInfo("[GetBlock] Lift returned");
            }
        }
        break;
    }

    // ═══════════════════ 完成 ═══════════════════
    case FSM::COMPLETE:
    {
        APP::monit.LogInfo("[GetBlock] Done: upper=%d lower=%d",
                          _upper, _lower);
        _fsm = FSM::IDLE;
        _fsm_busy = false;

        if (_store_pending) { _store_pending = false; Store(); return; }
        if (_put_pending)   { _put_pending   = false; Put();   return; }
        break;
    }

    // ═══════════════════ 异常中止 ═══════════════════
    case FSM::ABORT:
    {
        _StopBelt();
        motor_dm_lift.SetPosVel(_lift_pos_lower, _lift_vel);
        motor_2006_feed.SetPos(_feed_pos_retract);
        _fsm = FSM::COMPLETE;
        break;
    }

    default: break;
    }
}

