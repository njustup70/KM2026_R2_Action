#include "Get_Block.hpp"

GetBlock& APP::get_block = GetBlock::GetInstance();

// ==================== 超时间隔 ====================
static constexpr float T_BELT = 3.0f;
static constexpr float T_LIFT = 3.0f;
static constexpr float T_FEED = 2.0f;
static constexpr float BELT_RPM    = 15.0f;
static constexpr float DM_BELT_VEL = 15.0f;

// ==================== Start ====================
void GetBlock::Start()
{
    /**** 下摩擦带 DM (hcan_sub, ID 9) ****/
    motor_s2325.Init(Hardware::hcan_sub, 9, 19, VEL, DM_S2325);
    motor_s2325.SetFeedbackRange(3.14159f, 10.0f, 5.0f);
    motor_s2325.SetVelLimit(38);
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
    motor_dm_lift.Init(Hardware::hcan_sub, 10, 19, POSANDVEL, DM_S2325);
    motor_dm_lift.SetFeedbackRange(3.14159f, 10.0f, 5.0f);
    motor_dm_lift.SetPosLimit(0.5f, -0.1f);
    motor_dm_lift.SetVelLimit(5.0f);
    motor_dm_lift.Enable();

    /****  送块 2006 (hcan_main, ID 5)  ****/
    motor_2006_feed.Init(Hardware::hcan_main, 5, DJI_C610);
    motor_2006_feed.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(3.0e-5f, 0.18f)
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
        _sensor[i].Init(_pin_sensor[i]);

    /**** 初始位置 ****/
    motor_dm_lift.SetPosVel(_lift_pos_lower, _lift_vel);
    motor_2006_feed.SetPos(_feed_pos_retract);

    /**** 初始扫描 ****/
    _ReadSensors();
    _upper = _grid[1];
    _lower = _grid[0];
    APP::monit.LogInfo("[GetBlock] Init OK, upper=%d lower=%d", _upper, _lower);

    _fsm = FSM::IDLE;
    _fsm_busy = false;
}

// ==================== Update ====================
void GetBlock::Update()
{
    if (System.out_from_debugmode)
    {
        _StopBelt();
        motor_dm_lift.Disable();
        motor_2006_feed.driver.Disable();
        _fsm = FSM::IDLE;
        _fsm_busy = false;
        return;
    }

    _ReadSensors();
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
                // 块2已到位，托住上仓的块1 → 可以放抬升了
                _fsm = FSM::STORE_LIFT_DOWN;
                APP::monit.LogInfo("[GetBlock] Block2 at lower, lift descend");
            }
            else
            {
                // 块1已到，抬升到上仓
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

    // ═══════════════════ 存块: 抬升 ↑ ═══════════════════
    case FSM::STORE_LIFT_UP:
    {
        motor_dm_lift.SetPosVel(_lift_pos_upper, _lift_vel);

        if (_grid[1])  // 上仓检测到块（通过单向门）
        {
            _upper = true;
            _lower = false;

            _action_tick = System.runtime_tick;

            if (_storing_b2)
            {
                // 块2已到上仓
                _fsm = FSM::STORE_LIFT_DOWN;
                APP::monit.LogInfo("[GetBlock] Block2 at upper, lift down");
            }
            else
            {
                // ★ 块1到上仓 → 保持抬升↑，等待块2 ★
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

    // ═══════════════════ 存块: 等待块2 ═══════════════════
    case FSM::STORE_WAIT_SECOND:
    {
        // 抬升机构保持在上位
        motor_dm_lift.SetPosVel(_lift_pos_upper, _lift_vel);
        // 无超时 — 外部调用 Store() 会捕获此状态并转入 STORE_BELT_IN
        break;
    }

    // ═══════════════════ 存块: 抬升 ↓ (单向门打开穿过块2) ═══════════════════
    case FSM::STORE_LIFT_DOWN:
    {
        motor_dm_lift.SetPosVel(_lift_pos_lower, _lift_vel);

        if (_LiftNear(_lift_pos_lower))
        {
            _action_tick = System.runtime_tick;
            _fsm = FSM::STORE_LIFT_BOTTOM;
            APP::monit.LogInfo("[GetBlock] Lift at bottom, gate reset");
        }
        else if (_IsTimeout(T_LIFT))
        {
            APP::monit.LogError("[GetBlock] Lift down timeout");
            _fsm = FSM::ABORT;
        }
        break;
    }

    // ═══════════════════ 存块: 单向门复位 ═══════════════════
    case FSM::STORE_LIFT_BOTTOM:
    {
        // 单向门已机械复位，托住了下仓的块2
        if (_storing_b2)
        {
            // 块1在上仓，块2被门托住
            _lower = true;
            APP::monit.LogInfo("[GetBlock] Both stored: upper=1 lower=2");
        }
        else
        {
            // 块1存好，门复位，等待块2
            _lower = false;
            APP::monit.LogInfo("[GetBlock] Block1 stored, gate ready for block2");
        }
        _fsm = FSM::COMPLETE;
        break;
    }

    // ═══════════════════ 放块: 送块推出 ═══════════════════
    case FSM::PUT_FEED_PUSH:
    {
        motor_2006_feed.SetPos(_feed_pos_extend);

        if (_FeedNear(_feed_pos_extend))  // 送块到达头即认为成功
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
            if (_lower)
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
            if (_lower)
                _fsm = FSM::PUT_LIFT_UP;
            else
                _fsm = FSM::COMPLETE;
        }
        break;
    }

    // ═══════════════════ 放块: 补料抬升 ═══════════════════
    case FSM::PUT_LIFT_UP:
    {
        motor_dm_lift.SetPosVel(_lift_pos_upper, _lift_vel);

        if (_grid[1])  // 上仓检测到块
        {
            _upper = true;
            _lower = false;
            _action_tick = System.runtime_tick;
            _fsm = FSM::PUT_LIFT_DOWN;
            APP::monit.LogInfo("[GetBlock] Block lifted to upper");
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

        if (_LiftNear(_lift_pos_lower))
        {
            _fsm = FSM::COMPLETE;
            APP::monit.LogInfo("[GetBlock] Lift returned");
        }
        else if (_IsTimeout(T_LIFT))
        {
            _fsm = FSM::COMPLETE;
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

