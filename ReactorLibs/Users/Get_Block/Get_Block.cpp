#include "Get_Block.hpp"

GetBlock& APP::get_block = GetBlock::GetInstance();

// ==================== 超时间隔 ====================
static constexpr float T_BELT = 20.0f;
static constexpr float T_LIFT = 3.2f; // 摩擦带上下的时间
static constexpr float T_LIFT_SETTLE = 3.0f;    // 传感器触发后等待抬升到位的短超时
static constexpr float T_LIFT_BEFORE_BELT = 4.2f; // 取块1时抬升先走一段再转摩擦带
static constexpr float T_FEED = 10.0f;
static constexpr float BELT_RPM    = 17.0f;
static constexpr float DM_BELT_VEL = 17.0f;

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
        .SetDt(0.002)
        .Apply();
    motor_2006_fric.driver.Enable();
    

    /****  抬升 DM (hcan_sub, ID 10) ，向上为负，总行程 0 - 400 rad 左右****/
    motor_dm_lift.Init(Hardware::hcan_sub, 10, 20, POSANDVEL, DM_S2325);
    motor_dm_lift.SetFeedbackRange(3.14159f, 10.0f, 5.0f);
    motor_dm_lift.SetPosLimit(1500.0f, -1500.0f);
    motor_dm_lift.SetVelLimit(15.0f);
    motor_s2325.SetAutoEnable(true, 1000); // user_want_enable 可禁止重使能
    motor_dm_lift.Enable();

    /****  送块 2006 (hcan_main, ID 5) ，送出去是负值，总行程 1780000 ****/
    motor_2006_feed.Init(Hardware::hcan_main, 5, DJI_C610);
    motor_2006_feed.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(1.0e-4f, 0.18f)
        .ADRC_Limit(8.0f)
        .SpdLimit(8000.0f)
        .ADRC_MaxPlannedVel(6800.0f)
        .ADRC_SOTF(0.05f)
        .SetDt(0.002)
        .Apply();
    motor_2006_feed.driver.Enable();

    /****  两个传感器（下仓、上仓） ****/
    _sensor[0].Init(PIN_SENSOR_LOWER);
    _sensor[1].Init(PIN_SENSOR_UPPER);

    /****  九宫格小黄  ****/
    detector.Init(PIN_ARENA_DETECT);

    /****  负压泵 ****/
    pin_pump = PIN_PUMP;
    relay_pump.Init(pin_pump, Relay::HIGH_ON);

    /*  泄压阀  */
    pin_PRV = PIN_PRESSURE_RELIEF_VALVE;
    relay_PRV.Init(pin_PRV, Relay::HIGH_ON);
    relay_PRV.Off(); // 初始状态下关闭泄压阀

    /**** 初始位置 ****/
    dm_fric_spd = 0.0f;
    dm_lift_pos = -_lift_pos_lower;
    motor_2006_feed.SetPos(_feed_pos_retract);

    /**** 初始扫描 ****/
    _ReadSensors();
    _upper = _grid[1];
    _lower = _grid[0];

    _fsm = FSM::IDLE;
    _fsm_busy = false;

    // 初始化本地计时器
    DWT_GetDeltaTime(&_local_cyccnt);
    _local_tick = 0.0f;
}

// ==================== Update ====================
void GetBlock::Update()
{
    // 累积本地独立计时器（方案C：不依赖全局 System.runtime_tick）
    _local_tick += DWT_GetDeltaTime(&_local_cyccnt);

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

    static int damiao_cnt = 0;
    damiao_cnt++;
    if (damiao_cnt >= 4)
    {
        motor_s2325.SetVel(dm_fric_spd);
        motor_dm_lift.SetPosVel(dm_lift_pos, _lift_vel);
        damiao_cnt = 0;
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
    // return (System.runtime_tick - _action_tick) > sec;  // old
    return (_local_tick - _action_tick) > sec;
}

void GetBlock::_StartBelt()
{
    motor_2006_fric.SetSpd(BELT_RPM * MotorDJIConst::redu_M2006 * 60.0f / (2 * PI));
    dm_fric_spd = DM_BELT_VEL;
}

void GetBlock::_StopBelt()
{
    motor_2006_fric.SetSpd(0);
    dm_fric_spd = 0.0f;
}

void GetBlock::LiftUp()
{
    dm_lift_pos = -_lift_pos_upper;
}

void GetBlock::LiftDown()
{
    dm_lift_pos = -_lift_pos_lower;
}

void GetBlock::Push()
{
    motor_2006_feed.SetPos(-_feed_pos_extend);
}

void GetBlock::PushBack()
{
    motor_2006_feed.SetPos(_feed_pos_retract);
}

// ==================== 存块 ====================
void GetBlock::Store()
{
    // 在等待块2状态 → 直接入料块2
    if (_fsm_busy && _fsm == FSM::STORE_WAIT_SECOND)
    {
        _fsm = FSM::STORE_BELT_IN;
        _storing_b2 = true;
        // _action_tick = System.runtime_tick;  // old
        _action_tick = _local_tick;
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
        // 空仓 → 块1（先抬升到高位，再转摩擦带）
        _fsm = FSM::STORE_LIFT_BEFORE_BELT;
        _storing_b2 = false;
        APP::monit.LogInfo("[GetBlock] Store #1: lift up then belt in");
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
    // _action_tick = System.runtime_tick;  // old
    _action_tick = _local_tick;
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
            // _action_tick = System.runtime_tick;  // old
            _action_tick = _local_tick;
            APP::monit.LogInfo("[GetBlock] Put during store hold: push block1");
            return;
        }

        // 在等待块2时Put → 推出块1，取消存块2流程
        if (_fsm == FSM::STORE_WAIT_SECOND)
        {
            _fsm = FSM::PUT_FEED_PUSH;
            _storing_b2 = false;
            // _action_tick = System.runtime_tick;  // old
            _action_tick = _local_tick;
            APP::monit.LogInfo("[GetBlock] Put during wait second: push block1, cancel block2");
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
    // _action_tick = System.runtime_tick;  // old
    _action_tick = _local_tick;
    APP::monit.LogInfo("[GetBlock] Put begin, upper=%d lower=%d", _upper, _lower);
}

void GetBlock::OpenPump()
{
    relay_pump.On();
};

void GetBlock::ClosePump()
{
    relay_pump.Off();
}

void GetBlock::OpenPRV()
{
    relay_PRV.On();
}

void GetBlock::ClosePRV()
{
    relay_PRV.Off();
}

bool GetBlock::IsBlockPresent()
{
    // static int detect_cnt = 0;
    // if(detector.Read())
    // {
    //     detect_cnt++;
    //     if(detect_cnt > 5)
    //     {
    //         detect_cnt = 0;
    //         return true;
    //     }
    // }
    // else
    // {
    //     return false;
    // }
    // return false;
    return detector.Read();
}

// ==================== 内部状态机 ====================
void GetBlock::_UpdateFSM()
{
    if (!_fsm_busy) return;

    // ===== 状态变化日志 =====
    static FSM _last_fsm = FSM::IDLE;
    if (_fsm != _last_fsm)
    {
        _last_fsm = _fsm;
        static const char* _state_name[] = {
            "IDLE",                // 0
            "STORE_LIFT_BEFORE_BELT", // 1
            "STORE_LIFT_DOWN_TO_PICK",// 2
            "STORE_BELT_IN",       // 3
            "STORE_LIFT_UP",       // 4
            "STORE_WAIT_SECOND",   // 5
            "STORE_HOLD_PUT",      // 6
            "STORE_LIFT_DOWN",     // 7
            "STORE_LIFT_BOTTOM",   // 8
            "PUT_FEED_PUSH",       // 9
            "PUT_FEED_RETRACT",    // 10
            "PUT_LIFT_UP",         // 11
            "PUT_LIFT_DOWN",       // 12
            "COMPLETE",            // 13
            "ABORT",               // 14
        };
        int idx = (int)_fsm;
        if (idx >= 0 && idx < 15)
            APP::monit.LogInfo("[GetBlock] FSM >>> %s", _state_name[idx]);
    }

    switch (_fsm)
    {
    // ═══════════════════ 存块: 入料 ═══════════════════
    case FSM::STORE_BELT_IN:
    {
        _StartBelt();

        if (_grid[0])  // 下仓检测到块
        {
            APP::monit.LogInfo("[GetBlock] Block is inside");

            _StopBelt();
            _lower = true;
            ClosePRV();

            if (_storing_b2)
            {
                // 块2已到抬升平台，但挡片还托着上仓的块1→不能降
                // 保持上位，等Put()先推出块1
                // _action_tick = System.runtime_tick;  // old
                _action_tick = _local_tick;
                _fsm = FSM::STORE_HOLD_PUT;
                APP::monit.LogInfo("[GetBlock] Block2 at lower, waiting for Put()");
            }
            else if (_IsTimeout(T_LIFT))
            {
                // 块1已到且抬升已到高位 → 降下来接块
                // _action_tick = System.runtime_tick;  // old
                _action_tick = _local_tick;
                _fsm = FSM::STORE_LIFT_DOWN_TO_PICK;
                APP::monit.LogInfo("[GetBlock] Block1 at lower & lift at top, descending to pick");
            }
            // else: 块已到下仓但抬升未到顶 → 等待（不重置 _action_tick）
        }
        else if (_IsTimeout(T_BELT))
        {
            _StopBelt();
            APP::monit.LogError("[GetBlock] Belt timeout");
            _fsm = FSM::ABORT;
        }
        break;
    }

    // ═══════════════════ 存块: 抬升到高位 + 等一会再入料 ═══════════════════
    case FSM::STORE_LIFT_BEFORE_BELT:
    {
        dm_lift_pos = -_lift_pos_upper;

        // 抬升先走一段，再启动摩擦带
        if (_IsTimeout(T_LIFT_BEFORE_BELT))
        {
            _StartBelt();
        }

        // 下仓检测到块 && 抬升超时（到高位）→ 降下来接块
        if (_grid[0] && _IsTimeout(T_LIFT))
        {
            _StopBelt();
            _lower = true;
            // _action_tick = System.runtime_tick;  // old
            _action_tick = _local_tick;
            _fsm = FSM::STORE_LIFT_DOWN_TO_PICK;
            APP::monit.LogInfo("[GetBlock] Block1 at lower & lift at top, descending to pick");
        }
        else if (_IsTimeout(T_BELT))
        {
            _StopBelt();
            APP::monit.LogError("[GetBlock] Belt timeout in LIFT_BEFORE_BELT");
            _fsm = FSM::ABORT;
        }
        break;
    }

    // ═══════════════════ 存块: 高位降下来接块 ═══════════════════
    case FSM::STORE_LIFT_DOWN_TO_PICK:
    {
        dm_lift_pos = -_lift_pos_lower;

        if (_IsTimeout(T_LIFT))
        {
            // _action_tick = System.runtime_tick;  // old
            _action_tick = _local_tick;
            _fsm = FSM::STORE_LIFT_UP;
            APP::monit.LogInfo("[GetBlock] Lift at bottom (timeout), block on flap, lift up");
        }
        break;
    }

    // ═══════════════════ 存块: 抬升 ↑ (挡片托着块到上仓) ═══════════════════
    case FSM::STORE_LIFT_UP:
    {
        dm_lift_pos = -_lift_pos_upper;

        // 传感器在上仓下沿，块刚进入就会被检测到 → 记录状态，继续上行到目标位置
        if (_grid[1] && !_upper)
        {
            _upper = true;
            _lower = false;
            // _action_tick = System.runtime_tick;  // old
            _action_tick = _local_tick;
            APP::monit.LogInfo("[GetBlock] Block entered upper chamber, continuing to target");
        }

        // 传感器触发后等待抬升到位（短超时）
        if (_upper && _IsTimeout(T_LIFT_SETTLE))
        {
            if (_storing_b2)
            {
                _fsm = FSM::COMPLETE;
                APP::monit.LogInfo("[GetBlock] Block2 stored at upper (pos reached)");
            }
            else
            {
                _fsm = FSM::STORE_WAIT_SECOND;
                APP::monit.LogInfo("[GetBlock] Block1 at upper, waiting for block2");
            }
        }
        else if (!_upper && _IsTimeout(T_LIFT))
        {
            APP::monit.LogError("[GetBlock] Lift up timeout (sensor not triggered)");
            _fsm = FSM::ABORT;
        }
        break;
    }

    // ═══════════════════ 存块: 等待块2 (挡片托着块1) ═══════════════════
    case FSM::STORE_WAIT_SECOND:
    {
        // 抬升机构保持在上位
        dm_lift_pos = -_lift_pos_upper;
        // 无超时 — 外部调用 Store() 会捕获此状态并转入 STORE_BELT_IN
        break;
    }

    // ═══════════════════ 存块: 等待Put推出块1 (挡片不能降) ═══════════════════
    case FSM::STORE_HOLD_PUT:
    {
        // 块2已在下仓，但挡片托着块1——不能降抬升(块1会掉)
        // 保持上位，等待外部 Put() 先推出块1，再降抬升处理块2
        dm_lift_pos = -_lift_pos_upper;
        // 无超时 — 外部调用 Put() 会捕获此状态并转入 PUT_FEED_PUSH
        break;
    }

    // ═══════════════════ 存块: 抬升 ↓ (挡片穿过块2) ═══════════════════
    case FSM::STORE_LIFT_DOWN:
    {
        // 下降时机：块1已被推出，块2在下仓
        // 挡片从上往下穿过块2→被块2顶开→通过后在块2下方闭合
        // 块2随挡片下移后离开上仓传感器检测范围 → 传感器辅助判断
        dm_lift_pos = -_lift_pos_lower;

        // 上仓传感器不再检测到块（块已随挡片下移离开），或超时兜底
        if (!_grid[1] && _IsTimeout(T_LIFT))
        {
            // _action_tick = System.runtime_tick;  // old
            _action_tick = _local_tick;
            _fsm = FSM::STORE_LIFT_BOTTOM;
            APP::monit.LogInfo("[GetBlock] Lift at bottom, flap closed below block2");
        }
        break;
    }

    // ═══════════════════ 存块: 挡片在块2下方闭合 ═══════════════════
    case FSM::STORE_LIFT_BOTTOM:
    {
        if (_storing_b2)
        {
            // 等待下仓传感器确认块2已在挡片上，或短超时兜底
            if (_grid[0])
            {
                _lower = true;
                _fsm = FSM::STORE_LIFT_UP;
                APP::monit.LogInfo("[GetBlock] Block2 on flap (sensor confirmed), lift up");
            }
            else if (_IsTimeout(0.5f))
            {
                _lower = true;
                _fsm = FSM::STORE_LIFT_UP;
                APP::monit.LogWarning("[GetBlock] Block2 on flap (timeout fallback), lift up");
            }
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
            // _action_tick = System.runtime_tick;  // old
            _action_tick = _local_tick;
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
            // _action_tick = System.runtime_tick;  // old
            _action_tick = _local_tick;
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
        dm_lift_pos = -_lift_pos_upper;

        // 传感器在上仓下沿，块刚进入就会检测到 → 记录状态，继续上行到顶
        if (_grid[1] && !_upper)
        {
            _upper = true;
            _lower = false;
            // _action_tick = System.runtime_tick;  // old
            _action_tick = _local_tick;
            APP::monit.LogInfo("[GetBlock] Block entered upper, continuing to top for ejection");
        }

        // 传感器触发后等待抬升到位 → 直接推出
        if (_upper && _IsTimeout(T_LIFT_SETTLE))
        {
            _fsm = FSM::PUT_FEED_PUSH;
            APP::monit.LogInfo("[GetBlock] Lift at top, ejecting block");
        }
        else if (!_upper && _IsTimeout(T_LIFT))
        {
            APP::monit.LogError("[GetBlock] Put lift up timeout");
            _fsm = FSM::ABORT;
        }
        break;
    }

    // ═══════════════════ 放块: 抬升回位 ═══════════════════
    case FSM::PUT_LIFT_DOWN:
    {
        dm_lift_pos = -_lift_pos_lower;

        if (_IsTimeout(T_LIFT))
        {
            // _action_tick = System.runtime_tick;  // old
            _action_tick = _local_tick;
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
        _storing_b2 = false;  // 重置，允许重复使用
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
        dm_lift_pos = -_lift_pos_lower;
        motor_2006_feed.SetPos(_feed_pos_retract);
        _fsm = FSM::COMPLETE;
        break;
    }

    default: break;
    }
}

