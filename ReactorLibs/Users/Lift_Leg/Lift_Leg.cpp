#include "Lift_Leg.hpp"
#include "Interboardcomm.hpp"
#include "board.hpp"
#include "Chassis.hpp"

Lift_Leg& APP::leg = Lift_Leg::GetInstance();

void Lift_Leg::Start()
{
    // 小黄注册：0-3 分别是从前到后四个
    Pin pin[4] = 
    {
        {'I', 5},
        {'A', 1},
        {'A', 2},
        {'A', 3}
    };
    
    for(int i = 0; i < 4; i++)
    {
        detector[i].Init(pin[i]);
    }

    Pin pin_back_wheel = {'F', 0};

    // 接收 C 板回传的实际腿部位置：task 2
    MOD::board_can.RegisterTask(2, LegActRxCb, this);

    relay_daji.Init(pin_back_wheel, Relay::HIGH_ON);
    // 启动后，先关闭妲己
    relay_daji.Off();
}

void Lift_Leg::_SendLegPos(int16_t fpos, int16_t bpos)
{
    uint8_t payload[4];

    payload[0] = (fpos >> 8) & 0xFF;
    payload[1] = fpos & 0xFF;
    payload[2] = (bpos >> 8) & 0xFF;
    payload[3] = bpos & 0xFF;

    MOD::board_can.SendTask(MOD::C_sub_ctrl_board_id, 6, payload, sizeof(payload), false);
}

void Lift_Leg::_UpdateSlowMove()
{
    const float now = DWT_GetTimeline_Sec();

    auto update_axis = [now](SlowMoveState& st, int& pos, int16_t min_lim, int16_t max_lim)
    {
        if (!st.active)
        {
            return;
        }

        float elapsed = now - st.start_time;
        if (elapsed < 0.0f)
        {
            elapsed = 0.0f;
        }

        float t = (st.duration > 0.001f) ? (elapsed / st.duration) : 1.0f;
        if (t >= 1.0f)
        {
            pos = st.target_pos;
            st.active = false;
            return;
        }

        const float blend = t * t * (3.0f - 2.0f * t); // 平滑步进，起止更柔和
        pos = static_cast<int>(st.start_pos + (st.target_pos - st.start_pos) * blend);

        if (pos > max_lim) pos = max_lim;
        if (pos < min_lim) pos = min_lim;
    };

    update_axis(front_slow_, front_pos, _front_min_lim, _front_max_lim);
    update_axis(back_slow_, back_pos, _back_min_lim, _back_max_lim);
}

void Lift_Leg::Update()
{
    // 读取小黄
    front_ir = detector[0].Read();
    front_mid_ir = detector[1].Read();
    back_mid_ir = detector[2].Read();
    back_ir = detector[3].Read();
    
    if (!enabled_)  return;

    // 慢速动作由这里逐帧推进，避免在应用内部阻塞等待
    _UpdateSlowMove();
 
    // 进行位置限制
    PosLim();
    
    // 发送腿部位置指令
    _SendLegPos(front_pos, back_pos);

    // if(is_debug)
    // {
    //     relay_daji.On();
    // }
    // else
    // {
    //     relay_daji.Off();
    // }
}

void Lift_Leg::Enable()
{
    enabled_ = true;
}

void Lift_Leg::Disable()
{
    enabled_ = false;
}

inline void Lift_Leg::PosLim()
{
    if (front_pos > _front_max_lim)
    {
        front_pos = _front_max_lim;
    }
    else if (front_pos < _front_min_lim)
    {
        front_pos = _front_min_lim;
    }

    if (back_pos > _back_max_lim)
    {
        back_pos = _back_max_lim;
    }
    else if (back_pos < _back_min_lim)
    {
        back_pos = _back_min_lim;
    }
}

/**
 * @brief C板发回的，关于腿部位置 的 回调
 *
 * @param task_id 2
 * @param payload 0-1位：int16:前腿位置，2-3位：int16:后腿位置，均为mm
 * @param payload_len 4
 */
void Lift_Leg::LegActRxCb(uint8_t task_id, const uint8_t* payload, uint8_t payload_len, void* user_ctx)
{
    auto* self = static_cast<Lift_Leg*>(user_ctx);
    if (self == nullptr || payload == nullptr || payload_len < 4)
    {
        return;
    }

    const int16_t front_raw = static_cast<int16_t>((static_cast<uint16_t>(payload[0]) << 8) | payload[1]);
    const int16_t back_raw  = static_cast<int16_t>((static_cast<uint16_t>(payload[2]) << 8) | payload[3]);

    self->real_front_pos_mm = front_raw;
    self->real_back_pos_mm = back_raw;
}

/********************************************  越野部分 ********************************************/
namespace
{
    inline int16_t ClampInt16(int16_t value, int16_t min_v, int16_t max_v)
    {
        if (value < min_v) return min_v;
        if (value > max_v) return max_v;
        return value;
    }
}

// 前腿回到零位
void Lift_Leg::F_Origin()
{
    front_slow_.active = false;
    front_pos = -2;
}

// 后腿回到零位
void Lift_Leg::B_Origin()
{
    back_slow_.active = false;
    back_pos = -2;
}

void Lift_Leg::F_AlmostOrigin()
{
    front_slow_.active = false;
    front_pos = -5;
}

// 后腿回到零位
void Lift_Leg::B_AlmostOrigin()
{
    back_slow_.active = false;
    back_pos = -5;
}

// 前腿放低200
void Lift_Leg::F_Down_200()
{
    front_slow_.active = false;
    front_pos = -207;
}

// 前腿放低400
void Lift_Leg::F_Down_400()
{
    front_slow_.active = false;
    front_pos = -407;
}

// 后腿放低200
void Lift_Leg::B_Down_200()
{
    back_slow_.active = false;
    back_pos = -207;
}

// 后腿放低400
void Lift_Leg::B_Down_400()
{
    back_slow_.active = false;
    back_pos = -407;
}

/**
 * @brief 慢慢移动前腿到目标位置
 * 
 * @param target_pos 
 * @param use_sec 
 */
void Lift_Leg::SetFrontPosSlow(int16_t target_pos, float use_sec)
{
    target_pos = ClampInt16(target_pos, _front_min_lim, _front_max_lim);
    if (use_sec < 0.12f)
    {
        use_sec = 0.12f;
    }
    front_slow_.active = true;
    front_slow_.start_pos = static_cast<int16_t>(front_pos);
    front_slow_.target_pos = target_pos;
    front_slow_.start_time = DWT_GetTimeline_Sec();
    front_slow_.duration = use_sec;
}


void Lift_Leg::SetBackPosSlow(int16_t target_pos, float use_sec)
{
    target_pos = ClampInt16(target_pos, _back_min_lim, _back_max_lim);
    if (use_sec < 0.12f)
    {
        use_sec = 0.12f;
    }
    back_slow_.active = true;
    back_slow_.start_pos = static_cast<int16_t>(back_pos);
    back_slow_.target_pos = target_pos;
    back_slow_.start_time = DWT_GetTimeline_Sec();
    back_slow_.duration = use_sec;
}

void Wait_detected(InfraredDetector& xiaohuang_x)
{
    Seq::WaitUntil([&]()
    {
        return xiaohuang_x.Read();
    });
}
