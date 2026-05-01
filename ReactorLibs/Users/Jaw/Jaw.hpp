#pragma once
#include "System.hpp"
#include "motor_dji.hpp"  
#include "relay.hpp"

// 状态枚举
enum MotorState 
{
    STATE_NORMAL,
    STATE_PROBING,
    STATE_LOCKED
};

// 电机状态处理器
struct MotorFSM 
{
    MotorState state = STATE_NORMAL;
    uint16_t probe_cnt = 0;
    
    void update(MotorDJI &motor, float target_pos) 
    {
        bool over_limit = fabs(motor.motor_adrc.eso.z3) > 1700;

        switch (state) 
        {
            case STATE_NORMAL:
                motor.SetPos(target_pos);
                if (over_limit) state = STATE_PROBING;
                break;

            case STATE_PROBING:
                motor.SetPos(target_pos); 
                if (!over_limit) 
                {
                    state = STATE_NORMAL;
                    probe_cnt = 0;
                } 
                else if (++probe_cnt >= 10) 
                {
                    state = STATE_LOCKED;
                }
                break;

            case STATE_LOCKED:
                motor.targ_current = 0;   // 发现撞限位了就锁死输出，应该还有更好的方法
                // 自动恢复
                if (!over_limit) 
                {        
                    probe_cnt = 0;
                    state = STATE_NORMAL;
                }
                break;
        }
    }
};

class Jaw : public Application
{
    SINGLETON(Jaw) : Application("Jaw") {};

    APPLICATION_OVERRIDE

private:    
    enum class JawState
    {
        OPEN = 0,
        CLOSE = 1
    };

    void _UploadPos();
    void _Update_Log();

    /// @brief 通过轻轻撞机械限位，获取原点
    void _GetHoriOrigin();
    void _GetVertOrigin();
    void _GetPitchOrigin();

    void _SetJawState(JawState state);

    float _x_pos = 0;  // 电机位置，单位是code（编码值）
    float _y_pos = 0;  
    float _pitch_pos = 0;

    // 上位机传来的偏移量
    float _x_pos_bias = 0; 
    float _y_pos_bias = 0;

    /// @brief 是否已经获取了原点
    bool _origined = false;
    bool _hori_origined = false;
    bool _vert_origined = false;
    bool _pitch_origined = false;

    int _hori_origin_code = 0;
    int _vert_origin_code = 0;
    int _pitch_origin_code = 0;

    int _hori_origined_code = 0; // origin后的位置
    int _vert_origined_code = 0; 
    int _pitch_origined_code = 0; 

    // 打印日志所需变量
    uint32_t log_tick_cnt_ = 0;
    uint32_t log_header_cnt_ = 8;

    // 夹爪状态
    JawState _state = JawState::CLOSE;  

    // debug取夹爪标志位
    int _rotate_90_flag = 0;

public:
    int GetPitchOriginCode() const { return _pitch_origin_code; }
    static void JawCmdRxCallback(uint8_t func_code, const uint8_t* payload, uint8_t payload_len, void* user_ctx); 


    // 电机
    MotorDJI motor_3508_hori;  // 3508 按 C620 协议接入
    MotorDJI motor_3508_vert;  // 3508 按 C620 协议接入
    MotorDJI motor_2006;     // 2006 按 C610 协议接入
    
    // 继电器
    Relay relay;  // 继电器，控制夹爪
    
    void Open(); // 打开夹爪
    void Close(); // 关闭夹爪
    void Rotate();
    void Reset();
};

extern Jaw &jaw;