#pragma once
#include "System.hpp"
#include "motor_dji.hpp"  
#include "relay.hpp"

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

    /// @brief 是否已经获取了原点
    bool _origined = false;
    bool _hori_origined = false;
    bool _vert_origined = false;
    bool _pitch_origined = false;

    int _hori_origin_code = 0;
    int _vert_origin_code = 0;
    int _pitch_origin_code = 0;

    // 打印日志所需变量
    uint32_t log_tick_cnt_ = 0;
    uint32_t log_header_cnt_ = 8;

    // 夹爪状态
    JawState _state = JawState::CLOSE;  

public:
    int GetPitchOriginCode() const { return _pitch_origin_code; }


    // 电机
    MotorDJI motor_3508_hori;  // 3508 按 C620 协议接入
    MotorDJI motor_3508_vert;  // 3508 按 C620 协议接入
    MotorDJI motor_2006;     // 2006 按 C610 协议接入
    
    // 继电器
    Relay relay;  // 继电器，控制夹爪
    
    void Open(); // 打开夹爪
    void Close(); // 关闭夹爪
};
