#pragma once
#include "System.hpp"
#include "motor_dji.hpp"  
#include "relay.hpp"

class Jaw : public Application
{
    SINGLETON(Jaw) : Application("Jaw") {};

    APPLICATION_OVERRIDE

private:    
    static void _SpearHeadCb(uint8_t func_code, const uint8_t* payload, uint8_t payload_len, void* user_ctx);

    // 电机
    MotorDJI motor_3508_hori;  // 3508 按 C620 协议接入
    MotorDJI motor_3508_vert;  // 3508 按 C620 协议接入
    MotorDJI motor_2006;     // 2006 按 C610 协议接入
    
    // 继电器
    Relay relay;  // 继电器，控制夹爪

    void _UploadPos();
    void _Update_Log();
    void _Update_LogHead();
    void _Update_GetOrigin();
    void _Track_Head();

    /// @brief 通过轻轻撞机械限位，获取原点
    void _GetHoriOrigin();
    void _GetVertOrigin();
    void _GetPitchOrigin();

    float _x_pos = 0;  // 电机位置，单位是code（编码值）
    float _y_pos = 0;  
    float _pitch_pos = 0;

    /// @brief 是否已经获取了原点
    bool _origined = false;
    bool _hori_origined = false;
    bool _vert_origined = false;
    bool _pitch_origined = false;

    // 原点位置
    int _hori_origin_code = 0;
    int _vert_origin_code = 0;
    int _pitch_origin_code = 0;

    // 上位机下发的取矛头平面误差（单位 mm）
    float _spear_left_err_mm = 0.0f;
    float _spear_up_err_mm = 0.0f;
    bool _spear_err_valid = false;

    // 使能标志
    bool _enabled = false;

    // 打印日志所需变量
    uint32_t log_tick_cnt_ = 0;
    uint32_t log_head_tick_cnt_ = 0;
    uint32_t log_header_cnt_ = 8;


    /**--- ---  随动相关变量 --- ---**/
    // 随动范围限制
    int _hori_tracklim_max = 0;
    int _hori_tracklim_min = 0;
    int _vert_tracklim_max = 0;
    int _vert_tracklim_min = 0;

    bool _in_track_mode = false;

    Vec2 _arcuo_bias_mm = {8, 8};

    // code 到 mm 互转方法（静态）
    static float _HoriCodeToMM(int code);
    static float _VertCodeToMM(int code);
    static int _HoriMMToCode(float mm);
    static int _VertMMToCode(float mm);

    // int GetPitchOriginCode() const { return _pitch_origin_code; }
    // float GetSpearLeftErrMM() const { return _spear_left_err_mm; }
    // float GetSpearUpErrMM() const { return _spear_up_err_mm; }
    // bool HasSpearErr() const { return _spear_err_valid; }
    // void ClearSpearErrFlag() { _spear_err_valid = false; }

public:
    int GetPitchOriginCode() const { return _pitch_origin_code; }
    bool IsOrigined() const { return _origined; }

    /**
     * @brief 设置是否进入随动模式，进入后会根据取矛头误差自动调整位置
     * 
     * @param enable 
     */
    void SetTrackMode(bool enable) { _in_track_mode = enable; }

    /**
     * @brief 设置 **相对于中心的** 取矛头位置。向上和向右为正。
     * @details 所谓中心，就是完成初始化后，机构测得的中心。这个位置是由机械限位和自动调零结果决定的。
     * @warning 需要注意的是————完成调零后，机构并不是回到原点。
     * 
     * @param x_mm 
     * @param y_mm 
     */
    void SetPos(float x_mm, float y_mm);

    /**
     * @brief 相对当前位置改变位置。会被其他自动位置控制覆盖。
     * 
     * @param delta_x_mm
     * @param delta_y_mm
     */
    void ChangePos(float delta_x_mm, float delta_y_mm);

    /**
     * @brief 把夹爪竖起来
     */
    void PitchUp();

    /**
     * @brief 把夹爪放平
     */
    void PitchDown();

    /**
     * @brief 张开夹爪
     */
    void OpenClaw();

    /**
     * @brief 合上夹爪
     */
    void CloseClaw();

    void Enable();
    void Disable();
    
};

extern int jaw_recv_cnt_test;

namespace APP
{
    extern Jaw& jaw;
}
