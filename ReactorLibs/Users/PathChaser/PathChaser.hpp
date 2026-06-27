#pragma once
#include "System.hpp"
#include <cstddef>
#include <cstdint>

/**
 * @brief 单个路点所含有的信息
 */
struct PathWaypoint
{
    Vec2 pos;      // 位置
    float yaw_rad; // 朝向，单位rad
    Vec2 vel_vec;  // 期望速度向量，单位m/s
    float omega_rad; // 期望角速度，单位rad/s
    float t_sec;   // 时间戳，单位s
};

/**
 * @brief 提前设置好的路径。注意，本库没有动态规划功能
 * @warning 本类含有巨额内存开销，所以必须以static const的形式调用，否则炸RAM不怪我
 */
template <size_t waypointCount = 256>
class Path
{
    public:
    PathWaypoint waypoints[waypointCount];
};

/**
 * @brief 路径追踪器
 * 
 */
class PathChaserType : public Application
{
    SINGLETON(PathChaserType) : Application("PathChaser") {};
    APPLICATION_OVERRIDE

    public:

    struct CtrlCmd
    {
        Vec3 world;    // 世界坐标系速度命令 (vx, vy, omega)
        Vec3 body;     // 车体坐标系速度命令 (vx, vy, omega)
    };

    /// @brief 开始追踪路径并重置内部状态
    template <size_t N>
    void ChasePath(const Path<N>& path)
    {
        _waypoints = path.waypoints;
        _waypoint_count = static_cast<uint16_t>(N);
        _path_total_len = _path_length();
        _path_total_time = _path_time();
        _path_valid = _validate_path();
        Reset();
    }

    /// @brief 手动重置跟踪状态
    void Reset();

    /// @brief 是否完成路径跟踪（终点位置+姿态双阈值保持）
    bool IsFinished() const { return _finished; }

    /// @brief 获取世界系输出命令
    Vec3 GetCmdWorld() const { return _cmd.world; }

    /// @brief 获取车体系输出命令
    Vec3 GetCmdBody() const { return _cmd.body; }

    /// @brief 获取当前命令（世界系+车体系）
    CtrlCmd GetCmd() const { return _cmd; }

    private:

    struct PathRef
    {
        Vec2 pos;             // 时间参考位置
        float yaw = 0.0f;     // 时间参考朝向
        Vec2 vel;             // 时间参考世界系速度
        float omega = 0.0f;   // 时间参考角速度
        float t = 0.0f;       // 相对路径起点时间
    };

    // 当前跟踪的路径 的 路点数组指针 还有数量
    const PathWaypoint* _waypoints = nullptr;
    uint16_t _waypoint_count = 0;
    
    float _kp_pos = 1.8f;              // 位置误差反馈增益
    float _kp_yaw = 1.8f;              // 姿态误差反馈增益
    float _ff_gain = 1.1f;             // 前馈增强系数，可在实车上小幅调到1.05~1.20
    float _max_vel = 999.0f;             // 合速度上限，单位m/s
    float _max_omega = 999.0f;           // 角速度上限，单位rad/s
    float _max_pos_fb = 0.25f;         // 位置反馈速度上限，单位m/s
    float _max_yaw_fb = 0.50f;          // 姿态反馈角速度上限，单位rad/s
    float _ff_slow_start_err = 0.08f;  // 位置误差超过该值后开始压低前馈
    float _ff_slow_stop_err = 0.35f;   // 位置误差达到该值后前馈降到最小比例
    float _min_ff_scale = 0.20f;       // 误差较大时保留的最小前馈比例
    float _pos_tol = 0.005f;            // 终点位置阈值，单位m
    float _yaw_tol = 0.02f;            // 终点姿态阈值，单位rad
    uint16_t _finish_hold_ticks = 10;  // 完成保持周期，200Hz下约50ms

    float _path_total_len = 0.0f;      // 路径总长度（SetPath时预计算）
    float _path_total_time = 0.0f;     // 路径总时间
    bool _path_valid = false;          // 路径是否合法
    float _start_runtime_sec = 0.0f;   // 启动追踪时刻
    float _elapsed_sec = 0.0f;         // 已运行时间
    uint16_t _finish_tick_cnt = 0;
    bool _finished = false;

    CtrlCmd _cmd;

    PathRef _sample_ref_by_time(float t_sec) const;
    float _path_length() const;
    float _path_time() const;
    bool _validate_path() const;
    void _update_cmd(const Vec3& now_pose);
    static float _wrap_pi(float rad);
    static Vec2 _limit_vec2(Vec2 vec, float max_len);
};

namespace APP
{
    extern PathChaserType& path_chaser;
}
