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

    // 当前跟踪的路径 的 路点数组指针 还有数量
    const PathWaypoint* _waypoints = nullptr;
    uint16_t _waypoint_count = 0;
    
    float _kp_pos = 2.6f;              // 位置误差增益
    float _kp_yaw = 4.5f;              // 姿态误差增益
    float _k_sync_time = 1.0f;         // 与时间参考同步增益
    float _k_sync_meas = 0.45f;        // 与实测进度同步增益
    float _max_vel = 1.0f;             // 合速度上限，单位m/s
    float _max_omega = 3.14f;           // 角速度上限，单位rad/s
    float _max_pos_fb = 0.30f;         // 位置反馈速度上限，单位m/s
    float _max_yaw_fb = 1.00f;         // 姿态反馈角速度上限，单位rad/s
    float _max_s_dot = 1.20f;          // 目标进度速度上限，单位m/s
    float _min_s_dot = -0.15f;         // 目标进度速度下限，单位m/s
    float _pos_tol = 0.03f;            // 终点位置阈值，单位m
    float _yaw_tol = 0.04f;            // 终点姿态阈值，单位rad
    float _finish_time_margin = 0.05f; // 完成时间容差，单位s
    uint16_t _finish_hold_ticks = 24;  // 完成保持周期，24*5ms=120ms

    uint16_t _near_seg_idx = 0;        // 上一次最近段索引
    float _path_total_len = 0.0f;      // 路径总长度（SetPath时预计算）
    float _path_total_time = 0.0f;     // 路径总时间（SetPath时预计算）
    bool _path_valid = false;          // 路径是否合法
    float _start_runtime_sec = 0.0f;   // 启动追踪时刻
    float _last_runtime_sec = 0.0f;    // 上次Update时刻
    float _elapsed_sec = 0.0f;         // 已运行时间
    float _s_cmd = 0.0f;               // 控制目标进度
    float _s_ref = 0.0f;               // 时间参考进度
    float _s_meas = 0.0f;              // 位置实测进度
    float _s_dot_cmd = 0.0f;           // 控制目标进度速度
    uint16_t _finish_tick_cnt = 0;
    bool _finished = false;

    CtrlCmd _cmd;

    Vec2 _sample_pos(float dist) const;
    Vec2 _sample_vel(float dist) const;
    float _sample_yaw(float dist) const;
    float _sample_omega(float dist) const;
    float _path_length() const;
    float _path_time() const;
    bool _validate_path() const;
    float _sample_s_by_time(float t_sec) const;
    float _solve_progress(const Vec2& now_pos);
    void _update_cmd(const Vec3& now_pose);
    static float _wrap_pi(float rad);
    static Vec2 _limit_vec2(Vec2 vec, float max_len);
};

namespace APP
{
    extern PathChaserType& path_chaser;
}
