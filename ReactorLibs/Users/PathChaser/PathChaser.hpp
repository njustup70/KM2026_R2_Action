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

    struct PathFrame
    {
        float s = 0.0f;        // 弧长进度
        Vec2 pos;              // 当前弧长对应的路径点
        Vec2 tangent;          // 切向单位向量
        Vec2 normal;           // 法向单位向量
        float yaw = 0.0f;      // 切线朝向
        float curvature = 0.0f;// 曲率估计
    };

    // 当前跟踪的路径 的 路点数组指针 还有数量
    const PathWaypoint* _waypoints = nullptr;
    uint16_t _waypoint_count = 0;
    
    float _kp_pos = 2.0f;              // 横向误差增益（法向反馈）
    float _kp_yaw = 2.2f;              // 姿态误差增益
    float _k_sync_time = 0.0f;         // 预留：旧时间同步增益（几何跟踪中不作为主驱动）
    float _k_sync_meas = 4.0f;         // 弧长进度平滑/跟随增益
    float _max_vel = 1.20f;            // 合速度上限，单位m/s
    float _max_omega = 1.80f;          // 角速度上限，单位rad/s
    float _max_pos_fb = 0.28f;         // 法向反馈速度上限，单位m/s
    float _max_yaw_fb = 0.80f;         // 姿态反馈角速度上限，单位rad/s
    float _yaw_ff_degrade_range = 0.3f; // 预留：yaw前馈衰减范围
    float _max_s_dot = 1.00f;          // 进度平滑器速度上限，单位m/s
    float _min_s_dot = -0.15f;         // 目标进度速度下限，单位m/s
    float _pos_tol = 0.01f;            // 终点位置阈值，单位m
    float _yaw_tol = 0.02f;            // 终点姿态阈值，单位rad
    float _lookahead_min = 0.08f;      // 最小前瞻距离，单位m
    float _lookahead_max = 0.30f;      // 最大前瞻距离，单位m
    float _lookahead_gain = 0.18f;     // 前瞻距离随速度变化的增益
    float _curve_lat_accel = 1.60f;    // 曲率限速对应的允许横向加速度，单位m/s^2
    float _finish_progress_margin = 0.05f; // 判定完成前需进入的末端进度窗口，单位m
    uint16_t _finish_hold_ticks = 15;  // 完成保持周期，200Hz下约75ms

    uint16_t _near_seg_idx = 0;        // 上一次最近段索引
    float _path_total_len = 0.0f;      // 路径总长度（SetPath时预计算）
    float _path_total_time = 0.0f;     // 路径总时间（保留，兼容旧逻辑/调试）
    bool _path_valid = false;          // 路径是否合法
    float _start_runtime_sec = 0.0f;   // 启动追踪时刻
    float _last_runtime_sec = 0.0f;    // 上次Update时刻
    float _elapsed_sec = 0.0f;         // 已运行时间
    float _s_cmd = 0.0f;               // 控制目标进度
    float _s_ref = 0.0f;               // 时间参考进度（调试用，当前不主导控制）
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
    PathFrame _sample_frame(float dist) const;
    PathFrame _solve_progress(const Vec2& now_pos);
    void _update_cmd(const Vec3& now_pose);
    static float _wrap_pi(float rad);
    static Vec2 _limit_vec2(Vec2 vec, float max_len);
};

namespace APP
{
    extern PathChaserType& path_chaser;
}
