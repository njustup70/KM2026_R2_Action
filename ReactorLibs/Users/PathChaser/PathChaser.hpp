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
    void ChasePath(const Path<N>& path, bool use_t_ref = true)
    {
        _waypoints = path.waypoints;
        _waypoint_count = static_cast<uint16_t>(N);
        _path_total_len = _path_length();
        _path_total_time = _path_time();
        _path_valid = _validate_path();
        _use_t_ref = use_t_ref;
        _mode = ChaseMode::Path;
        Reset();
    }
    
    /// @brief 普通跑点：同时控制位置和yaw
    void ChaseGes(Vec3 targ_ges);

    /// @brief 普通跑点：只控制位置，不主动控制yaw
    void ChasePos(Vec2 targ_pos);

    /// @brief 普通跑点：只控制yaw，不主动控制位置
    void ChaseYaw(float ang_rad);

    /// @brief 给当前普通跑点挂上yaw目标，适合跑到一半再开始转头
    void SetYawTarget(float ang_rad);

    /// @brief 获取普通跑点当前位置误差，单位m
    float GetRemainDist() const { return _point_remain_dist; }

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

    enum class ChaseMode
    {
        Idle,
        Path,
        Point,
    };

    // 当前跟踪的路径 的 路点数组指针 还有数量
    const PathWaypoint* _waypoints = nullptr;
    uint16_t _waypoint_count = 0;
    
    float _kp_pos = 1.5f;              // 时间参考路径的位置误差反馈增益
    float _kp_yaw = 2.0f;              // 时间参考路径的姿态误差反馈增益
    float _ff_gain = 0.625f;            // 时间参考路径的前馈增强系数
    float _max_vel = 999.0f;             // 合速度上限，单位m/s
    float _max_omega = 999.0f;           // 角速度上限，单位rad/s
    float _max_pos_fb = 2.0f;         // 时间参考路径的位置反馈速度上限，单位m/s
    float _max_yaw_fb = 1.0f;         // 时间参考路径的姿态反馈角速度上限，单位rad/s
    float _ff_slow_start_err = 0.4f;  // 位置误差超过该值后开始压低前馈
    float _ff_slow_stop_err = 1.0f;   // 位置误差达到该值后前馈降到最小比例
    float _min_ff_scale = 0.20f;       // 误差较大时保留的最小前馈比例
    float _time_end_ff_fade_time = 0.0f; // 时间参考路径末端前馈衰减时间窗，单位s
    float _time_end_slow_dist = 0.5f; // 时间参考路径末端刹车距离，单位m
    float _time_end_max_acc = 0.75f;    // 时间参考路径末端刹车限速使用的最大减速度，单位m/s^2
    float _time_end_min_vel = 0.03f;   // 时间参考路径末端未完成时保留的最低修正速度，单位m/s
    float _pos_tol = 0.015f;            // 终点位置阈值，单位m
    float _yaw_tol = 0.02f;            // 终点姿态阈值，单位rad
    uint16_t _finish_hold_ticks = 10;  // 完成保持周期，200Hz下约50ms

    float _path_total_len = 0.0f;      // 路径总长度（SetPath时预计算）
    float _path_total_time = 0.0f;     // 路径总时间
    bool _path_valid = false;          // 路径是否合法
    bool _use_t_ref = true;            // true=按时间参考，false=按路径进度参考
    float _start_runtime_sec = 0.0f;   // 启动追踪时刻
    float _elapsed_sec = 0.0f;         // 已运行时间
    uint16_t _finish_tick_cnt = 0;
    bool _finished = false;

    float _prog_kp_along = 0.5f;       // 路径进度主导的沿路径方向反馈增益
    float _prog_kp_cross = 0.6f;       // 路径进度主导的横向贴线反馈增益
    float _prog_kp_yaw = 0.8f;         // 路径进度主导的姿态误差反馈增益
    float _prog_ff_gain = 0.45f;       // 路径进度主导的前馈增强系数
    float _prog_max_along_fb = 0.5f;  // 路径进度主导的沿路径反馈速度上限，单位m/s
    float _prog_max_cross_fb = 0.5f;  // 路径进度主导的横向贴线速度上限，单位m/s
    float _prog_max_yaw_fb = 0.5f;     // 路径进度主导的姿态反馈角速度上限，单位rad/s
    float _lookahead_dist = 0.1f;      // 路径进度主导模式的基础前瞻距离，单位m
    float _lookahead_min = 0.08f;       // 终点附近收缩后的最小前瞻距离，单位m
    float _lookahead_max = 0.35f;       // 前瞻距离上限，单位m
    float _path_end_slow_dist = 0.45f;  // 距终点小于该路径长度后开始按刹车距离限速
    float _path_end_max_acc = 1.5f;     // 路径末端刹车限速使用的最大减速度，单位m/s^2
    float _path_end_min_vel = 0.05f;    // 路径末端未完成时保留的最低修正速度，单位m/s

    ChaseMode _mode = ChaseMode::Idle;

    Vec2 _point_target_pos;
    float _point_target_yaw = 0.0f;
    bool _point_pos_enabled = false;
    bool _point_yaw_enabled = false;
    float _point_last_update_sec = 0.0f;
    Vec2 _point_last_vel_world;
    float _point_last_omega = 0.0f;
    float _point_remain_dist = 0.0f;
    float _point_yaw_err_abs = 0.0f;

    float _point_kp_pos = 1.6f;          // 普通跑点位置P增益
    float _point_kp_yaw = 1.25f;          // 普通跑点yaw P增益
    float _point_max_vel = 3.0f;        // 普通跑点最大平移速度，单位m/s
    float _point_min_vel = 0.2f;        // 进入减速段后的最低有效速度，防止尾段太软
    float _point_max_omega = 4.25f;      // 普通跑点最大角速度，单位rad/s
    float _point_min_omega = 0.2f;      // 普通跑点最低有效角速度，防止尾段太软
    float _point_max_acc = 3.50f;        // 平移速度变化率上限，单位m/s^2
    float _point_max_alpha = 3.50f;      // 角速度变化率上限，单位rad/s^2
    float _point_slow_dist = 0.5f;      // 小于该距离后按smoothstep收速度
    float _point_slow_yaw = 0.5f;       // 小于该角度后按smoothstep收角速度

    CtrlCmd _cmd;

    PathRef _sample_ref_by_time(float t_sec) const;
    PathRef _sample_ref_by_s(float s) const;
    Vec2 _sample_tangent_by_s(float s) const;
    float _find_nearest_path_s(Vec2 now_pos) const;
    float _path_length() const;
    float _path_time() const;
    bool _validate_path() const;
    void _update_cmd(const Vec3& now_pose);
    void _update_path_cmd(const Vec3& now_pose);
    void _update_time_ref_path_cmd(const Vec3& now_pose);
    void _update_progress_ref_path_cmd(const Vec3& now_pose);
    void _update_point_cmd(const Vec3& now_pose);
    static float _wrap_pi(float rad);
    static Vec2 _limit_vec2(Vec2 vec, float max_len);
    static Vec2 _limit_delta_vec2(Vec2 from, Vec2 to, float max_delta);
    static float _smooth_step(float x);
};

namespace APP
{
    extern PathChaserType& path_chaser;
}
