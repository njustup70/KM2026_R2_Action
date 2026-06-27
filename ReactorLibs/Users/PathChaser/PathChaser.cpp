#include "PathChaser.hpp"
#include <cmath>

PathChaserType& APP::path_chaser = PathChaserType::GetInstance();

void PathChaserType::Start()
{
    Reset();
}

void PathChaserType::Update()
{
    _update_cmd(System.position);
}

void PathChaserType::Reset()
{
    _elapsed_sec = 0.0f;
    _finish_tick_cnt = 0;
    _finished = false;
    _start_runtime_sec = System.runtime_tick;
    _point_last_update_sec = System.runtime_tick;
    _point_last_vel_world = Vec2(0, 0);
    _point_last_omega = 0.0f;
    _cmd.world = Vec3(0, 0, 0);
    _cmd.body = Vec3(0, 0, 0);
}

void PathChaserType::ChaseGes(Vec3 targ_ges)
{
    _mode = ChaseMode::Point;
    _point_target_pos = targ_ges.ToVec2();
    _point_target_yaw = _wrap_pi(targ_ges.z);
    _point_pos_enabled = true;
    _point_yaw_enabled = true;
    Reset();
}

void PathChaserType::ChasePos(Vec2 targ_pos)
{
    _mode = ChaseMode::Point;
    _point_target_pos = targ_pos;
    _point_pos_enabled = true;
    _point_yaw_enabled = false;
    Reset();
}

void PathChaserType::ChaseYaw(float ang_rad)
{
    _mode = ChaseMode::Point;
    _point_target_yaw = _wrap_pi(ang_rad);
    _point_pos_enabled = false;
    _point_yaw_enabled = true;
    Reset();
}

void PathChaserType::SetYawTarget(float ang_rad)
{
    float target_yaw = _wrap_pi(ang_rad);
    bool target_changed = !_point_yaw_enabled || fabsf(_wrap_pi(target_yaw - _point_target_yaw)) > 1e-5f;

    _mode = ChaseMode::Point;
    _point_target_yaw = target_yaw;
    _point_yaw_enabled = true;
    if (target_changed)
    {
        _finished = false;
        _finish_tick_cnt = 0;
    }
}

float PathChaserType::_wrap_pi(float rad)
{
    while (rad > PI) rad -= 2.0f * PI;
    while (rad < -PI) rad += 2.0f * PI;
    return rad;
}

/**
 * @brief 二维向量按长度限幅
 * @param vec 输入向量
 * @param max_len 允许的最大长度
 * @return 限幅后的向量
 */
Vec2 PathChaserType::_limit_vec2(Vec2 vec, float max_len)
{
    float len = vec.Length();
    if (len <= max_len || len < 1e-6f) return vec;
    return vec * (max_len / len);
}

Vec2 PathChaserType::_limit_delta_vec2(Vec2 from, Vec2 to, float max_delta)
{
    Vec2 delta = to - from;
    float delta_len = delta.Length();
    if (delta_len <= max_delta || delta_len < 1e-6f) return to;
    return from + delta * (max_delta / delta_len);
}

float PathChaserType::_smooth_step(float x)
{
    x = StdMath::fclamp(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

/**
 * @brief 计算整条路径总长度
 * @return 路径长度（单位m）
 */
float PathChaserType::_path_length() const
{
    if (_waypoints == nullptr || _waypoint_count < 2) return 0.0f;

    float length = 0.0f;
    for (uint16_t i = 0; i + 1 < _waypoint_count; i++)
    {
        length += (_waypoints[i + 1].pos - _waypoints[i].pos).Length();
    }
    return length;
}

/**
 * @brief 读取路径总时间（以首尾路点时间差为准）
 * @return 路径总时间（单位s）
 */
float PathChaserType::_path_time() const
{
    if (_waypoints == nullptr || _waypoint_count < 2) return 0.0f;
    float t0 = _waypoints[0].t_sec;
    float t1 = _waypoints[_waypoint_count - 1].t_sec;
    return t1 - t0;
}

/**
 * @brief 检查路径合法性（时间必须严格递增，路径需要有非零时长和非零长度）
 */
bool PathChaserType::_validate_path() const
{
    if (_waypoints == nullptr || _waypoint_count < 2) return false;
    for (uint16_t i = 0; i + 1 < _waypoint_count; i++)
    {
        if (!(_waypoints[i + 1].t_sec > _waypoints[i].t_sec))
        {
            return false;
        }
    }
    return _path_time() > 1e-6f && _path_length() > 1e-6f;
}

/**
 * @brief 按相对路径起点的时间采样参考状态
 * @param t_sec 相对路径起点时间（单位s）
 * @return 插值得到的位置、姿态、前馈速度和前馈角速度
 */
PathChaserType::PathRef PathChaserType::_sample_ref_by_time(float t_sec) const
{
    PathRef ref;
    if (_waypoints == nullptr || _waypoint_count == 0) return ref;

    float path_t0 = _waypoints[0].t_sec;
    float query_t = path_t0 + t_sec;

    if (_waypoint_count == 1 || query_t <= _waypoints[0].t_sec)
    {
        ref.pos = _waypoints[0].pos;
        ref.yaw = _waypoints[0].yaw_rad;
        ref.vel = _waypoints[0].vel_vec;
        ref.omega = _waypoints[0].omega_rad;
        ref.t = 0.0f;
        return ref;
    }

    const PathWaypoint& last = _waypoints[_waypoint_count - 1];
    if (query_t >= last.t_sec)
    {
        ref.pos = last.pos;
        ref.yaw = last.yaw_rad;
        ref.vel = last.vel_vec;
        ref.omega = last.omega_rad;
        ref.t = last.t_sec - path_t0;
        return ref;
    }

    for (uint16_t i = 0; i + 1 < _waypoint_count; i++)
    {
        const PathWaypoint& a = _waypoints[i];
        const PathWaypoint& b = _waypoints[i + 1];
        if (query_t <= b.t_sec)
        {
            float dt = b.t_sec - a.t_sec;
            float alpha = (dt < 1e-6f) ? 0.0f : ((query_t - a.t_sec) / dt);
            alpha = StdMath::fclamp(alpha, 0.0f, 1.0f);

            float dyaw = _wrap_pi(b.yaw_rad - a.yaw_rad);
            ref.pos = a.pos * (1.0f - alpha) + b.pos * alpha;
            ref.yaw = _wrap_pi(a.yaw_rad + dyaw * alpha);
            ref.vel = a.vel_vec * (1.0f - alpha) + b.vel_vec * alpha;
            ref.omega = a.omega_rad * (1.0f - alpha) + b.omega_rad * alpha;
            ref.t = query_t - path_t0;
            return ref;
        }
    }

    ref.pos = last.pos;
    ref.yaw = last.yaw_rad;
    ref.vel = last.vel_vec;
    ref.omega = last.omega_rad;
    ref.t = last.t_sec - path_t0;
    return ref;
}

/**
 * @brief 单周期更新控制输出（200Hz调用）
 * @details 按时间采样参考点，执行前馈+位置反馈+姿态反馈，并完成限幅和坐标变换
 * @param now_pose 当前位姿 (x, y, yaw)
 */
void PathChaserType::_update_cmd(const Vec3& now_pose)
{
    if (_mode == ChaseMode::Path)
    {
        _update_path_cmd(now_pose);
        return;
    }

    if (_mode == ChaseMode::Point)
    {
        _update_point_cmd(now_pose);
        return;
    }

    _cmd.world = Vec3(0, 0, 0);
    _cmd.body = Vec3(0, 0, 0);
}

void PathChaserType::_update_path_cmd(const Vec3& now_pose)
{
    if (_waypoints == nullptr || _waypoint_count == 0 || _finished || !_path_valid)
    {
        _cmd.world = Vec3(0, 0, 0);
        _cmd.body = Vec3(0, 0, 0);
        return;
    }

    _elapsed_sec = System.runtime_tick - _start_runtime_sec;
    if (_elapsed_sec < 0.0f) _elapsed_sec = 0.0f;

    PathRef ref = _sample_ref_by_time(_elapsed_sec);
    Vec2 now_pos = now_pose.ToVec2();
    Vec2 pos_err = ref.pos - now_pos;
    float pos_err_len = pos_err.Length();

    Vec2 pos_fb = _limit_vec2(pos_err * _kp_pos, _max_pos_fb);
    float yaw_err = _wrap_pi(ref.yaw - now_pose.z);
    float yaw_fb = StdMath::fclamp(_kp_yaw * yaw_err, -_max_yaw_fb, _max_yaw_fb);

    float slowdown = 1.0f;
    if (_ff_slow_stop_err > _ff_slow_start_err && pos_err_len > _ff_slow_start_err)
    {
        float alpha = (pos_err_len - _ff_slow_start_err) / (_ff_slow_stop_err - _ff_slow_start_err);
        alpha = StdMath::fclamp(alpha, 0.0f, 1.0f);
        slowdown = 1.0f - (1.0f - _min_ff_scale) * alpha;
    }
    slowdown = StdMath::fclamp(slowdown, _min_ff_scale, 1.0f);

    float ff_scale = _ff_gain * slowdown;
    Vec2 v_world = _limit_vec2(ref.vel * ff_scale + pos_fb, _max_vel);
    float omega = StdMath::fclamp(ref.omega * ff_scale + yaw_fb, -_max_omega, _max_omega);

    Vec2 body_xy = v_world.Rotate(-now_pose.z);
    _cmd.world = Vec3(v_world.x, v_world.y, omega);
    _cmd.body = Vec3(body_xy.x, body_xy.y, omega);

    const PathWaypoint& final_wp = _waypoints[_waypoint_count - 1];
    float final_pos_err = (final_wp.pos - now_pos).Length();
    float final_yaw_err = fabsf(_wrap_pi(final_wp.yaw_rad - now_pose.z));
    bool time_ready = _elapsed_sec >= _path_total_time;

    if (time_ready && final_pos_err < _pos_tol && final_yaw_err < _yaw_tol)
    {
        if (_finish_tick_cnt < _finish_hold_ticks) _finish_tick_cnt++;
    }
    else
    {
        _finish_tick_cnt = 0;
    }

    if (_finish_tick_cnt >= _finish_hold_ticks)
    {
        _finished = true;
        _cmd.world = Vec3(0, 0, 0);
        _cmd.body = Vec3(0, 0, 0);
    }
}

void PathChaserType::_update_point_cmd(const Vec3& now_pose)
{
    if (_finished)
    {
        _cmd.world = Vec3(0, 0, 0);
        _cmd.body = Vec3(0, 0, 0);
        return;
    }

    float now_sec = System.runtime_tick;
    float dt = now_sec - _point_last_update_sec;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.005f;
    _point_last_update_sec = now_sec;

    Vec2 now_pos = now_pose.ToVec2();
    Vec2 pos_err = _point_target_pos - now_pos;
    _point_remain_dist = pos_err.Length();

    Vec2 target_vel_world(0, 0);
    if (_point_pos_enabled && _point_remain_dist > _pos_tol)
    {
        Vec2 dir = pos_err / _point_remain_dist;
        float v_by_p = _point_kp_pos * _point_remain_dist;
        float slow_scale = _smooth_step(_point_remain_dist / _point_slow_dist);
        float v_limit = _point_max_vel * slow_scale;
        float brake_limit = sqrtf(2.0f * _point_max_acc * _point_remain_dist);
        if (v_limit > brake_limit) v_limit = brake_limit;
        if (v_limit < _point_min_vel && brake_limit > _point_min_vel) v_limit = _point_min_vel;

        float speed = v_by_p;
        if (speed > v_limit) speed = v_limit;
        target_vel_world = dir * speed;
    }

    Vec2 v_world = _limit_delta_vec2(_point_last_vel_world,
                                     target_vel_world,
                                     _point_max_acc * dt);
    _point_last_vel_world = v_world;

    float yaw_err = _wrap_pi(_point_target_yaw - now_pose.z);
    _point_yaw_err_abs = fabsf(yaw_err);

    float target_omega = 0.0f;
    if (_point_yaw_enabled && _point_yaw_err_abs > _yaw_tol)
    {
        float omega_by_p = _point_kp_yaw * yaw_err;
        float slow_scale = _smooth_step(_point_yaw_err_abs / _point_slow_yaw);
        float omega_limit = _point_max_omega * slow_scale;
        float brake_limit = sqrtf(2.0f * _point_max_alpha * _point_yaw_err_abs);
        if (omega_limit > brake_limit) omega_limit = brake_limit;
        if (omega_limit < _point_min_omega && brake_limit > _point_min_omega) omega_limit = _point_min_omega;
        target_omega = StdMath::fclamp(omega_by_p, -omega_limit, omega_limit);
    }

    float omega_delta = target_omega - _point_last_omega;
    float max_omega_delta = _point_max_alpha * dt;
    omega_delta = StdMath::fclamp(omega_delta, -max_omega_delta, max_omega_delta);
    float omega = _point_last_omega + omega_delta;
    _point_last_omega = omega;

    Vec2 body_xy = v_world.Rotate(-now_pose.z);
    _cmd.world = Vec3(v_world.x, v_world.y, omega);
    _cmd.body = Vec3(body_xy.x, body_xy.y, omega);

    bool pos_done = !_point_pos_enabled || _point_remain_dist < _pos_tol;
    bool yaw_done = !_point_yaw_enabled || _point_yaw_err_abs < _yaw_tol;

    if (pos_done && yaw_done)
    {
        if (_finish_tick_cnt < _finish_hold_ticks) _finish_tick_cnt++;
    }
    else
    {
        _finish_tick_cnt = 0;
    }

    if (_finish_tick_cnt >= _finish_hold_ticks)
    {
        _finished = true;
        _point_last_vel_world = Vec2(0, 0);
        _point_last_omega = 0.0f;
        _cmd.world = Vec3(0, 0, 0);
        _cmd.body = Vec3(0, 0, 0);
    }
}
