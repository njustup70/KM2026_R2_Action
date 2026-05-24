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
    _near_seg_idx = 0;
    _elapsed_sec = 0.0f;
    _s_cmd = 0.0f;
    _s_ref = 0.0f;
    _s_meas = 0.0f;
    _s_dot_cmd = 0.0f;
    _finish_tick_cnt = 0;
    _finished = false;
    _start_runtime_sec = System.runtime_tick;
    _last_runtime_sec = _start_runtime_sec;
    _cmd.world = Vec3(0, 0, 0);
    _cmd.body = Vec3(0, 0, 0);
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
 * @brief 检查路径合法性（时间必须严格递增）
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
    return _path_time() > 1e-6f;
}

/**
 * @brief 最近点投影求路径进度（弧长）
 * @param now_pos 当前世界坐标位置
 * @return 当前进度弧长（单位m）
 */
float PathChaserType::_solve_progress(const Vec2& now_pos)
{
    if (_waypoints == nullptr || _waypoint_count < 2) return 0.0f;

    // 只在上一次最近段附近做窗口搜索，降低每周期计算量
    uint16_t seg_max = static_cast<uint16_t>(_waypoint_count - 1);
    uint16_t idx_begin = (_near_seg_idx > 2) ? (_near_seg_idx - 2) : 0;
    uint16_t idx_end = (_near_seg_idx + 6 < seg_max) ? (_near_seg_idx + 6) : seg_max;

    // 先累计到窗口起点的弧长，后续 prefix 在窗口内递推
    float base_dist = 0.0f;
    for (uint16_t i = 0; i < idx_begin; i++)
    {
        base_dist += (_waypoints[i + 1].pos - _waypoints[i].pos).Length();
    }

    float best_dist2 = 1e30f;
    float best_progress = _s_meas;
    float prefix = base_dist;
    uint16_t best_seg = idx_begin;

    for (uint16_t i = idx_begin; i < idx_end; i++)
    {
        Vec2 a = _waypoints[i].pos;
        Vec2 b = _waypoints[i + 1].pos;
        Vec2 ab = b - a;
        float len2 = ab * ab;
        float seg_len = sqrtf(len2);
        float t = 0.0f;
        if (len2 > 1e-8f)
        {
            // t 为点到线段的投影参数，并钳制到 [0, 1]
            t = ((now_pos - a) * ab) / len2;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
        }

        Vec2 proj = a + ab * t;
        Vec2 err = now_pos - proj;
        float d2 = err * err;
        if (d2 < best_dist2)
        {
            // 选取“投影误差最小”的段，并转换为全路径弧长进度
            best_dist2 = d2;
            best_progress = prefix + seg_len * t;
            best_seg = i;
        }
        prefix += seg_len;
    }

    // 避免进度大幅倒退（例如局部抖动/短时偏航），允许小范围容错回退
    if (best_progress + 0.08f < _s_meas)
    {
        best_progress = _s_meas - 0.08f;
    }
    if (best_progress < 0.0f) best_progress = 0.0f;

    _near_seg_idx = best_seg;
    return best_progress;
}

/**
 * @brief 按时间采样参考弧长进度
 * @param t_sec 相对路径起点时间（单位s）
 * @return 对应的路径弧长进度（单位m）
 */
float PathChaserType::_sample_s_by_time(float t_sec) const
{
    if (_waypoints == nullptr || _waypoint_count < 2) return 0.0f;

    float t0 = _waypoints[0].t_sec;
    float t_end = _waypoints[_waypoint_count - 1].t_sec - t0;
    if (t_sec <= 0.0f) return 0.0f;
    if (t_sec >= t_end) return _path_total_len;

    float acc_s = 0.0f;
    for (uint16_t i = 0; i + 1 < _waypoint_count; i++)
    {
        float ta = _waypoints[i].t_sec - t0;
        float tb = _waypoints[i + 1].t_sec - t0;
        Vec2 pa = _waypoints[i].pos;
        Vec2 pb = _waypoints[i + 1].pos;
        float seg_len = (pb - pa).Length();

        if (t_sec <= tb)
        {
            float dt = tb - ta;
            float alpha = (dt < 1e-6f) ? 0.0f : ((t_sec - ta) / dt);
            alpha = StdMath::fclamp(alpha, 0.0f, 1.0f);
            return acc_s + seg_len * alpha;
        }
        acc_s += seg_len;
    }
    return _path_total_len;
}

/**
 * @brief 按弧长采样目标位置
 * @param dist 距离路径起点的弧长
 * @return 插值得到的位置
 */
Vec2 PathChaserType::_sample_pos(float dist) const
{
    if (_waypoints == nullptr || _waypoint_count == 0) return Vec2(0, 0);
    if (_waypoint_count == 1) return _waypoints[0].pos;
    if (dist <= 0.0f) return _waypoints[0].pos;

    float acc = 0.0f;
    for (uint16_t i = 0; i + 1 < _waypoint_count; i++)
    {
        Vec2 a = _waypoints[i].pos;
        Vec2 b = _waypoints[i + 1].pos;
        float seg_len = (b - a).Length();
        if (acc + seg_len >= dist)
        {
            float t = (seg_len < 1e-6f) ? 0.0f : (dist - acc) / seg_len;
            return a + (b - a) * t;
        }
        acc += seg_len;
    }
    return _waypoints[_waypoint_count - 1].pos;
}

/**
 * @brief 按弧长采样前馈平移速度
 * @param dist 距离路径起点的弧长
 * @return 插值得到的速度向量
 */
Vec2 PathChaserType::_sample_vel(float dist) const
{
    if (_waypoints == nullptr || _waypoint_count == 0) return Vec2(0, 0);
    if (_waypoint_count == 1) return _waypoints[0].vel_vec;
    if (dist <= 0.0f) return _waypoints[0].vel_vec;

    float acc = 0.0f;
    for (uint16_t i = 0; i + 1 < _waypoint_count; i++)
    {
        Vec2 a = _waypoints[i].pos;
        Vec2 b = _waypoints[i + 1].pos;
        float seg_len = (b - a).Length();
        if (acc + seg_len >= dist)
        {
            float t = (seg_len < 1e-6f) ? 0.0f : (dist - acc) / seg_len;
            return _waypoints[i].vel_vec * (1.0f - t) + _waypoints[i + 1].vel_vec * t;
        }
        acc += seg_len;
    }
    return _waypoints[_waypoint_count - 1].vel_vec;
}

/**
 * @brief 按弧长采样参考朝向（带角度环绕处理）
 * @param dist 距离路径起点的弧长
 * @return 参考yaw（单位rad）
 */
float PathChaserType::_sample_yaw(float dist) const
{
    if (_waypoints == nullptr || _waypoint_count == 0) return 0.0f;
    if (_waypoint_count == 1) return _waypoints[0].yaw_rad;
    if (dist <= 0.0f) return _waypoints[0].yaw_rad;

    float acc = 0.0f;
    for (uint16_t i = 0; i + 1 < _waypoint_count; i++)
    {
        Vec2 a = _waypoints[i].pos;
        Vec2 b = _waypoints[i + 1].pos;
        float seg_len = (b - a).Length();
        if (acc + seg_len >= dist)
        {
            float t = (seg_len < 1e-6f) ? 0.0f : (dist - acc) / seg_len;
            float yaw0 = _waypoints[i].yaw_rad;
            float yaw1 = _waypoints[i + 1].yaw_rad;
            float dyaw = _wrap_pi(yaw1 - yaw0);
            return _wrap_pi(yaw0 + dyaw * t);
        }
        acc += seg_len;
    }
    return _waypoints[_waypoint_count - 1].yaw_rad;
}

/**
 * @brief 按弧长采样前馈角速度
 * @param dist 距离路径起点的弧长
 * @return 前馈omega（单位rad/s）
 */
float PathChaserType::_sample_omega(float dist) const
{
    if (_waypoints == nullptr || _waypoint_count == 0) return 0.0f;
    if (_waypoint_count == 1) return _waypoints[0].omega_rad;
    if (dist <= 0.0f) return _waypoints[0].omega_rad;

    float acc = 0.0f;
    for (uint16_t i = 0; i + 1 < _waypoint_count; i++)
    {
        Vec2 a = _waypoints[i].pos;
        Vec2 b = _waypoints[i + 1].pos;
        float seg_len = (b - a).Length();
        if (acc + seg_len >= dist)
        {
            float t = (seg_len < 1e-6f) ? 0.0f : (dist - acc) / seg_len;
            return _waypoints[i].omega_rad * (1.0f - t) + _waypoints[i + 1].omega_rad * t;
        }
        acc += seg_len;
    }
    return _waypoints[_waypoint_count - 1].omega_rad;
}

/**
 * @brief 单周期更新控制输出（200Hz调用）
 * @details 执行投影前瞻、前馈+反馈、限幅、坐标变换与终点判定
 * @param now_pose 当前位姿 (x, y, yaw)
 */
void PathChaserType::_update_cmd(const Vec3& now_pose)
{
    // 路径点不足 或 已完成，就不给速度
    if (_waypoints == nullptr || _waypoint_count == 0 || _finished || !_path_valid)
    {
        _cmd.world = Vec3(0, 0, 0);
        _cmd.body = Vec3(0, 0, 0);
        return;
    }

    float now_rt = System.runtime_tick;
    float dt = now_rt - _last_runtime_sec;
    _last_runtime_sec = now_rt;
    if (dt < 0.0f) dt = 0.0f;
    if (dt > 0.05f) dt = 0.05f;

    _elapsed_sec = now_rt - _start_runtime_sec;
    if (_elapsed_sec < 0.0f) _elapsed_sec = 0.0f;

    // 获取当前二维位置
    Vec2 now_pos = now_pose.ToVec2();
    _s_meas = _solve_progress(now_pos);

    // 时间参考进度
    _s_ref = _sample_s_by_time(_elapsed_sec);

    // 基于 s_cmd 采样参考速度，融合推进更新 s_cmd
    Vec2 vel_on_cmd = _sample_vel(_s_cmd);
    float v_ref = vel_on_cmd.Length();
    _s_dot_cmd = v_ref
               + _k_sync_time * (_s_ref - _s_cmd)
               + _k_sync_meas * (_s_meas - _s_cmd);
    _s_dot_cmd = StdMath::fclamp(_s_dot_cmd, _min_s_dot, _max_s_dot);
    _s_cmd += _s_dot_cmd * dt;
    _s_cmd = StdMath::fclamp(_s_cmd, 0.0f, _path_total_len);

    // 按 s_cmd 采样目标状态（位置、速度、姿态、角速度）
    Vec2 targ_pos = _sample_pos(_s_cmd);
    Vec2 vel_ff = _sample_vel(_s_cmd);
    float yaw_ref = _sample_yaw(_s_cmd);
    float omega_ff = _sample_omega(_s_cmd);

    // 计算误差
    Vec2 pos_err = targ_pos - now_pos;
    float yaw_err = _wrap_pi(yaw_ref - now_pose.z);

    // 叠加前馈与反馈，反馈项单独限幅，避免误差导致长时间满速追赶
    Vec2 pos_fb = _limit_vec2(pos_err * _kp_pos, _max_pos_fb);
    Vec2 v_world = vel_ff + pos_fb;
    v_world = _limit_vec2(v_world, _max_vel);

    // 角速度前馈+反馈
    float yaw_fb = StdMath::fclamp(_kp_yaw * yaw_err, -_max_yaw_fb, _max_yaw_fb);
    float omega = omega_ff + yaw_fb;
    omega = StdMath::fclamp(omega, _max_omega);

    // 转换到车体系（使用本周期新计算的世界系速度，避免一拍延迟）
    Vec2 body_xy = v_world.Rotate(-now_pose.z);

    // 填写控制指令
    _cmd.world = Vec3(v_world.x, v_world.y, omega);
    _cmd.body = Vec3(body_xy.x, body_xy.y, _cmd.world.z);

    // 终点完成判定，先获取终点状态
    Vec2 final_pos = _waypoints[_waypoint_count - 1].pos;
    float final_yaw = _waypoints[_waypoint_count - 1].yaw_rad;

    // 计算位置误差 和 角度误差
    float pos_err_len = (final_pos - now_pos).Length();
    float final_yaw_err = fabsf(_wrap_pi(final_yaw - now_pose.z));

    // 完成条件：时间接近末端 + 位置姿态进入阈值
    bool time_ready = _elapsed_sec >= (_path_total_time - _finish_time_margin);
    if (time_ready && pos_err_len < _pos_tol && final_yaw_err < _yaw_tol)
    {
        // 如果还没计时结束，继续加
        if (_finish_tick_cnt < _finish_hold_ticks) _finish_tick_cnt++;
    }
    else
    {
        _finish_tick_cnt = 0;
    }

    // 如果满足阈值持续一段时间，认为抵达
    if (_finish_tick_cnt >= _finish_hold_ticks)
    {
        _finished = true;
        _cmd.world = Vec3(0, 0, 0);
        _cmd.body = Vec3(0, 0, 0);
    }
}
