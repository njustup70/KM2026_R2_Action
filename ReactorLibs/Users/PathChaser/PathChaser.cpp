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
    return _path_time() > 1e-6f && _path_length() > 1e-6f;
}

PathChaserType::PathFrame PathChaserType::_sample_frame(float dist) const
{
    PathFrame frame;
    if (_waypoints == nullptr || _waypoint_count == 0) return frame;

    if (_waypoint_count == 1 || _path_total_len <= 1e-6f)
    {
        frame.pos = _waypoints[0].pos;
        frame.tangent = Vec2(1.0f, 0.0f);
        frame.normal = Vec2(0.0f, 1.0f);
        frame.yaw = 0.0f;
        frame.curvature = 0.0f;
        return frame;
    }

    dist = StdMath::fclamp(dist, 0.0f, _path_total_len);

    float probe = StdMath::fclamp(_path_total_len * 0.015f, 0.03f, 0.08f);
    float s0 = StdMath::fclamp(dist - probe, 0.0f, _path_total_len);
    float s1 = dist;
    float s2 = StdMath::fclamp(dist + probe, 0.0f, _path_total_len);

    Vec2 p0 = _sample_pos(s0);
    Vec2 p1 = _sample_pos(s1);
    Vec2 p2 = _sample_pos(s2);
    Vec2 d0 = p1 - p0;
    Vec2 d1 = p2 - p1;
    Vec2 d2 = p2 - p0;

    float d2_len = d2.Length();
    if (d2_len > 1e-6f)
    {
        frame.tangent = d2 * (1.0f / d2_len);
    }
    else if (d1.Length() > 1e-6f)
    {
        frame.tangent = d1.Norm();
    }
    else if (d0.Length() > 1e-6f)
    {
        frame.tangent = d0.Norm();
    }
    else
    {
        frame.tangent = Vec2(1.0f, 0.0f);
    }

    frame.normal = Vec2(-frame.tangent.y, frame.tangent.x);
    frame.pos = p1;
    frame.yaw = atan2f(frame.tangent.y, frame.tangent.x);

    float yaw0 = atan2f(d0.y, d0.x);
    float yaw1 = atan2f(d1.y, d1.x);
    float dyaw = _wrap_pi(yaw1 - yaw0);
    frame.curvature = (probe > 1e-6f) ? (dyaw / (2.0f * probe)) : 0.0f;
    if (!std::isfinite(frame.curvature))
    {
        frame.curvature = 0.0f;
    }

    frame.s = dist;
    return frame;
}

/**
 * @brief 最近点投影求路径进度（弧长）与局部几何参考
 * @param now_pos 当前世界坐标位置
 * @return 当前投影点对应的局部路径信息
 */
PathChaserType::PathFrame PathChaserType::_solve_progress(const Vec2& now_pos)
{
    PathFrame best;
    if (_waypoints == nullptr || _waypoint_count < 2) return best;

    uint16_t seg_max = static_cast<uint16_t>(_waypoint_count - 1);
    float prefix = 0.0f;
    float best_dist2 = 1e30f;
    uint16_t best_seg = 0;

    for (uint16_t i = 0; i < seg_max; i++)
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
        bool better = (d2 < best_dist2);
        if (!better && fabsf(d2 - best_dist2) < 1e-6f)
        {
            int curr_bias = static_cast<int>(i) - static_cast<int>(_near_seg_idx);
            int best_bias = static_cast<int>(best_seg) - static_cast<int>(_near_seg_idx);
            if (curr_bias < 0) curr_bias = -curr_bias;
            if (best_bias < 0) best_bias = -best_bias;
            better = (curr_bias < best_bias);
        }
        if (better)
        {
            // 选取“投影误差最小”的段，并转换为全路径弧长进度
            best_dist2 = d2;
            best.s = prefix + seg_len * t;
            best.pos = proj;
            if (seg_len > 1e-6f)
            {
                best.tangent = ab * (1.0f / seg_len);
            }
            else
            {
                best.tangent = Vec2(1.0f, 0.0f);
            }
            best.normal = Vec2(-best.tangent.y, best.tangent.x);
            best.yaw = atan2f(best.tangent.y, best.tangent.x);
            best_seg = i;
        }
        prefix += seg_len;
    }

    // 避免实测进度大幅倒退（例如局部抖动/短时偏航），允许小范围容错回退
    if (best.s + 0.05f < _s_meas)
    {
        best.s = _s_meas - 0.05f;
    }
    if (best.s < 0.0f) best.s = 0.0f;

    _near_seg_idx = best_seg;

    // 估计局部曲率：用相邻两段的夹角除以平均弧长
    if (_waypoint_count >= 3 && seg_max >= 2)
    {
        uint16_t left_seg = (best_seg > 0) ? (best_seg - 1) : best_seg;
        uint16_t right_seg = (best_seg + 1 < seg_max) ? (best_seg + 1) : best_seg;
        if (left_seg != right_seg)
        {
            Vec2 left = _waypoints[left_seg + 1].pos - _waypoints[left_seg].pos;
            Vec2 right = _waypoints[right_seg + 1].pos - _waypoints[right_seg].pos;
            float left_len = left.Length();
            float right_len = right.Length();
            if (left_len > 1e-6f && right_len > 1e-6f)
            {
                Vec2 left_dir = left * (1.0f / left_len);
                Vec2 right_dir = right * (1.0f / right_len);
                float cross = left_dir.x * right_dir.y - left_dir.y * right_dir.x;
                float dot = left_dir * right_dir;
                float turn = atan2f(cross, dot);
                float span = 0.5f * (left_len + right_len);
                if (span > 1e-6f)
                {
                    best.curvature = turn / span;
                }
            }
        }
    }

    if (!std::isfinite(best.curvature))
    {
        best.curvature = 0.0f;
    }

    return best;
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
    PathFrame proj = _solve_progress(now_pos);
    _s_meas = proj.s;
    _s_ref = _sample_s_by_time(_elapsed_sec);

    // 以最近点为主，附加一个随速度变化的前瞻距离，形成更稳的几何跟踪目标
    Vec2 ff_at_proj = _sample_vel(_s_meas);
    float proj_ff_speed = ff_at_proj.Length();
    float lookahead = StdMath::fclamp(_lookahead_min + _lookahead_gain * proj_ff_speed, _lookahead_min, _lookahead_max);
    float target_s = StdMath::fclamp(_s_meas + lookahead, 0.0f, _path_total_len);

    // 进度平滑器：用于稳定前瞻采样点，不再承担时间主导推进
    _s_dot_cmd = StdMath::fclamp(_k_sync_meas * (target_s - _s_cmd), _min_s_dot, _max_s_dot);
    _s_cmd += _s_dot_cmd * dt;
    _s_cmd = StdMath::fclamp(_s_cmd, 0.0f, _path_total_len);
    if (_s_cmd < _s_meas)
    {
        _s_cmd = _s_meas;
    }
    float track_s = StdMath::fclamp(_s_cmd, 0.0f, _path_total_len);

    // 按平滑后的前瞻点采样前馈参考，控制本身围绕“预瞄曲线”展开
    PathFrame track = _sample_frame(track_s);
    Vec2 vel_ff = _sample_vel(track_s);

    // 只对法向误差做主反馈，避免把整个位移误差直接“拉向目标点”导致切弯
    float lat_err = (now_pos - proj.pos) * proj.normal;
    float lat_fb = StdMath::fclamp(-_kp_pos * lat_err, -_max_pos_fb, _max_pos_fb);
    Vec2 v_world = Vec2(0.0f, 0.0f);

    // 曲率越大，允许的切向速度越低
    float abs_curvature = fabsf(track.curvature);
    float curve_speed_limit = _max_vel;
    if (abs_curvature > 1e-5f)
    {
        curve_speed_limit = sqrtf(_curve_lat_accel / abs_curvature);
    }
    curve_speed_limit = StdMath::fclamp(curve_speed_limit, 0.08f, _max_vel);

    float v_t_ff = vel_ff * track.tangent;
    if (!std::isfinite(v_t_ff))
    {
        v_t_ff = 0.0f;
    }
    if (v_t_ff < 0.0f)
    {
        v_t_ff = 0.0f;
    }
    float v_t = StdMath::fclamp(v_t_ff, 0.0f, curve_speed_limit);
    v_world = track.tangent * v_t + track.normal * lat_fb;
    v_world = _limit_vec2(v_world, _max_vel);

    // 终点完成判定，先获取终点状态
    Vec2 final_pos = _waypoints[_waypoint_count - 1].pos;
    float final_yaw = _waypoints[_waypoint_count - 1].yaw_rad;

    // 计算位置误差 和 角度误差
    float pos_err_len = (final_pos - now_pos).Length();
    float final_yaw_err = fabsf(_wrap_pi(final_yaw - now_pose.z));

    // 姿态跟随路径文件中规划的 yaw/omega，适配全向底盘的任务朝向需求
    float yaw_ref = _sample_yaw(track_s);
    float yaw_err = _wrap_pi(yaw_ref - now_pose.z);
    float yaw_fb = StdMath::fclamp(_kp_yaw * yaw_err, -_max_yaw_fb, _max_yaw_fb);
    float omega_ff = _sample_omega(track_s);
    if (!std::isfinite(omega_ff))
    {
        omega_ff = 0.0f;
    }
    float omega = StdMath::fclamp(omega_ff + yaw_fb, -_max_omega, _max_omega);

    // 完成条件：几何上已进入末端窗口 + 位置姿态都满足阈值
    bool end_ready = (_path_total_len - _s_meas) <= _finish_progress_margin;

    // 进入末端窗口后，不再只做“沿路径前馈”，改为直接收敛到终点。
    // 这样即使冲过终点，也会产生反向切向速度把车拉回去，而不是继续吃最后一个前馈速度。
    if (end_ready)
    {
        Vec2 end_err = final_pos - now_pos;
        float end_t_err = end_err * track.tangent;
        float end_n_err = end_err * track.normal;

        float end_v_t = vel_ff * track.tangent + _kp_pos * end_t_err;
        float end_v_n = _kp_pos * end_n_err;

        v_world = track.tangent * StdMath::fclamp(end_v_t, -curve_speed_limit, curve_speed_limit)
                + track.normal * StdMath::fclamp(end_v_n, -_max_pos_fb, _max_pos_fb);
        v_world = _limit_vec2(v_world, _max_vel);

        float end_yaw_fb = StdMath::fclamp(_kp_yaw * _wrap_pi(final_yaw - now_pose.z), -_max_yaw_fb, _max_yaw_fb);
        omega = _sample_omega(track_s) + end_yaw_fb;
        omega = StdMath::fclamp(omega, -_max_omega, _max_omega);
    }

    // 转换到车体系（使用本周期新计算的世界系速度，避免一拍延迟）
    Vec2 body_xy = v_world.Rotate(-now_pose.z);

    // 填写控制指令
    _cmd.world = Vec3(v_world.x, v_world.y, omega);
    _cmd.body = Vec3(body_xy.x, body_xy.y, _cmd.world.z);

    if (end_ready && pos_err_len < _pos_tol && final_yaw_err < _yaw_tol)
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
