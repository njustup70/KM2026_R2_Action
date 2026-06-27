# PathChaser

`PathChaser` 是一个轻量路径跟踪库。
输入是一组离线路点（位置、姿态、世界系速度前馈、角速度前馈、时间戳），输出当前时刻的建议速度命令。

本模块不直接控制底盘，只负责产出命令；上层可决定是否直接下发给 `APP::chas.Move()`。

## 使用示例

```cpp
static const Path<4> demo_path = {{
    {{0.0f, 0.0f}, 0.0f, {0.0f, 0.0f}, 0.0f, 0.0f},
    {{0.5f, 0.0f}, 0.0f, {0.4f, 0.0f}, 0.0f, 1.2f},
    {{1.0f, 0.2f}, 0.1f, {0.3f, 0.1f}, 0.2f, 2.6f},
    {{1.2f, 0.4f}, 0.2f, {0.0f, 0.0f}, 0.0f, 3.0f},
}};

void StateAct_1()
{
    APP::path_chaser.ChasePath(demo_path);         // 默认：时间参考主导
    // APP::path_chaser.ChasePath(demo_path, false); // 可选：路径进度/前瞻主导
}

void StateAct_2()
{
    Vec3 cmd_body = APP::path_chaser.GetCmdBody();
    APP::chas.Move(cmd_body);
}
```

## 路点格式

```cpp
struct PathWaypoint
{
    Vec2 pos;        // 世界坐标位置，单位m
    float yaw_rad;   // 世界坐标朝向，单位rad
    Vec2 vel_vec;    // 世界坐标速度前馈，单位m/s
    float omega_rad; // yaw角速度前馈，单位rad/s
    float t_sec;     // 时间戳，单位s
};
```

`Path<N>` 是编译期定长数组，建议用 `static const Path<N>` 定义，避免占用过多 RAM。

## 跟踪逻辑

默认版本是时间参考主导算法：

1. `ChasePath(path)` 记录路径并重置状态，默认 `use_t_ref = true`。
2. 每个 `Update()` 用 `System.runtime_tick - _start_runtime_sec` 得到路径相对时间。
3. 按 `t_sec` 插值参考 `pos / yaw / vel_vec / omega_rad`。
4. 平移控制使用“速度前馈 + 位置反馈”。
5. 姿态控制使用“角速度前馈 + yaw反馈”。
6. 误差较大时自动压低前馈，末端时间窗内逐渐衰减前馈。
7. 末端按终点剩余距离做刹车限速，时间到达路径末端且终点位置/yaw 连续满足阈值后，判定完成。

如果调用 `ChasePath(path, false)`，会使用路径进度/前瞻主导算法：按当前位置投影到路径折线得到 `s_nearest`，再追 `s_nearest + lookahead` 附近的参考点。

核心公式：

```cpp
ref = path(t);
pos_err = ref.pos - now_pos;
yaw_err = wrap_pi(ref.yaw - now_yaw);

pos_fb = limit_vec2(_kp_pos * pos_err, _max_pos_fb);
yaw_fb = clamp(_kp_yaw * yaw_err, _max_yaw_fb);

end_ff_scale = min(time_to_end_scale, dist_to_end_scale);
ff_scale = _ff_gain * error_slowdown_scale * end_ff_scale;

v_world = limit_vec2(ref.vel_vec * ff_scale + pos_fb, _max_vel);
v_world = limit_vec2(v_world, sqrt(2 * _time_end_max_acc * final_pos_err)); // 末端启用
omega = clamp(ref.omega_rad * ff_scale + yaw_fb, _max_omega);
```

其中 `ref.vel_vec` 是世界系速度，`GetCmdBody()` 会把平移速度旋转到车体系，兼容当前底盘接口。

## 参数说明

### `_prog_kp_along = 0.65f`

默认路径进度主导算法的沿路径方向反馈增益。它负责追前瞻点，让车继续沿路径向前走。

- 增大：偏离参考点后回收更快
- 过大：可能抖动、过冲
- 过小：跟点软，误差消除慢

### `_prog_kp_cross = 1.30f`

默认路径进度主导算法的横向贴线反馈增益。它负责把车拉回最近路径线，末端直线段的 x/y 贴合主要靠它。

### `_prog_kp_yaw = 0.8f`

默认路径进度主导算法的姿态误差反馈增益。

- 增大：朝向跟随更快
- 过大：车头容易摆
- 过小：转角和终点姿态会滞后

### `_prog_ff_gain = 1.05f`

默认路径进度主导算法的前馈增强系数。

实车上如果低层速度环偏软、摩擦大、执行有延迟，可以小幅调到 `1.05f ~ 1.20f`。
不建议默认给太大，因为最终会被限幅截断，且弯道和终点会更激进。

### `_max_vel = 1.0f`

最终世界系平移合速度上限，单位 `m/s`。

### `_max_omega = 1.0f`

最终角速度上限，单位 `rad/s`。

### `_prog_max_along_fb = 0.45f`

沿路径方向反馈速度上限，单位 `m/s`。

前瞻点拉得太猛时先减小它；前进方向回收太慢时再增大它或 `_prog_kp_along`。

### `_prog_max_cross_fb = 0.35f`

横向贴线反馈速度上限，单位 `m/s`。

末端横向过冲时优先增大 `_prog_kp_cross` 或 `_prog_max_cross_fb`；横向来回晃时减小它们。

### `_prog_max_yaw_fb = 0.5f`

yaw 反馈角速度上限，单位 `rad/s`。

### `_kp_pos / _kp_yaw / _ff_gain / _max_pos_fb / _max_yaw_fb`

这些给默认时间参考算法使用。

### `_time_end_ff_fade_time / _time_end_slow_dist`

时间参考算法的末端收敛参数。前者决定离路径结束前多久开始衰减前馈，后者决定离终点多近开始按剩余距离刹车。

### `_time_end_max_acc / _time_end_min_vel`

时间参考算法的末端刹车限速参数。过冲明显时减小 `_time_end_max_acc` 或增大 `_time_end_slow_dist`；末端太慢时反过来调。

### `_ff_slow_start_err = 0.08f`

位置误差超过该值后开始压低前馈。

### `_ff_slow_stop_err = 0.35f`

位置误差达到该值后，前馈降到 `_min_ff_scale`。

### `_min_ff_scale = 0.20f`

误差较大时保留的最小前馈比例。

如果被撞偏后希望更专注于回到参考点，可以减小它；如果希望路径节奏更强，可以增大它。

### `_pos_tol = 0.03f`

终点位置阈值，单位 `m`。

### `_yaw_tol = 0.03f`

终点姿态阈值，单位 `rad`。

### `_finish_hold_ticks = 15`

完成保持周期。200Hz 下约为 `75ms`。

## 调参建议

1. 先让 `_ff_gain = 1.0f`，确认路径能稳定跑完。
2. 调 `_max_vel / _max_omega`，先把速度安全边界定住。
3. 调 `_kp_pos / _max_pos_fb`，让中段轨迹误差能收敛且不过猛。
4. 调 `_time_end_ff_fade_time / _time_end_slow_dist`，决定末端多早开始收。
5. 调 `_time_end_max_acc`，处理终点过冲和末端耗时。
6. 调 `_kp_yaw / _max_yaw_fb`，让车头跟随稳定。

## 常见现象

- 车总是慢半拍：小幅增大 `_ff_gain`，例如 `1.05f`。
- 车一偏就猛追：减小 `_max_pos_fb` 或 `_min_ff_scale`。
- 车到终点冲过：增大 `_time_end_slow_dist / _time_end_ff_fade_time`，或减小 `_time_end_max_acc`。
- 末端太慢：增大 `_time_end_max_acc`，或减小 `_time_end_slow_dist`。
- 转头太猛：减小 `_kp_yaw`、`_max_yaw_fb` 或 `_max_omega`。
- 到终点不结束：放宽 `_pos_tol / _yaw_tol`，或确认路径最后一个点的 yaw 正确。

## 对外接口

- `ChasePath(const Path<N>& path, bool use_t_ref = true)`：设置路径并重置状态。默认时间参考主导，传 `false` 使用路径进度/前瞻算法。
- `Reset()`：手动重置内部状态
- `IsFinished()`：是否完成路径
- `GetCmdWorld()`：世界系命令 `(vx, vy, omega)`
- `GetCmdBody()`：车体系命令 `(vx, vy, omega)`
- `GetCmd()`：同时获取世界系 + 车体系命令

全局实例：

```cpp
APP::path_chaser
```
