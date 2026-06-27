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
    APP::path_chaser.ChasePath(demo_path);
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

当前版本是按时间 `t` 的纯跟踪算法：

1. `ChasePath()` 记录启动时刻，并检查路径时间戳是否合法。
2. 每个 `Update()` 用 `System.runtime_tick - _start_runtime_sec` 得到路径相对时间。
3. 在相邻路点之间按 `t_sec` 插值参考 `pos / yaw / vel_vec / omega_rad`。
4. 平移控制使用“速度前馈 + 位置反馈”。
5. 姿态控制使用“角速度前馈 + yaw反馈”。
6. 误差较大时自动压低前馈，防止机器人偏离后继续猛追远处参考点。
7. 时间到达路径末端，并且终点位置/yaw 连续满足阈值后，判定完成。

核心公式：

```cpp
pos_err = ref.pos - now_pos;
yaw_err = wrap_pi(ref.yaw - now_yaw);

pos_fb = limit_vec2(_kp_pos * pos_err, _max_pos_fb);
yaw_fb = clamp(_kp_yaw * yaw_err, _max_yaw_fb);

ff_scale = _ff_gain * error_slowdown_scale;

v_world = limit_vec2(ref.vel_vec * ff_scale + pos_fb, _max_vel);
omega = clamp(ref.omega_rad * ff_scale + yaw_fb, _max_omega);
```

其中 `ref.vel_vec` 是世界系速度，`GetCmdBody()` 会把平移速度旋转到车体系，兼容当前底盘接口。

## 参数说明

### `_kp_pos = 1.8f`

位置误差反馈增益。

- 增大：偏离参考点后回收更快
- 过大：可能抖动、过冲
- 过小：跟点软，误差消除慢

### `_kp_yaw = 3.2f`

姿态误差反馈增益。

- 增大：朝向跟随更快
- 过大：车头容易摆
- 过小：转角和终点姿态会滞后

### `_ff_gain = 1.0f`

前馈增强系数。

实车上如果低层速度环偏软、摩擦大、执行有延迟，可以小幅调到 `1.05f ~ 1.20f`。
不建议默认给太大，因为最终会被限幅截断，且弯道和终点会更激进。

### `_max_vel = 1.0f`

最终世界系平移合速度上限，单位 `m/s`。

### `_max_omega = 1.0f`

最终角速度上限，单位 `rad/s`。

### `_max_pos_fb = 0.45f`

位置反馈速度上限，单位 `m/s`。

它限制的是反馈项，不限制路径前馈。偏离后回收太猛时先减小它；回收太慢时再增大它或 `_kp_pos`。

### `_max_yaw_fb = 1.0f`

yaw 反馈角速度上限，单位 `rad/s`。

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
3. 调 `_kp_pos / _max_pos_fb`，让位置误差能收敛且不过冲。
4. 调 `_kp_yaw / _max_yaw_fb`，让车头跟随稳定。
5. 如果整段路径总是慢一点，再小幅提高 `_ff_gain`。
6. 如果偏离后追得太凶，减小 `_min_ff_scale` 或减小 `_max_pos_fb`。

## 常见现象

- 车总是慢半拍：小幅增大 `_ff_gain`，例如 `1.05f`。
- 车一偏就猛追：减小 `_max_pos_fb` 或 `_min_ff_scale`。
- 车到终点冲过：减小 `_ff_gain`、`_max_vel`，或放大路径末端减速段。
- 转头太猛：减小 `_kp_yaw`、`_max_yaw_fb` 或 `_max_omega`。
- 到终点不结束：放宽 `_pos_tol / _yaw_tol`，或确认路径最后一个点的 yaw 正确。

## 对外接口

- `ChasePath(const Path<N>& path)`：设置路径并重置状态
- `Reset()`：手动重置内部状态
- `IsFinished()`：是否完成路径
- `GetCmdWorld()`：世界系命令 `(vx, vy, omega)`
- `GetCmdBody()`：车体系命令 `(vx, vy, omega)`
- `GetCmd()`：同时获取世界系 + 车体系命令

全局实例：

```cpp
APP::path_chaser
```
