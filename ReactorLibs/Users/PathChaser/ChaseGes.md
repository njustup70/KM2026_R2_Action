# 普通跑点 ChaseGes

`ChaseGes` 是 `PathChaser` 里的普通跑点模式，适合不想提前生成路径、只想让机器人 PID 到某个点的场景。

它仍然保持原来的调用习惯：先调用一次 `ChaseGes / ChasePos / ChaseYaw` 设置目标，然后在循环里不断读取 `GetCmdBody()` 下发到底盘，直到 `IsFinished()` 为 true。

## 基本用法

```cpp
APP::path_chaser.ChaseGes(Vec3(1.0f, 0.5f, 1.57f));

while (!APP::path_chaser.IsFinished())
{
    Vec3 speed_vec = APP::path_chaser.GetCmdBody();
    APP::chas.Move(speed_vec, 0.01f);
    Seq::Wait(0.005f);
}
```

`ChaseGes(Vec3(x, y, yaw))` 会同时控制世界系位置和世界系 yaw。

## 只跑位置

如果动作过程中不想主动转头，可以只控制 xy：

```cpp
APP::path_chaser.ChasePos(Vec2(1.0f, 0.5f));

while (!APP::path_chaser.IsFinished())
{
    APP::chas.Move(APP::path_chaser.GetCmdBody(), 0.01f);
    Seq::Wait(0.005f);
}
```

## 只转 yaw

```cpp
APP::path_chaser.ChaseYaw(1.57f);

while (!APP::path_chaser.IsFinished())
{
    APP::chas.Move(APP::path_chaser.GetCmdBody(), 0.01f);
    Seq::Wait(0.005f);
}
```

## 跑到一半再转头

推荐先用 `ChasePos()` 跑位置，再用 `SetYawTarget()` 在中途挂上 yaw 目标。

```cpp
APP::path_chaser.ChasePos(Vec2(1.0f, 0.5f));

while (!APP::path_chaser.IsFinished())
{
    if (APP::path_chaser.GetRemainDist() < 0.4f)
    {
        APP::path_chaser.SetYawTarget(1.57f);
    }

    APP::chas.Move(APP::path_chaser.GetCmdBody(), 0.01f);
    Seq::Wait(0.005f);
}
```

`SetYawTarget()` 可以在循环里重复调用同一个目标角，不会反复重置完成计数。

## 控制思路

普通跑点不是完整路径规划，它只根据当前误差实时给速度命令：

```cpp
pos_err = target_pos - now_pos;
dir = pos_err.Norm();

v_by_p = kp_pos * dist;
v_limit = max_vel * smoothstep(dist / slow_dist);
v = min(v_by_p, v_limit);

v_world = slew_limit(last_v_world, dir * v, max_acc * dt);
```

所以它比纯 `kp * err` 多了两层保护：

- `smoothstep` 减速包络：接近目标时速度自然收下来，有一点 S-Curve 的感觉。
- 速度变化率限幅：起步不会一上来给满速度，减少冲劲。

yaw 也是同样思路：P 控制、近目标收角速度、角速度变化率限幅。

## 完成判定

完成条件复用 `PathChaser` 原有阈值：

- 位置误差小于 `_pos_tol`
- yaw 误差小于 `_yaw_tol`
- 连续满足 `_finish_hold_ticks` 个周期

如果调用的是 `ChasePos()`，yaw 不参与完成判定；如果调用的是 `ChaseYaw()`，位置不参与完成判定。

## 调参入口

普通跑点参数在 `PathChaser.hpp` 里，以 `_point_` 开头：

- `_point_kp_pos`：位置 P 增益
- `_point_kp_yaw`：yaw P 增益
- `_point_max_vel`：最大平移速度
- `_point_min_vel`：减速段最低有效速度，太小会尾段发软
- `_point_max_omega`：最大角速度
- `_point_min_omega`：最低有效角速度
- `_point_max_acc`：平移速度变化率上限，越小起步越柔
- `_point_max_alpha`：角速度变化率上限，越小转头越柔
- `_point_slow_dist`：小于该距离后开始明显收平移速度
- `_point_slow_yaw`：小于该角度后开始明显收角速度

建议先调 `_point_max_vel / _point_max_omega` 定安全速度，再调 `_point_max_acc / _point_max_alpha` 定起步手感，最后微调 `_point_kp_pos / _point_kp_yaw` 和减速距离。

## 注意

`ChaseGes()`、`ChasePos()`、`ChaseYaw()` 是启动函数，通常只在进入状态时调用一次。循环里只读 `GetCmdBody()` 并下发。

中途改 yaw 用 `SetYawTarget()`，不要在跑位置的循环里切到 `ChaseYaw()`，否则平移控制会被新的只转头任务替换。
