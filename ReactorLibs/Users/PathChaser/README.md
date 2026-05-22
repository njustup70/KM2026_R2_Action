# PathChaser

`PathChaser` 是一个轻量路径跟踪库。  
输入是一组离线路点（包含位置、姿态、前馈平移速度、前馈角速度），输出是当前时刻“建议控制量”。

本模块 **不直接控制底盘**，只负责 **产出命令**；上层可自行决定如何使用这些命令。

## 使用示例

```cpp
// 提前写好的路点
static const Path<4> demo_path = {{
    {{0.0f, 0.0f}, 0.0f, {0.4f, 0.0f}, 0.0f},
    {{0.5f, 0.0f}, 0.0f, {0.4f, 0.0f}, 0.0f},
    {{1.0f, 0.2f}, 0.1f, {0.3f, 0.1f}, 0.2f},
    {{1.2f, 0.4f}, 0.2f, {0.0f, 0.0f}, 0.0f},
}};

void StateAct_1()
{
    // 启动路径追踪，直接生效
    APP::path_chaser.ChasePath(demo_path);

    // 转移到 StateAct_2 ...
}

void StateAct_2()
{
    Vec3 cmd_body = APP::path_chaser.GetCmdBody();

    // 由用户决定是否直接下发，或用作其他用途
    // APP::chas.Move(cmd_body);   
}
```

## 数据结构

## 路点

```cpp
struct PathWaypoint
{
    Vec2 pos;       // 位置（m）
    float yaw_rad;  // 朝向（rad）
    Vec2 vel_vec;   // 前馈平移速度（m/s）
    float omega_rad;// 前馈角速度（rad/s）
};
```

## 路径

```cpp
template <size_t waypointCount = 256>
class Path
{
public:
    PathWaypoint waypoints[waypointCount];
};
```

说明：
- `Path` 是编译期定长数组，避免动态分配。
- 建议按 `static const Path<N>` 方式定义，减小 RAM 压力。

## 跟踪逻辑

每个周期主要流程如下：

用当前位置在路径折线中做最近点投影，得到当前弧长进度 `s`

取前瞻进度 `s_targ = s + lookahead_dist`

按 `s_targ` 采样目标状态：`pos / vel_ff / yaw_ref / omega_ff`

计算误差：
   - `pos_err = pos_targ - pos_now`
   - `yaw_err = wrap_pi(yaw_ref - yaw_now)`
   生成输出命令：
   - `v_world = vel_ff + kp_pos * pos_err`
   - `omega = omega_ff + kp_yaw * yaw_err`
   - 线速度矢量限幅 + 角速度限幅

判断是否到达终点（阈值保持一段时间）

## 对外接口

`PathChaserType` 提供的核心接口：

- `SetPath(const Path<N>& path)`：设置路径并重置状态
- `Reset()`：手动重置内部状态
- `IsFinished()`：是否完成路径
- `GetCmdWorld()`：世界系命令 `(vx, vy, omega)`
- `GetCmdBody()`：车体系命令 `(vx, vy, omega)`
- `GetCmd()`：同时获取世界系 + 车体系命令

全局实例：

```cpp
APP::path_chaser
```