# PathChaser

`PathChaser` 是一个轻量路径跟踪库。  
输入是一组离线路点（包含位置、姿态、前馈平移速度、前馈角速度、时间戳），输出是当前时刻“建议控制量”。

本模块 **不直接控制底盘**，只负责 **产出命令**；上层可自行决定如何使用这些命令。

## 使用示例

```cpp
// 提前写好的路点
static const Path<4> demo_path = {{
    {{0.0f, 0.0f}, 0.0f, {0.4f, 0.0f}, 0.0f, 0.0f},
    {{0.5f, 0.0f}, 0.0f, {0.4f, 0.0f}, 0.0f, 1.2f},
    {{1.0f, 0.2f}, 0.1f, {0.3f, 0.1f}, 0.2f, 2.6f},
    {{1.2f, 0.4f}, 0.2f, {0.0f, 0.0f}, 0.0f, 3.0f},
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
    float t_sec;    // 时间戳（s）
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

用当前位置做最近点投影，得到实测进度 `s_meas`

按运行时间 `t_now` 映射得到参考进度 `s_ref`

融合推进得到控制进度 `s_cmd`：
`s_cmd_dot = v_ref + k_t * (s_ref - s_cmd) + k_m * (s_meas - s_cmd)`

按 `s_cmd` 采样目标状态：`pos_ref / vel_ff / yaw_ref / omega_ff`

计算误差并生成命令：
- `v_world = vel_ff + pos_fb`（`pos_fb = kp_pos * pos_err` 后单独限幅）
- `omega = omega_ff + yaw_fb`（`yaw_fb = kp_yaw * yaw_err` 后单独限幅）
- 再做全局线速度与角速度限幅

时间接近末端且终点误差满足阈值保持一段时间后，判定完成

## 对外接口

`PathChaserType` 提供的核心接口：

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
