# ActionChain 库使用文档


## 基本介绍

- 在机器人比赛中，机器人可能会执行由一连串基本动作组成的 __动作链__，本库将一个动作链视为类进行封装
- 采用 __链式法则__ 构建动作链
- 库中方法已经给出 __日志说明__ 用于调试，用户无需重复编写

## 快速开始

### 定义动作函数

:warning:__要求：无参数，返回bool__

__例如：__
```
bool OpenClaw()
{
    APP::jaw.OpenClaw();
    return true;
}

bool MoveToTable() 
{
    // 可能会失败的动作
    return MOVE::MoveToTargPos(Path_To_Table);
}
```
### 动作链取名

#### 方法一：构造函数时
```
void SimpleMission(StateCore *core)
{
    bool success = ActionChain("SimpleMission")
}
```
#### 方法二：链式调用时
```
void SimpleMission(StateCore *core)
{
    bool success = ActionChain()
        .SetName("SimpleMission")
}
```

### 构建动作链

#### 示例一：无须重试的简单动作
```
void SimpleMission(StateCore *core)
{
    bool success = ActionChain("SimpleMission")
        .Add("Open Claw", OpenClaw)
        .Add("Close Claw", CloseClaw)
        .Run();
}
```
#### 示例二：带重试的动作
```
void ReliableMission(StateCore *core)
{
    bool success = ActionChain("ReliableMission")
        .Add("Move to Table", MoveToTable, 3, 2.0f)  // 重试3次，间隔2秒
        .Add("Close Claw", CloseClaw, 1)              // 重试1次（不重试）
        .Run();
}
```
### 外部接口
| 方法 | 参数 | 说明 |
|------|------|------|
| `Add(name, func, retries, delay)` | `name`：动作名称<br>`func`：动作函数<br>`retries`：重试次数（默认1）<br>`delay`：重试间隔秒数（默认1.0） | 添加一个动作，返回自身引用 |
| `Run()` | 无 | 执行所有动作，返回是否全部成功 |
| `Clear()` | 无 | 清空所有动作 |
| `SetName(name)` | `name`：新名称 | 修改动作链名称 |
| `GetName()` | 无 | 获取动作链名称 |
| `GetCount()` | 无 | 获取动作数量 |
| `IsEmpty()` | 无 | 检查是否为空 |

### 注意事项
- __动作函数必须返回 bool：__ true 表示成功，false 表示失败

- __重试机制：__ 失败后会自动重试，全部重试失败则整个链失败

- __失败即停：__ 任何一个动作失败后，后续动作不会执行

- __链名称：__ 建议使用有意义的名称，便于日志调试

- __最大动作数：__ 默认 MAX_ACTIONS = 10，可修改