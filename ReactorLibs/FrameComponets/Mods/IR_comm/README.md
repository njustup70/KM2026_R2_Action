# IR 红外通信模块 (NEC 协议)

## 硬件引脚

| 功能 | 引脚 | 外设 |
|------|------|------|
| TX 发射 | PE12 | TIM1_CH3N (38kHz PWM + DMA) |
| RX 接收 | PA0 | EXTI0 (双边沿中断) |

---

## 发送

### 函数接口

```cpp
// 发送一帧 NEC 数据
// address: 0x00 ~ 0xFF
// command: 0x00 ~ 0xFF
APP::main_loop.SendIR(address, command);
```

### 示例

```cpp
APP::main_loop.SendIR(0x01, 0xA5);
APP::main_loop.SendIR(0x01, 0x01);
```

### 发送日志

发送成功（DMA 完成）后输出：

```
[IR_TX] #1 sent OK addr=0x01 cmd=0xA5
```

DMA 忙导致发送失败：

```
[IR_TX] #1 send FAILED (busy) addr=0x01 cmd=0x01
```

---

## 接收

接收完全自动，**无需额外调用**。成功解码一帧后输出日志：

```
[IR_RX] #28 recv addr=0x01 cmd=0x01 repeat=0
```

每条日志包含：序号、地址、命令、重复码标志，可用于跟踪丢帧情况。

### 接收回调处理

在 `MainLoop::Start()` 中已注册回调，如需自定义处理逻辑，在回调内添加：

```cpp
ir_rx.RegisterRxHandler([](const IR_RX *rx,
                            uint8_t address, uint8_t command,
                            bool is_repeat, void *user_ctx) {
    // 在此处添加你的业务逻辑
    if (address == 0x01 && command == 0xA5) {
        // 执行对应动作
    }
}, this);
```

---

## 安装注意事项

- **距离和角度敏感**：测试发现距离稍远或接收头未正对发射管时，很容易解码失败（无任何日志输出、无报错、直接收不到）
- 建议红外发射管与接收头尽量**正对安装**
- 控制使用距离，留足余量
- 如果安装空间受限导致角度偏大，建议加装**红外聚光罩**或选用**广角接收头**
