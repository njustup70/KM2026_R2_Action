# 红外通信模块使用说明 (NEC 协议)

## 1. 硬件引脚配置

### 发射端 — PA8（TIM1_CH1）

CubeMX 配置：

```
PA8 → TIM1_CH1
```

TIM1 参数（与 PE12 旧 IR_TX 复用同一组）：

| 参数 | 值 | 说明 |
|------|-----|------|
| Prescaler | 179 | 180MHz / 180 = 1MHz |
| Counter Period | 25 | 1MHz / 26 = 38.46 kHz |
| AutoReloadPreload | Disable | |
| CH1 PWM Mode | PWM Mode 1 | |
| CH1 Pulse | 13 | 50% 占空比 |

TIM1_UP DMA：

| 参数 | 值 |
|------|-----|
| DMA 流 | DMA2_Stream5 |
| 通道 | Channel 6 |
| 方向 | Memory to Peripheral |
| 数据宽度 | Half Word |
| NVIC | 无需单独使能（使用 NDTR 轮询） |

> **注意**：TIM1 是高级定时器，使用 `HAL_TIM_PWM_Start` 后必须手动置 `BDTR.MOE` 位，否则输出被硬件禁止。

### 接收端 — PC1（GPIO_EXTI1）

CubeMX 配置：

```
PC1 → GPIO_EXTI1
  GPIO mode:     External Interrupt Mode with Rising/Falling edge trigger
  GPIO Pull:     Pull-up
  NVIC:          EXTI line1 interrupt → Enable
```

> 可根据实际硬件情况换成其他支持 EXTI 的 GPIO 引脚。

---

## 2. BSP 层所需修改

移植到新工程时，以下 BSP 文件需确保已修改：

### 2.1 `bsp_tim_pwm.cpp` — 两处修改

**修改 A**：`StartUpdateDma` 和 `StopUpdateDma` 中去掉 `channel == 0` 校验（`BSP::TIM::CH1` 值为 `0x00`，旧校验误判为非法）。

**修改 B**：`StartUpdateDma` 中 DMA 必须**先于**定时器启动：

```cpp
// ✅ 正确顺序
HAL_DMA_Start(...);           // 1. 先配 DMA
htim->Instance->DIER |= TIM_DIER_UDE;
HAL_TIM_PWM_Start(htim, ch);  // 2. 再开定时器
htim->Instance->BDTR |= TIM_BDTR_MOE;  // 3. 开 MOE
```

**修改 C**：CCR 寄存器地址按 channel 动态解析（不再硬编码 `CCR3`）。

### 2.2 `IR_TX.cpp` — 一处修改

`Init()` 中去掉 `channel == 0` 校验，只保留 `tim_id == nullptr`。

### 2.3 调用方 `SendIR` — 帧间 DMA 重置

每帧发送前必须停止 PWM、销毁并重建 DMA 流：

```cpp
while (hdma_tim1_up.Instance->NDTR > 0) { Seq::Wait(0.001); }
HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
HAL_DMA_DeInit(&hdma_tim1_up);
HAL_DMA_Init(&hdma_tim1_up);
ir_tx.Send(address, command);
```

### 2.4 IR_TX 帧尾追加关载波

`_BuildFrame` 末尾追加 5ms 关载波，确保 DMA 结束后 IR LED 彻底关闭：

```cpp
_FillPulse(bit_on_us, true);   // Stop bit
_FillPulse(5000, false);       // 5ms 尾随关载波
```

---

## 3. 使用方法

### 3.1 初始化

发送端初始化建议按下面顺序做，避免因为时序或底层句柄未就绪导致 `IR_TX` 无法正常工作：

1. 先在 CubeMX 中完成 PA8/TIM1_CH1 和 TIM1_UP DMA 的底层配置，并确保 `htim1`、`hdma_tim1_up` 已生成。
2. 在工程启动流程中，等 HAL 和外设初始化完成后，再创建 `IR_TX` 对象并调用初始化接口。
3. 初始化时使用 TIM1 句柄和通道 1：`ir_tx.Init(ToID(&htim1), BSP::TIM::CH1);`。
4. 若工程里需要复用同一套发送逻辑，调用发送前要保证上一帧 DMA 已结束，并按本文件第 2.3 节的顺序重置 DMA/停止 PWM。
5. 初始化完成后即可直接调用 `ir_tx.Send(addr, cmd);` 发送 NEC 帧。

```cpp
#include "IR_RX.hpp"
#include "IR_TX.hpp"
#include "tim.h"

IR_RX ir_rx;
IR_TX ir_tx;

void Init()
{
    // 接收端
    ir_rx.Init(Pin{'C', 1});        // PC1, 可换其他 EXTI 引脚
    ir_rx.local_addr = 0xFF;        // 0xFF = 接收所有地址
    ir_rx.RegisterRxHandler([](const IR_RX* rx,
                                uint8_t addr, uint8_t cmd,
                                bool repeat, void* ctx) {
        // 收到一帧后的处理逻辑
    }, nullptr);

    // 发射端
    ir_tx.Init(ToID(&htim1), BSP::TIM::CH1);
}
```

### 3.2 发送

```cpp
void SendIR(uint8_t addr, uint8_t cmd)
{
    extern DMA_HandleTypeDef hdma_tim1_up;

    while (hdma_tim1_up.Instance->NDTR > 0) { /* 等上次 DMA 完成 */ }
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_DMA_DeInit(&hdma_tim1_up);
    HAL_DMA_Init(&hdma_tim1_up);

    ir_tx.Send(addr, cmd);  // 非阻塞，DMA 后台发送
}
```

### 3.3 接收超时保护

在 `Update()` 中定期调用：

```cpp
ir_rx.Handle();  // 状态机卡住超过 100ms 自动复位
```

---

## 4. 使用范围与限制

| 项目 | 说明 |
|------|------|
| **角度对准** | 发射端与接收端必须大致对准，**不可有太大角度偏差**（建议 ±15° 以内），红外方向性强 |
| **有效距离** | **60~70 cm**，满足场地内通信需求 |
| **环境光** | 强光直射（如太阳光、大功率灯光）可能干扰，尽量遮挡 |
| **遮挡物** | 红外无法穿透非透明物体，需保持视线直达 |
| **通信速率** | NEC 帧约 78ms/帧，含帧间间隔建议 ≥150ms |
| **丢帧率** | 正常条件下 <10%，可通过重复发送保证可靠性 |
