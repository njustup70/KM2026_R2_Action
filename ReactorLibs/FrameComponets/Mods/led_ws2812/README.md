# led_ws2812 使用说明（App层）

## 模块定位
`LedWs2812` 是一个基于 `PWM + DMA` 的 WS2812 灯带驱动，提供：
- 单点设色：`SetColor`
- 整带填充：`Fill`
- 预设效果：`Breath / GradientFlow / Running / Lit / Expand / Blink / ProgBar`
- 循环维护：`Handle`
- 底层下发：`Upload`

推荐流程是两步：
1. 在需要切换灯效的地方，单次调用 `Breath / GradientFlow / Running / Expand / Fill / Lit / SetColor`
2. 在某个固定循环里持续调用 `Handle()`，由模块自动推进动画并按需上传

`Upload()` 仍然保留，供需要直接控制底层发送时使用。

## 模板参数与内存
类定义是模板：
```cpp
template <size_t MaxLedNumsT = 16>
class LedWs2812;
```

`MaxLedNumsT` 既决定最大/实际灯数，也决定RAM占用：
- 缓冲区大小约为：`MaxLedNumsT * 24 * 4 bytes`
- 例如 `LedWs2812<8>` 比 `LedWs2812<16>` 缓冲区约减半

建议按真实灯数声明实例，不要默认吃满16。

## 快速开始

> 注：作为一个MOD模块，其只能在更高的层级中调用

### 在 App 类里声明实例
```cpp
#include "led_ws2812.hpp"

class MyLightApp : public Application
{
    SINGLETON(MyLightApp)
    APPLICATION_OVERRIDE(MyLightApp)

private:
    LedWs2812<13> led;   // 13颗灯珠
};
```

### 在 `Start()` 初始化一次
```cpp
void MyLightApp::Start()
{
    led.Init(Hardware::tim_led, TIM_CHANNEL_1);
    led.BiasFactor = Vec3(1.0f, 1.0f, 1.0f); // 可选：颜色校正
}
```

### 在 `Update()` 维护
```cpp
void MyLightApp::Update()
{
    led.Handle();
}
```
### 在需要切换效果的地方单次调用
```cpp
void MyLightApp::Start()
{
    led.Init(Hardware::tim_led, TIM_CHANNEL_1);
    led.BiasFactor = Vec3(1.0f, 1.0f, 1.0f);
    led.Breath(Color(32, 128, 255), 2.0f);
}
```

### Blink 用法
```cpp
led.Blink(Color(a, b, c), 3, 0.5f);
```

这表示灯带会按 50% 占空比闪烁 3 次，单个闪烁节拍为 0.5s；完成后会再保持熄灭 1.5s，然后重新进入下一轮闪烁。

### 进度条用法
```cpp
float progress = 0.0f;
led.ProgBar(Color(0, 255, 0), &progress);
```

`ProgBar()` 会持续读取你传入的 `float*`，并把数值按 `0.0f ~ 1.0f` 归一化后映射到整条灯带。`0.0f` 时全灭，`1.0f` 时整条点亮，过程中会在前沿保留一个过渡灯珠，视觉上更像真正的进度条。

#### 额外包装
也可以根据自己的需求，再额外包装几层

注意：
- 新推荐用法下，动画和静态效果都由 `Handle()` 自动下发，不需要每轮手写 `Upload()`
- 如果你仍然想走底层直控方式，`Fill()/SetColor()/动画函数` 后手动 `Upload()` 依然可用

## 注意事项（很重要）
1. `Init()` 只需调用一次，通常放 `Start()`。
2. `Handle()` 所在的外部循环不要阻塞；灯效靠周期调用推进内部时间。
3. DMA完成回调已由 BSP `pwm` 子系统统一分发，支持按 `TimID + channel` 路由。
4. `Upload()` 依赖定时器/PWM/DMA配置正确；若不亮，优先检查 CubeMX 的 TIM+DMA 配置和通道匹配。
