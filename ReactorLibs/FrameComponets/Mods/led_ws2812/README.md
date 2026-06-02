# led_ws2812 使用说明（App层）

## 模块定位
`LedWs2812` 是一个基于 `PWM + DMA` 的 WS2812 灯带驱动，提供：
- 单点设色：`SetColor`
- 整带填充：`Fill`
- 预设效果：`Breath / GradientFlow / Running / Lit / Expand`
- 下发刷新：`Upload`

核心流程是两步：
1. 先改内部颜色缓冲区（`SetColor/Fill/动画函数`）
2. 再调用 `Upload()` 把缓冲区真正发到灯带

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
    // App的 Update 默认是200Hz，可以自行分频
    led.Upload();
}
```
#### 额外包装
也可以根据自己的需求，再额外包装几层

注意：动画函数只更新内存，不会自动发出；必须 `Upload()` 才会生效。

## 注意事项（很重要）
1. `Init()` 只需调用一次，通常放 `Start()`。
2. `Update()` 不要阻塞；灯效靠周期调用推进内部时间。
3. DMA完成回调已由 BSP `pwm` 子系统统一分发，支持按 `TimID + channel` 路由。
4. `Upload()` 依赖定时器/PWM/DMA配置正确；若不亮，优先检查 CubeMX 的 TIM+DMA 配置和通道匹配。
