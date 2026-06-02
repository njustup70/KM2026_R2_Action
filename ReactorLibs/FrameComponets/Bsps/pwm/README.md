# bsp_tim_pwm（TimID版）
## 快速开始
### 映射硬件
在 `Hardware::Config_Hardwares()` 中映射：
   - `Hardware::tim_led = ToID(&htim4);`

### 使用普通PWM波


业务模块使用时带上通道：
   - 例如 `TIM_CHANNEL_1`
1. 普通PWM：
   - `BSP::TIM::Enable` / `BSP::TIM::SetDuty` / `BSP::TIM::Disable`
2. DMA发送：
   - `BSP::TIM::RegisterPulseFinishedCallback`
   - `BSP::TIM::StartDma`
   - 在回调里按需 `BSP::TIM::StopDma`

## 约束
- HAL 回调 `HAL_TIM_PWM_PulseFinishedCallback` 只在 BSP 内实现。
- MOD/APP 不应直接调用 `HAL_TIM_*`。

