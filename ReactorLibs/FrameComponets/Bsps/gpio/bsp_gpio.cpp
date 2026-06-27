#include "bsp_gpio.hpp"
#include "bsp_halport.hpp"
#include "bsp_log.hpp"

using namespace BSP::GPIO;

/// @brief EXTI0~EXTI15 的单槽回调表（后注册覆盖先注册）
static ExtiHandler exti_handlers[16] = {nullptr};

/**
 * @brief 将单 bit 引脚掩码转换为 EXTI 索引
 * @param pin_mask 引脚掩码（应为 2^n）
 * @return 0~15 为有效索引，16 表示非法掩码
 */
static uint8_t GetPinIndex(uint16_t pin_mask)
{
    if (pin_mask == 0 || (pin_mask & (pin_mask - 1U)) != 0)
    {
        return 16;
    }

    uint8_t index = 0;
    while (pin_mask > 1U)
    {
        pin_mask >>= 1U;
        index++;
    }
    return index;
}

/**
 * @brief 将引脚号转换为位掩码
 * @param pin_number 引脚号（0~15）
 * @return 对应位掩码；非法编号返回 0
 */
static uint16_t NumberToMask(uint8_t pin_number)
{
    if (pin_number > 15U)
    {
        return 0U;
    }
    return static_cast<uint16_t>(1U << pin_number);
}

/**
 * @brief 将端口字母映射为不透明端口指针
 * @param port_char 端口字母（A~I）
 * @return 有效端口返回非空，非法端口返回 nullptr
 */
static void* PortCharToOpaque(char port_char)
{
#ifdef USE_REAL_HAL
    switch (port_char)
    {
#ifdef GPIOA
    case 'A': return (void*)GPIOA;
#endif
#ifdef GPIOB
    case 'B': return (void*)GPIOB;
#endif
#ifdef GPIOC
    case 'C': return (void*)GPIOC;
#endif
#ifdef GPIOD
    case 'D': return (void*)GPIOD;
#endif
#ifdef GPIOE
    case 'E': return (void*)GPIOE;
#endif
#ifdef GPIOF
    case 'F': return (void*)GPIOF;
#endif
#ifdef GPIOG
    case 'G': return (void*)GPIOG;
#endif
#ifdef GPIOH
    case 'H': return (void*)GPIOH;
#endif
#ifdef GPIOI
    case 'I': return (void*)GPIOI;
#endif
    default: return nullptr;
    }
#else
    (void)port_char;
    return nullptr;
#endif
}

/**
 * @brief 检测 GPIO 端口时钟是否已开启
 * @param port_opaque 端口指针
 * @return true 已开启，false 未开启或无效
 */
static bool IsPortClockEnabled(void* port_opaque)
{
#ifdef USE_REAL_HAL
    if (port_opaque == nullptr)
    {
        return false;
    }

    GPIO_TypeDef* GPIOx = static_cast<GPIO_TypeDef*>(port_opaque);
#ifdef GPIOA
    if (GPIOx == GPIOA) return __HAL_RCC_GPIOA_IS_CLK_ENABLED();
#endif
#ifdef GPIOB
    if (GPIOx == GPIOB) return __HAL_RCC_GPIOB_IS_CLK_ENABLED();
#endif
#ifdef GPIOC
    if (GPIOx == GPIOC) return __HAL_RCC_GPIOC_IS_CLK_ENABLED();
#endif
#ifdef GPIOD
    if (GPIOx == GPIOD) return __HAL_RCC_GPIOD_IS_CLK_ENABLED();
#endif
#ifdef GPIOE
    if (GPIOx == GPIOE) return __HAL_RCC_GPIOE_IS_CLK_ENABLED();
#endif
#ifdef GPIOF
    if (GPIOx == GPIOF) return __HAL_RCC_GPIOF_IS_CLK_ENABLED();
#endif
#ifdef GPIOG
    if (GPIOx == GPIOG) return __HAL_RCC_GPIOG_IS_CLK_ENABLED();
#endif
#ifdef GPIOH
    if (GPIOx == GPIOH) return __HAL_RCC_GPIOH_IS_CLK_ENABLED();
#endif
#ifdef GPIOI
    if (GPIOx == GPIOI) return __HAL_RCC_GPIOI_IS_CLK_ENABLED();
#endif
#endif
    return false;
}

/**
 * @brief 轻量检测引脚是否已在 CubeMX 中配置
 * @param port_opaque 端口指针
 * @param pin_mask 引脚位掩码
 * @return true 已配置，false 未配置
 * @note 此处逻辑针对 STM32 F4/F7/H7/G4 等具有 MODER 寄存器的系列。
 *       F1 系列（使用 CRL/CRH 寄存器）需要另行适配。
 * @note 这里按“是否偏离上电复位态”做轻量判断：
 *       - MODER != 00 认为已配置（输出/复用/模拟）；
 *       - MODER == 00 时，再看 PUPDR 是否被配置了上下拉；
 *       - 仅当 MODER == 00 且 PUPDR == 00 时，才认为仍是默认未配置态。
 */
static bool IsPinInitialized(void* port_opaque, uint16_t pin_mask)
{
#ifdef USE_REAL_HAL
    if (port_opaque == nullptr || pin_mask == 0U)
    {
        return false;
    }

    const uint8_t index = GetPinIndex(pin_mask);
    if (index >= 16)
    {
        return false;
    }

    GPIO_TypeDef* GPIOx = static_cast<GPIO_TypeDef*>(port_opaque);

    // 轻量判定：优先看 MODER 是否已经被改成输出/复用/模拟。
    // 对于输入脚，CubeMX 往往只会改 PUPDR；因此需要把“输入 + 上下拉已配置”也视为已初始化。
    const uint32_t mode = (GPIOx->MODER >> (index * 2U)) & 0x03U;
    if (mode != 0U)
    {
        return true;
    }

    const uint32_t pull = (GPIOx->PUPDR >> (index * 2U)) & 0x03U;
    return (pull != 0U);
#else
    (void)port_opaque;
    (void)pin_mask;
    return false;
#endif
}

Inst::Inst() : port_opaque(nullptr), pin_mask(0U), valid(false)
{
}

Inst::Inst(Pin pin)
{
    this->port_opaque = PortCharToOpaque(pin.port);
    this->pin_mask = NumberToMask(pin.number);

    // 基础合法性检查：端口和掩码必须对应底层硬件定义
    const bool basic_valid = (this->port_opaque != nullptr && this->pin_mask != 0U);
    if (!basic_valid)
    {
        BspLog_LogWarning("[GPIO] Pin Definition Illegal: Port %c, Pin %d\n", pin.port, pin.number);
        this->valid = false;
        return;
    }

    // 扩展检查：检测 CubeMX 硬件配置是否就绪
    const bool clock_enabled = IsPortClockEnabled(this->port_opaque);
    const bool pin_init = IsPinInitialized(this->port_opaque, this->pin_mask);

    if (!clock_enabled)
    {
        BspLog_LogWarning("[GPIO] Port %c Clock NOT Enabled. Check CubeMX!\n", pin.port);
    }

    if (!pin_init)
    {
        BspLog_LogWarning("[GPIO] Pin %c%d NOT Configured. Check MODER!\n", pin.port, pin.number);
    }

    // 只有时钟已开启且引脚模式已被配置（非模拟输入）才判定为有效
    this->valid = basic_valid && clock_enabled && pin_init;
}

bool Inst::Read() const
{
    if (!this->valid)
    {
        // BspLog_LogWarning("Read Invalid Pin!");
        return false;
    }

#ifdef USE_REAL_HAL
    return HAL_GPIO_ReadPin((GPIO_TypeDef*)this->port_opaque, this->pin_mask) == GPIO_PIN_SET;
#else
    return false;
#endif
}

void Inst::Write(bool high) const
{
    if (!this->valid)
    {
        return;
    }

#ifdef USE_REAL_HAL
    HAL_GPIO_WritePin((GPIO_TypeDef*)this->port_opaque, this->pin_mask, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)high;
#endif
}

void Inst::Toggle() const
{
    if (!this->valid)
    {
        return;
    }

#ifdef USE_REAL_HAL
    HAL_GPIO_TogglePin((GPIO_TypeDef*)this->port_opaque, this->pin_mask);
#endif
}

void Inst::Lock() const
{
    if (!this->valid)
    {
        return;
    }

#ifdef USE_REAL_HAL
    HAL_GPIO_LockPin((GPIO_TypeDef*)this->port_opaque, this->pin_mask);
#endif
}

void Inst::RegisterExtiHandler(ExtiHandler handler) const
{
    if (!this->valid)
    {
        return;
    }

    const uint8_t index = GetPinIndex(this->pin_mask);
    if (index >= 16)
    {
        return;
    }

    exti_handlers[index] = handler;
}

bool Inst::IsValid() const
{
    return this->valid;
}

/**
 * @brief HAL EXTI 回调桥接入口
 * @param GPIO_Pin HAL 上报的引脚掩码
 * @note 仅按引脚线分发，回调中的 pin.port 固定为 '\\0'
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    const uint8_t index = GetPinIndex(GPIO_Pin);
    if (index >= 16)
    {
        return;
    }

    ExtiHandler handler = exti_handlers[index];
    if (handler == nullptr)
    {
        return;
    }

    Pin pin = {'\0', index};
    handler(pin);
}
