// Handler-name family smoke TU: the vector names an app binds on more
// than one board, derived by the reserve from the header's presence
// macros (stm32g0/device_tables.hpp's BRIO_STM32G0_*_HANDLER). Declaring
// each as the strong symbol an app would define is what proves every
// macro expands on every header of the pack - and the IRQn each name
// stands for is the one the reserve's own verb answers, so a name and
// its line cannot drift apart.
#include "stm32g0/device_tables.hpp"

using namespace brio;

extern "C" {
void BRIO_STM32G0_USART2_HANDLER() {}
void BRIO_STM32G0_TIM3_HANDLER() {}
void BRIO_STM32G0_TIM16_HANDLER() {}
void BRIO_STM32G0_TIM17_HANDLER() {}
void BRIO_STM32G0_DMA1_CH4_UP_HANDLER() {}
void BRIO_STM32G0_ADC1_HANDLER() {}
void BRIO_STM32G0_SPI2_HANDLER() {}
void BRIO_STM32G0_I2C2_HANDLER() {}
#if defined(USART3_BASE)
void BRIO_STM32G0_USART3_HANDLER() {}
#endif
#if defined(LPUART1_BASE) && !defined(USART3_BASE)
void BRIO_STM32G0_LPUART1_HANDLER() {}
#endif
#if defined(TIM6_BASE)
void BRIO_STM32G0_TIM6_HANDLER() {}
#endif
#if defined(TIM7_BASE)
void BRIO_STM32G0_TIM7_HANDLER() {}
#endif
#if defined(LPTIM1_BASE) && !defined(TIM6_BASE)
void BRIO_STM32G0_LPTIM1_HANDLER() {}
#endif
#if defined(LPTIM2_BASE) && !defined(TIM7_BASE)
void BRIO_STM32G0_LPTIM2_HANDLER() {}
#endif
}

// The name's line is the verb's line: spelled out for the lines whose
// acronym changes across the pack.
#if defined(LPUART2_BASE)
static_assert(usart_irq(2) == USART2_LPUART2_IRQn);
#else
static_assert(usart_irq(2) == USART2_IRQn);
#endif
#if defined(TIM4_BASE)
static_assert(tim_irq(3) == TIM3_TIM4_IRQn);
#else
static_assert(tim_irq(3) == TIM3_IRQn);
#endif
#if defined(COMP1_BASE)
static_assert(adc_irq() == ADC1_COMP_IRQn);
#else
static_assert(adc_irq() == ADC1_IRQn);
#endif
#if defined(SPI3_BASE)
static_assert(spi_irq(2) == SPI2_3_IRQn && spi_irq(3) == SPI2_3_IRQn);
#else
static_assert(spi_irq(2) == SPI2_IRQn);
#endif
#if defined(I2C3_BASE)
static_assert(i2c_irq(2) == I2C2_3_IRQn && i2c_irq(3) == I2C2_3_IRQn);
#else
static_assert(i2c_irq(2) == I2C2_IRQn);
#endif
