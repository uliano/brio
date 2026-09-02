/*
 * device_tables.hpp
 *
 * THE RESERVE of the stm32g0 stratum - the one file where the
 * preprocessor is allowed to ask the device header what exists. The
 * standing rule (docs/design/overview.md, "Generalization rule"): a probe
 * that yields a VALUE lives here and is exported as constexpr data; a
 * driver keeps `#ifdef` only to select per-instance CODE, never to learn
 * a fact. Nothing here is a copied table - every entry is the device
 * header's own symbol, so a variant the pack adds tomorrow is described
 * by its header and not by a list someone has to keep.
 *
 * WHAT DIFFERS ACROSS THE STM32G0 FAMILY, as far as the bring-up strata
 * reach (cmsis-device-g0 v1.4.5, headers stm32g0b1xx/g071xx/g031xx):
 *  - GPIO ports: A, B, C, D, F on every part; E only on the G0B1/G0C1
 *    (the 100-pin bonding); no G or H anywhere.
 *  - USART instances: 1..2 on the G031/G041, 1..4 on the G071/G081,
 *    1..6 on the G0B1/G0C1; LPUART1 everywhere, LPUART2 on G0B1/G0C1.
 *  - Interrupt lines: USART2 shares its vector with LPUART2 on the
 *    G0B1/G0C1 and has one of its own elsewhere; USART3..6 and LPUART1
 *    share ONE line on the G0B1/G0C1 where the G071's is USART3/4/
 *    LPUART1. IRQn values are ENUMERATORS, which the preprocessor cannot
 *    probe, so the vector of an instance is read off the DEVICE SELECT
 *    macro (STM32G0B1xx and friends) - the one place a device name is
 *    spelled in this stratum.
 *  - The USART kernel-clock multiplexer (RCC_CCIPR.USARTnSEL) exists for
 *    USART1 everywhere, for USART2 on the G071 class and up, for USART3
 *    on the G0B1 class; the other instances run on PCLK with no choice.
 */

#pragma once

#include <stdint.h>

#include "stm32g0xx.h"

namespace brio {

// ---- GPIO ports -------------------------------------------------------------

/// Register block base of GPIO port `letter` (A..H), 0 when the device
/// header does not declare it. The header's GPIOx_BASE is the authority.
constexpr uint32_t gpio_port_base(char letter) {
    switch (letter) {
#if defined(GPIOA_BASE)
        case 'A': return GPIOA_BASE;
#endif
#if defined(GPIOB_BASE)
        case 'B': return GPIOB_BASE;
#endif
#if defined(GPIOC_BASE)
        case 'C': return GPIOC_BASE;
#endif
#if defined(GPIOD_BASE)
        case 'D': return GPIOD_BASE;
#endif
#if defined(GPIOE_BASE)
        case 'E': return GPIOE_BASE;
#endif
#if defined(GPIOF_BASE)
        case 'F': return GPIOF_BASE;
#endif
#if defined(GPIOG_BASE)
        case 'G': return GPIOG_BASE;
#endif
#if defined(GPIOH_BASE)
        case 'H': return GPIOH_BASE;
#endif
        default: return 0;
    }
}

/// Whether this device bonds GPIO port `letter` at all.
constexpr bool gpio_port_present(char letter) { return gpio_port_base(letter) != 0u; }

/// The RCC_IOPENR bit that clocks port `letter`; 0 when absent. The
/// header spells one macro per port, so each is probed by name.
constexpr uint32_t gpio_port_clock_mask(char letter) {
    switch (letter) {
#if defined(RCC_IOPENR_GPIOAEN)
        case 'A': return RCC_IOPENR_GPIOAEN;
#endif
#if defined(RCC_IOPENR_GPIOBEN)
        case 'B': return RCC_IOPENR_GPIOBEN;
#endif
#if defined(RCC_IOPENR_GPIOCEN)
        case 'C': return RCC_IOPENR_GPIOCEN;
#endif
#if defined(RCC_IOPENR_GPIODEN)
        case 'D': return RCC_IOPENR_GPIODEN;
#endif
#if defined(RCC_IOPENR_GPIOEEN)
        case 'E': return RCC_IOPENR_GPIOEEN;
#endif
#if defined(RCC_IOPENR_GPIOFEN)
        case 'F': return RCC_IOPENR_GPIOFEN;
#endif
#if defined(RCC_IOPENR_GPIOGEN)
        case 'G': return RCC_IOPENR_GPIOGEN;
#endif
#if defined(RCC_IOPENR_GPIOHEN)
        case 'H': return RCC_IOPENR_GPIOHEN;
#endif
        default: return 0;
    }
}

// ---- USART instances --------------------------------------------------------

/// Register block base of USARTn (n = 1..6), 0 when the device does not
/// have it. LPUARTs are a different peripheral (their own baud
/// arithmetic and clock domain) and get their own probes when a driver
/// wants them.
constexpr uint32_t usart_base(uint8_t n) {
    switch (n) {
#if defined(USART1_BASE)
        case 1: return USART1_BASE;
#endif
#if defined(USART2_BASE)
        case 2: return USART2_BASE;
#endif
#if defined(USART3_BASE)
        case 3: return USART3_BASE;
#endif
#if defined(USART4_BASE)
        case 4: return USART4_BASE;
#endif
#if defined(USART5_BASE)
        case 5: return USART5_BASE;
#endif
#if defined(USART6_BASE)
        case 6: return USART6_BASE;
#endif
        default: return 0;
    }
}

constexpr bool usart_present(uint8_t n) { return usart_base(n) != 0u; }

/// Which APB enable register carries USARTn, and which bit. USART1 is
/// the one APB2 instance of the family; every other USART sits on
/// APBENR1. `apb2` false + mask 0 means "no such instance".
struct UsartBusClock {
    bool apb2 = false;
    uint32_t mask = 0;
};

constexpr UsartBusClock usart_bus_clock(uint8_t n) {
    switch (n) {
#if defined(RCC_APBENR2_USART1EN)
        case 1: return {true, RCC_APBENR2_USART1EN};
#endif
#if defined(RCC_APBENR1_USART2EN)
        case 2: return {false, RCC_APBENR1_USART2EN};
#endif
#if defined(RCC_APBENR1_USART3EN)
        case 3: return {false, RCC_APBENR1_USART3EN};
#endif
#if defined(RCC_APBENR1_USART4EN)
        case 4: return {false, RCC_APBENR1_USART4EN};
#endif
#if defined(RCC_APBENR1_USART5EN)
        case 5: return {false, RCC_APBENR1_USART5EN};
#endif
#if defined(RCC_APBENR1_USART6EN)
        case 6: return {false, RCC_APBENR1_USART6EN};
#endif
        default: return {};
    }
}

/// Position of USARTn's kernel-clock select field in RCC_CCIPR, or 0xFF
/// when the instance has no multiplexer (it then runs on PCLK, full
/// stop). USART1's field sits at bit 0, so the "absent" code cannot be 0.
constexpr uint8_t usart_clock_select_pos(uint8_t n) {
    switch (n) {
#if defined(RCC_CCIPR_USART1SEL_Pos)
        case 1: return RCC_CCIPR_USART1SEL_Pos;
#endif
#if defined(RCC_CCIPR_USART2SEL_Pos)
        case 2: return RCC_CCIPR_USART2SEL_Pos;
#endif
#if defined(RCC_CCIPR_USART3SEL_Pos)
        case 3: return RCC_CCIPR_USART3SEL_Pos;
#endif
        default: return 0xFF;
    }
}

constexpr bool usart_has_clock_select(uint8_t n) { return usart_clock_select_pos(n) != 0xFF; }

/// The NVIC line of USARTn. IRQn_Type values are enumerators and cannot
/// be probed, so the family's vector sharing is read off the DEVICE
/// SELECT macro - the one place this stratum spells a device name. A
/// wrong line here would be a silent Default_Handler spin (the samc
/// stratum's NMI lesson), which is why the family fixture instantiates
/// every present instance on every header the pack ships.
constexpr IRQn_Type usart_irq(uint8_t n) {
#if defined(STM32G0B1xx) || defined(STM32G0C1xx) || defined(STM32G0B0xx)
    switch (n) {
        case 1: return USART1_IRQn;
        case 2: return USART2_LPUART2_IRQn;
        default: return USART3_4_5_6_LPUART1_IRQn;
    }
#elif defined(STM32G071xx) || defined(STM32G081xx) || defined(STM32G070xx)
    switch (n) {
        case 1: return USART1_IRQn;
        case 2: return USART2_IRQn;
        default: return USART3_4_LPUART1_IRQn;
    }
#else
    // G031/G041/G030/G051/G061/G050: USART1 and USART2 only, one line each.
    return n == 1 ? USART1_IRQn : USART2_IRQn;
#endif
}

} // namespace brio
