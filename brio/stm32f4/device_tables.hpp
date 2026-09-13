/*
 * device_tables.hpp
 *
 * THE RESERVE of the stm32f4 stratum - the one file where the
 * preprocessor is allowed to ask the device header what exists. The
 * standing rule (docs/design/overview.md, "Generalization rule"): a probe
 * that yields a VALUE lives here and is exported as constexpr data; a
 * driver keeps `#ifdef` only to select per-instance CODE, never to learn
 * a fact. Nothing here is a copied table - every entry is the device
 * header's own symbol, so a variant the pack adds tomorrow is described
 * by its header and not by a list someone has to keep - with ONE class
 * of exception, stated where it is made: the frequency ladders.
 *
 * WHAT DIFFERS ACROSS THE STM32F4 FAMILY, as far as the strata reach
 * (cmsis-device-f4 v2.6.9, all twenty-three headers of the pack, one
 * umbrella stm32f4xx.h dispatching on the device-select define):
 *  - GPIO ports: A, B, C and H on every part; D from the 64-pin
 *    bondings of the F412 up and on every F401/F411, E from the
 *    100-pin ones and again on every F401/F411; F and G on the F405
 *    class, the F412Zx, the F413/F423 and the F446; I on the F405 class
 *    and the big packages; J and K on the F42x/F43x and F469/F479 alone.
 *  - Serial instances: USART1 and USART2 everywhere, USART6 everywhere
 *    but the 36-pin F410Tx; USART3 on the F405 class and up and on the
 *    F412/F413; UART4 and UART5 on the F405 class, the F413/F423, the
 *    F42x/F43x, the F446 and the F469/F479; UART7 and UART8 on the
 *    F413/F423, the F42x/F43x and the F469/F479; UART9 and UART10 on the
 *    F413/F423 alone. USART1 and USART6 sit on APB2, every other
 *    instance on APB1 - which decides the bus rate a baud divisor
 *    divides.
 *  - Which instances are FULL. RM0090 table 148 (and its RM0390 and
 *    RM0383 twins): UART4, UART5, UART7 and UART8 have no synchronous
 *    mode, no smartcard and no hardware flow control. That is not in
 *    the header - the U(S)ART distinction in the NAME is what carries
 *    it, and usart_is_full() reads the name.
 *  - Interrupt lines: on this family a vector is mostly ONE
 *    peripheral's, and the sharing the table does show (TIM1_BRK_TIM9,
 *    TIM8_UP_TIM13, TIM6_DAC, the EXTI groups, the I2C's EV/ER pair) is
 *    the same on every header that has both parties. IRQn values are
 *    enumerators the preprocessor cannot probe, so every vector verb
 *    here is guarded by the instance's base-address macro and a header
 *    without the instance has no verb for it.
 *  - The regulator: two VOS bits on every part but the F405/F407/F415/
 *    F417, whose PWR_CR carries ONE (PWR_CR_VOS_1 is what the header
 *    declares or not); the over-drive pair ODEN/ODSWEN on the F42x/F43x,
 *    the F446 and the F469/F479 alone (PWR_CR_ODEN).
 *  - The flash interface's LATENCY field: four bits where the ladder
 *    reaches 180 MHz, three where it does not (FLASH_ACR_LATENCY_Msk).
 *  - The PLL's R output (PLLCFGR.PLLR, the F446's and F469's), and the
 *    second PLL's names (PLLI2S everywhere, PLLSAI where an LTDC or a
 *    SAI needs it) - none of them this stratum's business yet.
 *
 * THE FREQUENCY LADDERS ARE THE EXCEPTION. How fast a part may run at
 * each regulator scale, with and without over-drive, how many flash
 * wait states a HCLK needs and how fast each APB may go are facts of
 * the REFERENCE MANUAL, not of the header, and they differ per part
 * class. They are keyed here on the device-select macro - the one
 * thing about the part the preprocessor can know - and stated ONLY for
 * the classes whose manual was read for them (RM0090 for the F405/F407
 * and F42x/F43x classes, RM0390 for the F446, RM0383 for the F411). A
 * header outside those classes gets `sysclk_ladder().known == false`,
 * and stm32f4/clock.hpp refuses any rate above the reset one there
 * rather than run a part on a ladder nobody read. The voltage range
 * every ladder is stated for is 2.7..3.6 V, the boards' supply; the
 * lower ranges' columns are declared and not carried.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx.h"

namespace brio {

// ---- GPIO ports -------------------------------------------------------------

/// Register block base of GPIO port `letter` (A..K), 0 when the device
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
#if defined(GPIOI_BASE)
        case 'I': return GPIOI_BASE;
#endif
#if defined(GPIOJ_BASE)
        case 'J': return GPIOJ_BASE;
#endif
#if defined(GPIOK_BASE)
        case 'K': return GPIOK_BASE;
#endif
        default: return 0;
    }
}

/// Whether this device bonds GPIO port `letter` at all.
constexpr bool gpio_port_present(char letter) { return gpio_port_base(letter) != 0u; }

/// The RCC_AHB1ENR bit that clocks port `letter`; 0 when absent. The
/// header spells one macro per port, so each is probed by name.
constexpr uint32_t gpio_port_clock_mask(char letter) {
    switch (letter) {
#if defined(RCC_AHB1ENR_GPIOAEN)
        case 'A': return RCC_AHB1ENR_GPIOAEN;
#endif
#if defined(RCC_AHB1ENR_GPIOBEN)
        case 'B': return RCC_AHB1ENR_GPIOBEN;
#endif
#if defined(RCC_AHB1ENR_GPIOCEN)
        case 'C': return RCC_AHB1ENR_GPIOCEN;
#endif
#if defined(RCC_AHB1ENR_GPIODEN)
        case 'D': return RCC_AHB1ENR_GPIODEN;
#endif
#if defined(RCC_AHB1ENR_GPIOEEN)
        case 'E': return RCC_AHB1ENR_GPIOEEN;
#endif
#if defined(RCC_AHB1ENR_GPIOFEN)
        case 'F': return RCC_AHB1ENR_GPIOFEN;
#endif
#if defined(RCC_AHB1ENR_GPIOGEN)
        case 'G': return RCC_AHB1ENR_GPIOGEN;
#endif
#if defined(RCC_AHB1ENR_GPIOHEN)
        case 'H': return RCC_AHB1ENR_GPIOHEN;
#endif
#if defined(RCC_AHB1ENR_GPIOIEN)
        case 'I': return RCC_AHB1ENR_GPIOIEN;
#endif
#if defined(RCC_AHB1ENR_GPIOJEN)
        case 'J': return RCC_AHB1ENR_GPIOJEN;
#endif
#if defined(RCC_AHB1ENR_GPIOKEN)
        case 'K': return RCC_AHB1ENR_GPIOKEN;
#endif
        default: return 0;
    }
}

// ---- serial instances ---------------------------------------------------------

/// Register block base of instance n (1..10: USART1, USART2, USART3,
/// UART4, UART5, USART6, UART7, UART8, UART9, UART10 - the family's
/// numbering, the U(S)ART spelling being what usart_is_full() reads), 0
/// when the device does not have it.
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
#if defined(UART4_BASE)
        case 4: return UART4_BASE;
#endif
#if defined(UART5_BASE)
        case 5: return UART5_BASE;
#endif
#if defined(USART6_BASE)
        case 6: return USART6_BASE;
#endif
#if defined(UART7_BASE)
        case 7: return UART7_BASE;
#endif
#if defined(UART8_BASE)
        case 8: return UART8_BASE;
#endif
#if defined(UART9_BASE)
        case 9: return UART9_BASE;
#endif
#if defined(UART10_BASE)
        case 10: return UART10_BASE;
#endif
        default: return 0;
    }
}

constexpr bool usart_present(uint8_t n) { return usart_base(n) != 0u; }

/// USART1, USART6 and the F413's UART9/UART10 are APB2's; every other
/// instance is APB1's (RM0090 table 1, the memory map - and the header
/// agrees, by which ENR the enable bit sits in).
constexpr bool usart_on_apb2(uint8_t n) { return n == 1 || n == 6 || n == 9 || n == 10; }

/// A FULL instance has synchronous mode, smartcard and hardware flow
/// control; the UARTs (4, 5, 7, 8, 9, 10) have not (RM0090 table 148,
/// RM0390 table 130, RM0383 table 91 - the same rows on every manual).
constexpr bool usart_is_full(uint8_t n) { return n == 1 || n == 2 || n == 3 || n == 6; }

/// The instance's enable bit in its bus's ENR (RCC_APB1ENR or
/// RCC_APB2ENR by usart_on_apb2); 0 when absent. The same position
/// resets it in the matching RSTR.
constexpr uint32_t usart_clock_mask(uint8_t n) {
    switch (n) {
#if defined(RCC_APB2ENR_USART1EN)
        case 1: return RCC_APB2ENR_USART1EN;
#endif
#if defined(RCC_APB1ENR_USART2EN)
        case 2: return RCC_APB1ENR_USART2EN;
#endif
#if defined(RCC_APB1ENR_USART3EN)
        case 3: return RCC_APB1ENR_USART3EN;
#endif
#if defined(RCC_APB1ENR_UART4EN)
        case 4: return RCC_APB1ENR_UART4EN;
#endif
#if defined(RCC_APB1ENR_UART5EN)
        case 5: return RCC_APB1ENR_UART5EN;
#endif
#if defined(RCC_APB2ENR_USART6EN)
        case 6: return RCC_APB2ENR_USART6EN;
#endif
#if defined(RCC_APB1ENR_UART7EN)
        case 7: return RCC_APB1ENR_UART7EN;
#endif
#if defined(RCC_APB1ENR_UART8EN)
        case 8: return RCC_APB1ENR_UART8EN;
#endif
#if defined(RCC_APB2ENR_UART9EN)
        case 9: return RCC_APB2ENR_UART9EN;
#endif
#if defined(RCC_APB2ENR_UART10EN)
        case 10: return RCC_APB2ENR_UART10EN;
#endif
        default: return 0;
    }
}

/// The instance's NVIC line. Every serial instance of this family has a
/// line of its own (no sharing), so the enumerator's name is the
/// instance's; the guard is the instance's presence.
constexpr IRQn_Type usart_irq(uint8_t n) {
    switch (n) {
#if defined(USART1_BASE)
        case 1: return USART1_IRQn;
#endif
#if defined(USART2_BASE)
        case 2: return USART2_IRQn;
#endif
#if defined(USART3_BASE)
        case 3: return USART3_IRQn;
#endif
#if defined(UART4_BASE)
        case 4: return UART4_IRQn;
#endif
#if defined(UART5_BASE)
        case 5: return UART5_IRQn;
#endif
#if defined(USART6_BASE)
        case 6: return USART6_IRQn;
#endif
#if defined(UART7_BASE)
        case 7: return UART7_IRQn;
#endif
#if defined(UART8_BASE)
        case 8: return UART8_IRQn;
#endif
#if defined(UART9_BASE)
        case 9: return UART9_IRQn;
#endif
#if defined(UART10_BASE)
        case 10: return UART10_IRQn;
#endif
        default: return NonMaskableInt_IRQn;   // unreachable: callers check usart_present first
    }
}

// ---- the regulator and the clock ladders --------------------------------------

/// Whether PWR_CR carries the over-drive pair (ODEN, ODSWEN) - the
/// F42x/F43x, F446 and F469/F479 classes.
constexpr bool pwr_has_over_drive() {
#if defined(PWR_CR_ODEN)
    return true;
#else
    return false;
#endif
}

/// Whether VOS is a two-bit field (scales 3, 2, 1) or the F405 class's
/// single bit (scales 2, 1).
constexpr bool pwr_vos_two_bits() {
#if defined(PWR_CR_VOS_1)
    return true;
#else
    return false;
#endif
}

/// What a part class may run at, in Hz, at 2.7..3.6 V - the reference
/// manual's numbers, keyed on the device-select define (see the file
/// header for why this one table is a table).
struct SysclkLadder {
    bool known = false;
    uint32_t scale3_hz = 0;      ///< HCLK ceiling at regulator scale 3 (0 on a one-bit VOS)
    uint32_t scale2_hz = 0;      ///< ... at scale 2
    uint32_t scale1_hz = 0;      ///< ... at scale 1
    uint32_t od_scale2_hz = 0;   ///< scale 2 with over-drive (0 where there is none)
    uint32_t od_scale1_hz = 0;   ///< scale 1 with over-drive
    uint32_t apb1_max_hz = 0;    ///< PCLK1 ceiling (RCC_CFGR.PPRE1's caution)
    uint32_t apb2_max_hz = 0;    ///< PCLK2 ceiling (PPRE2's)
    uint8_t ws_bands = 0;        ///< how many rows of ws_ceiling_hz are used
    uint32_t ws_ceiling_hz[9] = {};   ///< HCLK ceiling for 0, 1, 2, ... wait states
};

constexpr SysclkLadder sysclk_ladder() {
    SysclkLadder l{};
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)
    // RM0090 5.4.1 (one VOS bit: scale 2 up to 144 MHz, scale 1 up to
    // 168 MHz), 6.3.3 (PPRE1 42 MHz, PPRE2 84 MHz), 3.5.1 table 11.
    l.known = true;
    l.scale2_hz = 144'000'000u;
    l.scale1_hz = 168'000'000u;
    l.apb1_max_hz = 42'000'000u;
    l.apb2_max_hz = 84'000'000u;
    l.ws_bands = 6;
    l.ws_ceiling_hz[0] = 30'000'000u;  l.ws_ceiling_hz[1] = 60'000'000u;  l.ws_ceiling_hz[2] = 90'000'000u;
    l.ws_ceiling_hz[3] = 120'000'000u; l.ws_ceiling_hz[4] = 150'000'000u; l.ws_ceiling_hz[5] = 168'000'000u;
#elif defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F446xx)
    // RM0090 5.1.4 / RM0390 5.1.4 (scale 3 up to 120, scale 2 up to
    // 144 - 168 in over-drive -, scale 1 up to 168 - 180 in over-drive),
    // 7.3.3 (PPRE1 45 MHz, PPRE2 90 MHz), 3.5.1 table 12 / RM0390 table 5.
    l.known = true;
    l.scale3_hz = 120'000'000u;
    l.scale2_hz = 144'000'000u;
    l.scale1_hz = 168'000'000u;
    l.od_scale2_hz = 168'000'000u;
    l.od_scale1_hz = 180'000'000u;
    l.apb1_max_hz = 45'000'000u;
    l.apb2_max_hz = 90'000'000u;
    l.ws_bands = 6;
    l.ws_ceiling_hz[0] = 30'000'000u;  l.ws_ceiling_hz[1] = 60'000'000u;  l.ws_ceiling_hz[2] = 90'000'000u;
    l.ws_ceiling_hz[3] = 120'000'000u; l.ws_ceiling_hz[4] = 150'000'000u; l.ws_ceiling_hz[5] = 180'000'000u;
#elif defined(STM32F411xE)
    // RM0383 5.1.4 (scale 3 up to 64, scale 2 up to 84, scale 1 up to
    // 100 MHz, no over-drive), 6.3.3 (PPRE1 50 MHz, PPRE2 100 MHz),
    // 3.4.1 table 5.
    l.known = true;
    l.scale3_hz = 64'000'000u;
    l.scale2_hz = 84'000'000u;
    l.scale1_hz = 100'000'000u;
    l.apb1_max_hz = 50'000'000u;
    l.apb2_max_hz = 100'000'000u;
    l.ws_bands = 4;
    l.ws_ceiling_hz[0] = 30'000'000u; l.ws_ceiling_hz[1] = 64'000'000u;
    l.ws_ceiling_hz[2] = 90'000'000u; l.ws_ceiling_hz[3] = 100'000'000u;
#endif
    return l;
}

/// The highest HCLK this part may run at, over-drive included; 0 when
/// the ladder is not known.
constexpr uint32_t sysclk_max_hz() {
    constexpr SysclkLadder l = sysclk_ladder();
    if (!l.known) {
        return 0;
    }
    return l.od_scale1_hz != 0u ? l.od_scale1_hz : l.scale1_hz;
}

/// Wait states for `hclk` at 2.7..3.6 V; 0xFF when the ladder is not
/// known or the rate is above its last band.
constexpr uint8_t flash_wait_states_for(uint32_t hclk) {
    constexpr SysclkLadder l = sysclk_ladder();
    if (!l.known) {
        return 0xFF;
    }
    for (uint8_t ws = 0; ws < l.ws_bands; ++ws) {
        if (hclk <= l.ws_ceiling_hz[ws]) {
            return ws;
        }
    }
    return 0xFF;
}

/// The unique device ID's 96 bits (RM0090 39.1, RM0390 34.1, RM0383
/// 24.1), the flash size register and the package register - memory-
/// mapped system memory, at the addresses the header states.
inline constexpr uint32_t device_uid_base = UID_BASE;
inline constexpr uint32_t flash_size_register = FLASHSIZE_BASE;
inline constexpr uint32_t package_register = PACKAGE_BASE;

} // namespace brio
