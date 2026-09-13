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

// ---- reset and watchdogs ------------------------------------------------------

/// Whether the independent watchdog has a WINDOW register (IWDG_WINR and
/// its WVU update bit). NO header of this pack declares one - the F4's
/// IWDG is four registers and an early refresh is not a reset here - and
/// stm32f4/reset.hpp offers no window verb because of it. The probe is
/// what makes that a checked fact rather than a claim.
constexpr bool iwdg_has_window() {
#if defined(IWDG_WINR_WIN)
    return true;
#else
    return false;
#endif
}

/// How many codes the window watchdog's prescaler field holds: FOUR on
/// this family (WDGTB is two bits, /1 to /8), read off the header's own
/// mask rather than counted by hand.
constexpr uint8_t wwdg_prescaler_codes() {
    return static_cast<uint8_t>((WWDG_CFR_WDGTB_Msk >> WWDG_CFR_WDGTB_Pos) + 1u);
}

/// Whether this device has the window watchdog at all (every part of the
/// pack does; the probe is the reserve's habit, not a doubt).
constexpr bool wwdg_present() {
#if defined(WWDG_BASE)
    return true;
#else
    return false;
#endif
}

/// The window watchdog's NVIC line - position 0 of the vector table on
/// every part of this family, and its alone. An IRQn is an enumerator
/// the preprocessor cannot probe, so the guard is the instance's
/// base-address macro, as it is for the serial instances above.
constexpr IRQn_Type wwdg_irq() {
#if defined(WWDG_BASE)
    return WWDG_IRQn;
#else
    return NonMaskableInt_IRQn;   // unreachable: callers check wwdg_present first
#endif
}

// ---- EXTI and SYSCFG ----------------------------------------------------------
//
// WHAT DIFFERS ACROSS THE FAMILY HERE is which of the lines above 15
// exist, and that is decided by WHICH PERIPHERAL IS WIRED TO EACH -
// RM0090 12.2.5, RM0390 10.2.5, RM0383 10.2.5 list them line by line, and
// the three lists differ exactly where the silicon differs. So the mask
// below is built from the PERIPHERALS' base-address macros and not from a
// per-part table: the Ethernet wake-up line exists where the device header
// declares an Ethernet MAC, the USB OTG HS wake-up line where it declares
// that controller, and so on. The device header corroborates it a second
// way, which is worth knowing when reading the code: the wake-up VECTORS
// (ETH_WKUP_IRQn, OTG_HS_WKUP_IRQn, OTG_FS_WKUP_IRQn) are declared on
// exactly the same headers.
//
// The EXTI's own bit masks are NOT an authority for lines 16..22: the pack
// spells EXTI_IMR_MR18, MR19 and MR20 on every header, including parts
// with no USB and no Ethernet at all. Above 22 the header IS the only
// authority, and it is used: EXTI_IMR_MR23 is declared on the parts that
// have a 24th line and on no other.

/// The sixteen GPIO lines - one per PIN NUMBER, on every part of the
/// family (RM0090 12.2.5: the port is what SYSCFG_EXTICR selects).
inline constexpr uint8_t exti_gpio_lines = 16;

/// The SYSCFG_EXTICR code for GPIO port `letter` (0 for A, 1 for B, ...
/// RM0090 9.2.3), 0xFF when this device has no such port. The encoding is
/// contiguous over the eleven ports, so the letter's distance from 'A' IS
/// the code; the presence check is the reserve's own port table.
constexpr uint8_t exti_port_code(char letter) {
    if (!gpio_port_present(letter)) {
        return 0xFFu;
    }
    return static_cast<uint8_t>(letter - 'A');
}

/// Which EXTI lines this device implements, bit x = line x (RM0090
/// 12.2.5 and its RM0390/RM0383 twins - see the note above for why the
/// peripherals decide and not the EXTI's own bit macros).
constexpr uint32_t exti_implemented_mask() {
    uint32_t m = 0x0000FFFFu;   // the sixteen GPIO lines: every part
    m |= 1UL << 16;             // PVD output - every part has the supply monitor
    m |= 1UL << 17;             // RTC alarm
#if defined(USB_OTG_FS_PERIPH_BASE)
    m |= 1UL << 18;             // USB OTG FS wake-up
#endif
#if defined(ETH_BASE)
    m |= 1UL << 19;             // Ethernet wake-up
#endif
#if defined(USB_OTG_HS_PERIPH_BASE)
    m |= 1UL << 20;             // USB OTG HS (in FS mode) wake-up
#endif
    m |= 1UL << 21;             // RTC tamper and timestamp
    m |= 1UL << 22;             // RTC wake-up
#if defined(EXTI_IMR_MR23)
    // A 24th line on the parts that declare it, and the header is the
    // whole authority: what is wired there is the business of a reference
    // manual this project has not read, so nothing here names it. The
    // parts that declare it are exactly the parts with an LPTIM1
    // (test/family_stm32f4/exti.cpp asserts that on every header).
    m |= 1UL << 23;
#endif
    return m;
}

constexpr bool exti_line_implemented(uint8_t line) {
    return line < 32u && (exti_implemented_mask() & (1UL << line)) != 0u;
}

/// The NVIC line an EXTI line interrupts on (RM0090 tables 62 and 63, one
/// per part class): lines 0..4 one vector each, 5..9 and 10..15 grouped,
/// and every line above 15 the vector of the peripheral wired to it. Only
/// meaningful for an implemented line - callers ask
/// exti_line_implemented() first - and NonMaskableInt_IRQn otherwise, the
/// reserve's one unreachable value.
constexpr IRQn_Type exti_line_irq(uint8_t line) {
    switch (line) {
        case 0: return EXTI0_IRQn;
        case 1: return EXTI1_IRQn;
        case 2: return EXTI2_IRQn;
        case 3: return EXTI3_IRQn;
        case 4: return EXTI4_IRQn;
        case 5: case 6: case 7: case 8: case 9: return EXTI9_5_IRQn;
        case 10: case 11: case 12: case 13: case 14: case 15: return EXTI15_10_IRQn;
        case 16: return PVD_IRQn;
        case 17: return RTC_Alarm_IRQn;
#if defined(USB_OTG_FS_PERIPH_BASE)
        case 18: return OTG_FS_WKUP_IRQn;
#endif
#if defined(ETH_BASE)
        case 19: return ETH_WKUP_IRQn;
#endif
#if defined(USB_OTG_HS_PERIPH_BASE)
        case 20: return OTG_HS_WKUP_IRQn;
#endif
        case 21: return TAMP_STAMP_IRQn;
        case 22: return RTC_WKUP_IRQn;
        default: return NonMaskableInt_IRQn;
    }
}

/// The lines one vector answers for, as a mask - what a handler hands
/// Exti::isr() so that it can neither consume nor be confused by another
/// vector's pending bits. Zero for a vector that is not the EXTI's.
constexpr uint32_t exti_vector_lines(IRQn_Type v) {
    switch (v) {
        case EXTI0_IRQn: return 1UL << 0;
        case EXTI1_IRQn: return 1UL << 1;
        case EXTI2_IRQn: return 1UL << 2;
        case EXTI3_IRQn: return 1UL << 3;
        case EXTI4_IRQn: return 1UL << 4;
        case EXTI9_5_IRQn: return 0x000003E0UL;
        case EXTI15_10_IRQn: return 0x0000FC00UL;
        case PVD_IRQn: return 1UL << 16;
        case RTC_Alarm_IRQn: return 1UL << 17;
#if defined(USB_OTG_FS_PERIPH_BASE)
        case OTG_FS_WKUP_IRQn: return 1UL << 18;
#endif
#if defined(ETH_BASE)
        case ETH_WKUP_IRQn: return 1UL << 19;
#endif
#if defined(USB_OTG_HS_PERIPH_BASE)
        case OTG_HS_WKUP_IRQn: return 1UL << 20;
#endif
        case TAMP_STAMP_IRQn: return 1UL << 21;
        case RTC_WKUP_IRQn: return 1UL << 22;
        default: return 0;
    }
}

/// SYSCFG's enable bit in RCC_APB2ENR (RM0090 7.3.14): the gate that has
/// to be open before the EXTI's GPIO multiplexer can be written at all.
inline constexpr uint32_t syscfg_clock_mask = RCC_APB2ENR_SYSCFGEN;

/// SYSCFG_PMC's Ethernet PHY interface select (RM0090 9.2.2), 0 on a part
/// whose header does not declare the bit - which is every part with no
/// Ethernet MAC, plus the F405/F415, whose header declares the MAC's PHY
/// selector without the MAC.
constexpr uint32_t syscfg_phy_select_mask() {
#if defined(SYSCFG_PMC_MII_RMII_SEL)
    return SYSCFG_PMC_MII_RMII_SEL;
#else
    return 0;
#endif
}

// ---- RTC and the backup domain ------------------------------------------------

/// How many RTC_BKPxR words this part has, read off the DEVICE HEADER's
/// own register block: the backup registers are the tail of RTC_TypeDef,
/// so the struct's size minus where they start IS their count. Twenty
/// (80 bytes) on every header of the pack, and derived rather than
/// written down so a variant with fewer describes itself.
constexpr uint8_t rtc_backup_registers() {
    return static_cast<uint8_t>((sizeof(RTC_TypeDef) - offsetof(RTC_TypeDef, BKP0R)) /
                                sizeof(uint32_t));
}

/// Whether RCC_BDCR carries LSEMOD, the LSE's high-drive bit - the F410,
/// F411, F412, F413/F423, F446 and F469/F479 headers declare it, the
/// F401, F405 class and F42x/F43x do not.
constexpr bool rtc_has_lse_mode() {
#if defined(RCC_BDCR_LSEMOD)
    return true;
#else
    return false;
#endif
}

/**
 * The RTC's additional-function facts - AND THE DEVICE HEADER CANNOT BE
 * ASKED FOR THEM, which is why they are keyed on the device-select define
 * like the frequency ladders above.
 *
 * ST's headers declare ONE set of RTC bit masks for the whole family:
 * RTC_TAFCR_TAMP2E, RTC_TAFCR_TAMP2TRG and RTC_ISR_TAMP2F are defined on
 * every part, the F411 included, where RM0383 17.3.13 says "one tamper
 * detection input is available" and 17.6.17 leaves TSINSEL's and
 * TAMP1INSEL's 1 Reserved. So the count is the reference manual's:
 * two inputs with a second RTC pad on the F405 class and the F42x/F43x
 * (RM0090 8.3.15: RTC_AF1 = PC13, RTC_AF2 = PI8) and on the F446
 * (RM0390 7.3.15: RTC_AF2 = PA0), one on the F411 (RM0383 17.2).
 *
 * A part class whose manual is not on the desk gets `known == false` and
 * the ONE input every manual read documents (RTC_AF1); the second, and
 * the pad selects that reach RTC_AF2, are refused there rather than
 * guessed - a tamper detection erases the backup registers, so an armed
 * input that does not exist is a silent hole and an armed input that
 * does is a wiped breadcrumb.
 */
struct RtcPadFacts {
    bool known = false;
    uint8_t tamper_inputs = 1;   ///< TAMPER1 on RTC_AF1 everywhere; 2 where AF2 exists
    bool has_af2 = false;        ///< the second RTC pad, TSINSEL/TAMP1INSEL's 1
    char af2_port = 0;           ///< 'I' on the F42x/F43x (PI8), 'A' on the F446 (PA0)
    uint8_t af2_pin = 0;
};

constexpr RtcPadFacts rtc_pad_facts() {
    RtcPadFacts f{};
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
    defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
    f.known = true;
    f.tamper_inputs = 2;
    f.has_af2 = true;
    f.af2_port = 'I';
    f.af2_pin = 8;
#elif defined(STM32F446xx)
    f.known = true;
    f.tamper_inputs = 2;
    f.has_af2 = true;
    f.af2_port = 'A';
    f.af2_pin = 0;
#elif defined(STM32F411xE)
    f.known = true;
    f.tamper_inputs = 1;
#endif
    return f;
}

/// RTC_AF1, the pad every manual read puts the tamper, the timestamp and
/// RTC_OUT on: PC13 on the F405 class, the F42x/F43x, the F446 and the
/// F411 alike (RM0090 8.3.15, RM0390 7.3.15, RM0383 figure 159's note).
inline constexpr char rtc_af1_port = 'C';
inline constexpr uint8_t rtc_af1_pin = 13;

/// The three EXTI lines this peripheral's interrupts reach the NVIC
/// through, and they are CONFIGURABLE lines here (unlike the STM32G0's
/// direct ones): the rising edge must be selected, the mask opened and
/// the pending bit cleared in the handler. RM0383 10.2.5, RM0090 12.2.5
/// and RM0390 11.2.5 agree line for line.
inline constexpr uint8_t rtc_alarm_exti_line = 17;
inline constexpr uint8_t rtc_tamper_stamp_exti_line = 21;
inline constexpr uint8_t rtc_wakeup_exti_line = 22;

/// The three vectors. Every header of the pack declares all three, so
/// these need no presence guard - what differs across the family is what
/// is BEHIND them, not whether they exist.
constexpr IRQn_Type rtc_alarm_irq() { return RTC_Alarm_IRQn; }
constexpr IRQn_Type rtc_wakeup_irq() { return RTC_WKUP_IRQn; }
constexpr IRQn_Type rtc_tamper_stamp_irq() { return TAMP_STAMP_IRQn; }

// ---- DMA ----------------------------------------------------------------------

/// Register block base of DMA controller `n` (1 or 2), 0 when the device
/// header does not declare it. Every part of this pack has both.
constexpr uint32_t dma_base(uint8_t n) {
    switch (n) {
#if defined(DMA1_BASE)
        case 1: return DMA1_BASE;
#endif
#if defined(DMA2_BASE)
        case 2: return DMA2_BASE;
#endif
        default: return 0;
    }
}

constexpr bool dma_present(uint8_t n) { return dma_base(n) != 0u; }

/// Streams per controller, and the depth of a stream's FIFO in words -
/// the same on both controllers and on every part (RM0090 10.2).
inline constexpr uint8_t dma_streams = 8;
inline constexpr uint8_t dma_fifo_words = 4;
/// Channels (request lines) a stream chooses between with CHSEL (10.3.3).
inline constexpr uint8_t dma_channels = 8;

/// Register block base of stream `s` of controller `n`. The header spells
/// one macro per stream, so each is probed by name rather than computed
/// from an offset.
constexpr uint32_t dma_stream_base(uint8_t n, uint8_t s) {
    if (n == 1) {
        switch (s) {
#if defined(DMA1_Stream0_BASE)
            case 0: return DMA1_Stream0_BASE;
            case 1: return DMA1_Stream1_BASE;
            case 2: return DMA1_Stream2_BASE;
            case 3: return DMA1_Stream3_BASE;
            case 4: return DMA1_Stream4_BASE;
            case 5: return DMA1_Stream5_BASE;
            case 6: return DMA1_Stream6_BASE;
            case 7: return DMA1_Stream7_BASE;
#endif
            default: return 0;
        }
    }
    if (n == 2) {
        switch (s) {
#if defined(DMA2_Stream0_BASE)
            case 0: return DMA2_Stream0_BASE;
            case 1: return DMA2_Stream1_BASE;
            case 2: return DMA2_Stream2_BASE;
            case 3: return DMA2_Stream3_BASE;
            case 4: return DMA2_Stream4_BASE;
            case 5: return DMA2_Stream5_BASE;
            case 6: return DMA2_Stream6_BASE;
            case 7: return DMA2_Stream7_BASE;
#endif
            default: return 0;
        }
    }
    return 0;
}

constexpr bool dma_stream_present(uint8_t n, uint8_t s) {
    return dma_stream_base(n, s) != 0u;
}

/// The controller's enable bit in RCC_AHB1ENR; the same position resets
/// it in RCC_AHB1RSTR.
constexpr uint32_t dma_clock_mask(uint8_t n) {
    switch (n) {
#if defined(RCC_AHB1ENR_DMA1EN)
        case 1: return RCC_AHB1ENR_DMA1EN;
#endif
#if defined(RCC_AHB1ENR_DMA2EN)
        case 2: return RCC_AHB1ENR_DMA2EN;
#endif
        default: return 0;
    }
}

constexpr uint32_t dma_reset_mask(uint8_t n) {
    switch (n) {
#if defined(RCC_AHB1RSTR_DMA1RST)
        case 1: return RCC_AHB1RSTR_DMA1RST;
#endif
#if defined(RCC_AHB1RSTR_DMA2RST)
        case 2: return RCC_AHB1RSTR_DMA2RST;
#endif
        default: return 0;
    }
}

/// ONE VECTOR PER STREAM - sixteen lines, no sharing at all on this
/// family (RM0090 table 61). An IRQn is an enumerator the preprocessor
/// cannot probe, so each is guarded by its controller's base macro, as
/// every vector verb here is.
constexpr IRQn_Type dma_stream_irq(uint8_t n, uint8_t s) {
    if (n == 1) {
        switch (s) {
#if defined(DMA1_BASE)
            case 0: return DMA1_Stream0_IRQn;
            case 1: return DMA1_Stream1_IRQn;
            case 2: return DMA1_Stream2_IRQn;
            case 3: return DMA1_Stream3_IRQn;
            case 4: return DMA1_Stream4_IRQn;
            case 5: return DMA1_Stream5_IRQn;
            case 6: return DMA1_Stream6_IRQn;
            case 7: return DMA1_Stream7_IRQn;
#endif
            default: return NonMaskableInt_IRQn;
        }
    }
    if (n == 2) {
        switch (s) {
#if defined(DMA2_BASE)
            case 0: return DMA2_Stream0_IRQn;
            case 1: return DMA2_Stream1_IRQn;
            case 2: return DMA2_Stream2_IRQn;
            case 3: return DMA2_Stream3_IRQn;
            case 4: return DMA2_Stream4_IRQn;
            case 5: return DMA2_Stream5_IRQn;
            case 6: return DMA2_Stream6_IRQn;
            case 7: return DMA2_Stream7_IRQn;
#endif
            default: return NonMaskableInt_IRQn;
        }
    }
    return NonMaskableInt_IRQn;
}

/// ONLY DMA2 CAN MOVE MEMORY TO MEMORY, and it is a wiring fact and not a
/// register one: DMA1's AHB peripheral port is not connected to the bus
/// matrix (RM0090 figures 33 and 34, note 1 under each), so its peripheral
/// port cannot read a memory. Nothing in the header says so and nothing in
/// DMA1's registers refuses it - the stream would simply take a bus error -
/// which is why this is stated here and enforced by the driver.
constexpr bool dma_memory_to_memory_capable(uint8_t n) { return n == 2; }

/**
 * WHERE A SERIAL INSTANCE'S DMA REQUEST SITS IN THE FABRIC - the serial
 * slice of the request mapping tables, keyed on the device-select define
 * for the reason the frequency ladders are: no device header of this pack
 * carries a request mapping, and the tables DIFFER BY PART (RM0090
 * tables 43 and 44, RM0390 tables 28 and 29, RM0383 tables 27 and 28).
 *
 * A request is a CELL of those tables, not a number: a controller, a
 * stream and the channel that stream must select for the peripheral to
 * reach it. Several cells may carry the same request (USART1_RX is on two
 * streams of DMA2), so a placement list is up to two entries long - the
 * most any serial row of the three manuals has.
 *
 * Filled for the four part classes whose manual was read; on every other
 * header `known` is false and stm32f4/usart.hpp REFUSES a DMA engine
 * rather than run a stream on a guessed channel. The class boundary is
 * NOT the ladder's: the F42x/F43x carry UART7 and UART8 rows that the
 * F405 class has not, and the F411's USART2_RX has a second placement
 * (DMA1 stream 7, channel 6) that no other manual shows.
 */
struct DmaPlacement {
    uint8_t controller = 0;   ///< 1 or 2
    uint8_t stream = 0;       ///< 0..7
    uint8_t channel = 0;      ///< CHSEL, 0..7
};

struct DmaPlacements {
    bool known = false;       ///< false: this part class's table was not read
    uint8_t count = 0;
    DmaPlacement at[3] = {};  ///< three: RM0383's SPI1_TX, SPI4_RX and SPI5_TX have three cells each
};

constexpr DmaPlacements usart_dma_placements(uint8_t n, bool transmit) {
    DmaPlacements p{};
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || \
    defined(STM32F417xx) || defined(STM32F427xx) || defined(STM32F437xx) || \
    defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
    defined(STM32F411xE)
    p.known = true;
    if (!usart_present(n)) {
        return p;   // the class's table is read, this part has no such instance
    }
    switch (n) {
        case 1:
            if (transmit) { p.count = 1; p.at[0] = {2, 7, 4}; }
            else          { p.count = 2; p.at[0] = {2, 2, 4}; p.at[1] = {2, 5, 4}; }
            break;
        case 2:
            if (transmit) { p.count = 1; p.at[0] = {1, 6, 4}; }
            else {
                p.count = 1;
                p.at[0] = {1, 5, 4};
#if defined(STM32F411xE)
                // RM0383 table 27, channel 6 stream 7 - the F411's alone.
                p.count = 2;
                p.at[1] = {1, 7, 6};
#endif
            }
            break;
        case 3:
            if (transmit) { p.count = 2; p.at[0] = {1, 3, 4}; p.at[1] = {1, 4, 7}; }
            else          { p.count = 1; p.at[0] = {1, 1, 4}; }
            break;
        case 4:
            if (transmit) { p.count = 1; p.at[0] = {1, 4, 4}; }
            else          { p.count = 1; p.at[0] = {1, 2, 4}; }
            break;
        case 5:
            if (transmit) { p.count = 1; p.at[0] = {1, 7, 4}; }
            else          { p.count = 1; p.at[0] = {1, 0, 4}; }
            break;
        case 6:
            if (transmit) { p.count = 2; p.at[0] = {2, 6, 5}; p.at[1] = {2, 7, 5}; }
            else          { p.count = 2; p.at[0] = {2, 1, 5}; p.at[1] = {2, 2, 5}; }
            break;
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
        // RM0090 table 43's channel 5, marked "available on STM32F42xxx and
        // STM32F43xxx only" - and the instances themselves are that class's.
        case 7:
            if (transmit) { p.count = 1; p.at[0] = {1, 1, 5}; }
            else          { p.count = 1; p.at[0] = {1, 3, 5}; }
            break;
        case 8:
            if (transmit) { p.count = 1; p.at[0] = {1, 0, 5}; }
            else          { p.count = 1; p.at[0] = {1, 6, 5}; }
            break;
#endif
        default: break;   // an instance this class's table has no row for
    }
#else
    (void)n;
    (void)transmit;
#endif
    return p;
}

/// Whether (controller, stream, channel) is one of the cells instance `n`
/// can be served on. False on a part whose table was not read, and false
/// for an instance the read table has no row for - both refusals, never a
/// guess.
constexpr bool usart_dma_placement_valid(uint8_t n, bool transmit, uint8_t controller,
                                         uint8_t stream, uint8_t channel) {
    const DmaPlacements p = usart_dma_placements(n, transmit);
    if (!p.known) {
        return false;
    }
    for (uint8_t i = 0; i < p.count; ++i) {
        if (p.at[i].controller == controller && p.at[i].stream == stream &&
            p.at[i].channel == channel) {
            return true;
        }
    }
    return false;
}

// ---- timers -------------------------------------------------------------------
//
// WHAT THE HEADER CAN ANSWER AND WHAT IT CANNOT, and this chapter is where
// the two halves are furthest apart:
//  - WHICH TIMERS EXIST is the header's, three ways over: TIMn_BASE, the
//    RCC enable/reset masks and the IRQn enumerators. The F42x/F43x, F446,
//    F405 class, F412, F413/F423 and F469/F479 carry TIM1..TIM14; the
//    F401 and F411 have TIM1..TIM5 and TIM9..TIM11 and no other; the F410
//    has TIM1, TIM5, TIM6, TIM9 and TIM11 alone. Every one of those
//    statements is a probe below and not a list.
//  - WHAT A TIMER IS - counter width, how many channels, complementary
//    outputs, a slave controller, a repetition counter, an external
//    trigger, DMA - IS NOT IN THE DEVICE HEADER AT ALL. There is ONE
//    TIM_TypeDef and every register is a member of it on every instance,
//    so `TIM11->SMCR` compiles and writes a hole in the address map. Those
//    facts are the REFERENCE MANUALS' (RM0090 ch. 17..20, RM0390 ch. 15..18,
//    RM0383 ch. 12..14, each chapter's ".2 main features" and register
//    map), they are the same on every part of the family, and they are
//    spelled out below keyed by instance NUMBER and gated by the header's
//    own presence probe. Every verb of stm32f4/tim.hpp that names a
//    register an instance does not implement refuses instead of storing.
//  - THE PAD MAP IS THE DATASHEET'S (DS10314 table 9, DS10693 table 11,
//    the F429's table 12) and is not here at all, for the reason
//    stm32f4/pin.hpp gives once for the whole stratum: no symbol of the
//    device header carries it. A timer pad is a `PinSel` the caller
//    writes and the bench is the only check there is.

/// Register block base of TIMn (n = 1..14), 0 when the device has not got
/// it. TIM1, TIM5, TIM9 and TIM11 are on every part of the pack.
constexpr uint32_t tim_base(uint8_t n) {
    switch (n) {
#if defined(TIM1_BASE)
        case 1: return TIM1_BASE;
#endif
#if defined(TIM2_BASE)
        case 2: return TIM2_BASE;
#endif
#if defined(TIM3_BASE)
        case 3: return TIM3_BASE;
#endif
#if defined(TIM4_BASE)
        case 4: return TIM4_BASE;
#endif
#if defined(TIM5_BASE)
        case 5: return TIM5_BASE;
#endif
#if defined(TIM6_BASE)
        case 6: return TIM6_BASE;
#endif
#if defined(TIM7_BASE)
        case 7: return TIM7_BASE;
#endif
#if defined(TIM8_BASE)
        case 8: return TIM8_BASE;
#endif
#if defined(TIM9_BASE)
        case 9: return TIM9_BASE;
#endif
#if defined(TIM10_BASE)
        case 10: return TIM10_BASE;
#endif
#if defined(TIM11_BASE)
        case 11: return TIM11_BASE;
#endif
#if defined(TIM12_BASE)
        case 12: return TIM12_BASE;
#endif
#if defined(TIM13_BASE)
        case 13: return TIM13_BASE;
#endif
#if defined(TIM14_BASE)
        case 14: return TIM14_BASE;
#endif
        default: return 0;
    }
}

constexpr bool tim_present(uint8_t n) { return tim_base(n) != 0u; }

/// Which APB carries TIMn, and its enable/reset bit in that bus's ENR and
/// RSTR. TIM1, TIM8 and TIM9..TIM11 are the APB2 instances of this family;
/// TIM2..TIM7 and TIM12..TIM14 sit on APB1 (RM0090 table 1). `enable_mask`
/// 0 means "no such instance".
struct TimBusClock {
    bool apb2 = false;
    uint32_t enable_mask = 0;
    uint32_t reset_mask = 0;
};

constexpr TimBusClock tim_bus_clock(uint8_t n) {
    switch (n) {
#if defined(RCC_APB2ENR_TIM1EN)
        case 1: return {true, RCC_APB2ENR_TIM1EN, RCC_APB2RSTR_TIM1RST};
#endif
#if defined(RCC_APB1ENR_TIM2EN)
        case 2: return {false, RCC_APB1ENR_TIM2EN, RCC_APB1RSTR_TIM2RST};
#endif
#if defined(RCC_APB1ENR_TIM3EN)
        case 3: return {false, RCC_APB1ENR_TIM3EN, RCC_APB1RSTR_TIM3RST};
#endif
#if defined(RCC_APB1ENR_TIM4EN)
        case 4: return {false, RCC_APB1ENR_TIM4EN, RCC_APB1RSTR_TIM4RST};
#endif
#if defined(RCC_APB1ENR_TIM5EN)
        case 5: return {false, RCC_APB1ENR_TIM5EN, RCC_APB1RSTR_TIM5RST};
#endif
#if defined(RCC_APB1ENR_TIM6EN)
        case 6: return {false, RCC_APB1ENR_TIM6EN, RCC_APB1RSTR_TIM6RST};
#endif
#if defined(RCC_APB1ENR_TIM7EN)
        case 7: return {false, RCC_APB1ENR_TIM7EN, RCC_APB1RSTR_TIM7RST};
#endif
#if defined(RCC_APB2ENR_TIM8EN)
        case 8: return {true, RCC_APB2ENR_TIM8EN, RCC_APB2RSTR_TIM8RST};
#endif
#if defined(RCC_APB2ENR_TIM9EN)
        case 9: return {true, RCC_APB2ENR_TIM9EN, RCC_APB2RSTR_TIM9RST};
#endif
#if defined(RCC_APB2ENR_TIM10EN)
        case 10: return {true, RCC_APB2ENR_TIM10EN, RCC_APB2RSTR_TIM10RST};
#endif
#if defined(RCC_APB2ENR_TIM11EN)
        case 11: return {true, RCC_APB2ENR_TIM11EN, RCC_APB2RSTR_TIM11RST};
#endif
#if defined(RCC_APB1ENR_TIM12EN)
        case 12: return {false, RCC_APB1ENR_TIM12EN, RCC_APB1RSTR_TIM12RST};
#endif
#if defined(RCC_APB1ENR_TIM13EN)
        case 13: return {false, RCC_APB1ENR_TIM13EN, RCC_APB1RSTR_TIM13RST};
#endif
#if defined(RCC_APB1ENR_TIM14EN)
        case 14: return {false, RCC_APB1ENR_TIM14EN, RCC_APB1RSTR_TIM14RST};
#endif
        default: return {};
    }
}

/// Counter width in BITS: TIM2 and TIM5 are the family's two 32-bit
/// counters, every other timer is 16-bit (RM0090 18.1). 0 for an instance
/// this device has not got.
constexpr uint8_t tim_counter_bits(uint8_t n) {
    if (!tim_present(n)) {
        return 0;
    }
    return (n == 2u || n == 5u) ? 32u : 16u;
}

/// The largest ARR / CNT / CCR this counter holds.
constexpr uint32_t tim_max_period(uint8_t n) {
    const uint8_t bits = tim_counter_bits(n);
    return bits == 32u ? 0xFFFFFFFFUL : (bits == 16u ? 0xFFFFUL : 0UL);
}

/// Capture/compare channels: four on TIM1/TIM8 and TIM2..TIM5, two on
/// TIM9/TIM12, one on TIM10/TIM11/TIM13/TIM14, none at all on the basic
/// TIM6/TIM7 (each chapter's ".2 main features").
constexpr uint8_t tim_channels(uint8_t n) {
    if (!tim_present(n)) {
        return 0;
    }
    switch (n) {
        case 1: case 2: case 3: case 4: case 5: case 8: return 4;
        case 9: case 12: return 2;
        case 10: case 11: case 13: case 14: return 1;
        default: return 0;   // TIM6, TIM7
    }
}

/// Complementary outputs (CCxNE, and the OISxN idle levels): channels
/// 1..3 of the two advanced-control timers and nowhere else (RM0090 17.2).
constexpr uint8_t tim_complementary_channels(uint8_t n) {
    if (!tim_present(n)) {
        return 0;
    }
    return (n == 1u || n == 8u) ? 3u : 0u;
}

/// TIMx_SMCR - a slave controller at all. TIM10/TIM11/TIM13/TIM14 have no
/// such register and the basic timers have none either.
constexpr bool tim_has_slave_mode(uint8_t n) {
    if (!tim_present(n)) {
        return false;
    }
    switch (n) {
        case 1: case 2: case 3: case 4: case 5: case 8: case 9: case 12: return true;
        default: return false;
    }
}

/// SMS's encoder codes 001..011, which are RESERVED on TIM9/TIM12
/// (RM0383 14.4.2) - so only the four-channel timers count a quadrature
/// pair.
constexpr bool tim_has_encoder(uint8_t n) {
    if (!tim_present(n)) {
        return false;
    }
    switch (n) {
        case 1: case 2: case 3: case 4: case 5: case 8: return true;
        default: return false;
    }
}

/// TIMx_CR2 and its MMS field - TRGO. TIM9..TIM14 have NO CR2 at all
/// (their register maps jump from CR1 to SMCR), so they can drive no
/// slave; the basic timers have one with MMS alone.
constexpr bool tim_has_master_mode(uint8_t n) {
    if (!tim_present(n)) {
        return false;
    }
    switch (n) {
        case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8: return true;
        default: return false;
    }
}

/// TIMx_BDTR: the break input and the dead-time generator, the two
/// advanced-control timers' alone.
constexpr bool tim_has_break(uint8_t n) { return tim_complementary_channels(n) != 0u; }

/// TIMx_RCR, the repetition counter - the same two.
constexpr bool tim_has_repetition(uint8_t n) { return tim_has_break(n); }

/// CR1.DIR and CR1.CMS: down-counting and centre-aligned counting, on the
/// four-channel timers alone (TIM9..TIM14 count up, the basic timers too).
constexpr bool tim_has_direction(uint8_t n) { return tim_has_encoder(n); }
constexpr bool tim_has_center_aligned(uint8_t n) { return tim_has_encoder(n); }

/// CR1.CKD, the tDTS divider the input filters and the dead-time generator
/// are counted in: every timer with a channel, and not the basic pair.
constexpr bool tim_has_clock_division(uint8_t n) {
    return tim_present(n) && tim_channels(n) != 0u;
}

/// The ETR input and SMCR's ETF/ETPS/ECE/ETP half (external clock mode 2):
/// the four-channel timers alone. CCMRx's OCxCE (clear the output on
/// ocref_clr) is the same set, ocref_clr being ETRF.
constexpr bool tim_has_external_trigger(uint8_t n) { return tim_has_encoder(n); }

/// CR2.TI1S, the XOR of the first three channel inputs on TI1 (RM0090
/// 17.3.17): the four-channel timers'.
constexpr bool tim_has_ti1_xor(uint8_t n) { return tim_has_encoder(n); }

/// DIER's UDE/TDE/CCxDE request enables. TIM9..TIM14 have no DMA of any
/// kind (their DIER is four bits wide); the basic timers have UDE alone.
constexpr bool tim_has_dma_request(uint8_t n) {
    return tim_present(n) && n <= 8u;
}

/// TIMx_DCR and TIMx_DMAR, the burst engine that turns ONE request into a
/// walk over consecutive registers: the four-channel timers'. The basic
/// timers' register map stops at ARR.
constexpr bool tim_has_dma_burst(uint8_t n) { return tim_has_encoder(n); }

/// TIMx_OR, the option register - the input and trigger remaps that make
/// this chapter measurable with no wire. Only TIM2 (ITR1_RMP), TIM5
/// (TI4_RMP) and TIM11 (TI1_RMP) have one (RM0090 18.4.19, 18.4.20,
/// 19.5.11).
constexpr bool tim_has_option_register(uint8_t n) {
    return tim_present(n) && (n == 2u || n == 5u || n == 11u);
}

/// Where that instance's field sits in TIMx_OR, 0xFF for an instance with
/// no option register. The three positions are the header's own *_Pos
/// macros, and the pack declares each only where the instance it belongs
/// to exists - TIM_OR_ITR1_RMP_Pos is absent on exactly the three F410
/// headers, which are exactly the parts with no TIM2.
constexpr uint8_t tim_option_pos(uint8_t n) {
    if (!tim_present(n)) {
        return 0xFFu;
    }
    switch (n) {
#if defined(TIM_OR_ITR1_RMP_Pos)
        case 2: return TIM_OR_ITR1_RMP_Pos;
#endif
#if defined(TIM_OR_TI4_RMP_Pos)
        case 5: return TIM_OR_TI4_RMP_Pos;
#endif
#if defined(TIM_OR_TI1_RMP_Pos)
        case 11: return TIM_OR_TI1_RMP_Pos;
#endif
        default: return 0xFFu;
    }
}

/// THE VECTORS. On this family the two advanced-control timers have FOUR
/// lines each - break, update, trigger/commutation, capture/compare - and
/// three of the four are SHARED with a small general-purpose timer
/// (TIM1_BRK_TIM9, TIM1_UP_TIM10, TIM1_TRG_COM_TIM11 and the TIM8 twins
/// with TIM12, TIM13, TIM14). Every other timer reports everything on one
/// line of its own, except TIM6, which shares with the DAC where there is
/// one. IRQn values are enumerators the preprocessor cannot probe, so each
/// name is DERIVED FROM THE PRESENCE of what shares it, exactly as
/// usart_irq() and exti_line_irq() are.
///
/// tim_irq() is where the UPDATE event arrives, which is the line a timer
/// with one vector answers everything on. NonMaskableInt_IRQn for a timer
/// the device has not got - unreachable, Tim<n> refusing it first.
constexpr IRQn_Type tim_irq(uint8_t n) {
    switch (n) {
        case 1:
#if defined(TIM10_BASE)
            return TIM1_UP_TIM10_IRQn;
#else
            return TIM1_UP_IRQn;
#endif
#if defined(TIM2_BASE)
        case 2: return TIM2_IRQn;
#endif
#if defined(TIM3_BASE)
        case 3: return TIM3_IRQn;
#endif
#if defined(TIM4_BASE)
        case 4: return TIM4_IRQn;
#endif
#if defined(TIM5_BASE)
        case 5: return TIM5_IRQn;
#endif
#if defined(TIM6_BASE) && defined(DAC_BASE)
        case 6: return TIM6_DAC_IRQn;
#elif defined(TIM6_BASE)
        case 6: return TIM6_IRQn;
#endif
#if defined(TIM7_BASE)
        case 7: return TIM7_IRQn;
#endif
#if defined(TIM8_BASE)
        case 8: return TIM8_UP_TIM13_IRQn;
#endif
#if defined(TIM9_BASE)
        case 9: return TIM1_BRK_TIM9_IRQn;
#endif
#if defined(TIM10_BASE)
        case 10: return TIM1_UP_TIM10_IRQn;
#endif
#if defined(TIM11_BASE)
        case 11: return TIM1_TRG_COM_TIM11_IRQn;
#endif
#if defined(TIM12_BASE)
        case 12: return TIM8_BRK_TIM12_IRQn;
#endif
#if defined(TIM13_BASE)
        case 13: return TIM8_UP_TIM13_IRQn;
#endif
#if defined(TIM14_BASE)
        case 14: return TIM8_TRG_COM_TIM14_IRQn;
#endif
        default: return NonMaskableInt_IRQn;
    }
}

/// The capture/compare line: its own on the advanced-control timers,
/// tim_irq() everywhere else - which is what lets a handler bind both
/// without asking whether they are two.
constexpr IRQn_Type tim_cc_irq(uint8_t n) {
    switch (n) {
        case 1: return TIM1_CC_IRQn;
#if defined(TIM8_BASE)
        case 8: return TIM8_CC_IRQn;
#endif
        default: return tim_irq(n);
    }
}

/// The break line, and the trigger/commutation line: the same two timers'
/// third and fourth vectors.
constexpr IRQn_Type tim_break_irq(uint8_t n) {
    switch (n) {
        case 1: return TIM1_BRK_TIM9_IRQn;
#if defined(TIM8_BASE)
        case 8: return TIM8_BRK_TIM12_IRQn;
#endif
        default: return tim_irq(n);
    }
}

constexpr IRQn_Type tim_trigger_irq(uint8_t n) {
    switch (n) {
        case 1: return TIM1_TRG_COM_TIM11_IRQn;
#if defined(TIM8_BASE)
        case 8: return TIM8_TRG_COM_TIM14_IRQn;
#endif
        default: return tim_irq(n);
    }
}

/// Are this timer's four event groups spread over four DIFFERENT vectors?
/// True for the advanced-control pair alone.
constexpr bool tim_has_split_vectors(uint8_t n) {
    return tim_present(n) && (n == 1u || n == 8u);
}

/// Whether RCC_DCKCFGR carries TIMPRE, the bit that decides whether a
/// timer on a divided APB runs at twice or four times its bus clock
/// (RM0383 6.3.24, RM0090 7.3.24). The F405/F407/F415/F417 headers declare
/// no such bit - that class's timers are always at twice - and every other
/// header of the pack does.
constexpr bool rcc_has_timpre() {
#if defined(RCC_DCKCFGR_TIMPRE)
    return true;
#else
    return false;
#endif
}

// ---- the analog converters ----------------------------------------------------
//
// WHAT DIFFERS ACROSS THE FAMILY HERE, and where each difference is read
// from:
//  - How many ADCs: one on the F401, F410, F411, F412 and F413/F423,
//    three on the F405 class, the F42x/F43x, the F446 and the F469/F479.
//    The header says it (ADC2_BASE, ADC3_BASE) and so does the name of
//    the COMMON block, which the pack spells ADC1_COMMON_BASE on a
//    one-converter part and ADC123_COMMON_BASE on a three-converter one.
//  - Whether there is a DAC at all: DAC_BASE. The F401, F411 and F412
//    have none.
//  - Which TIMERS exist, which is what decides how many of the sixteen
//    EXTSEL / JEXTSEL codes and the eight TSEL codes carry anything: the
//    F410 has TIM1, TIM5, TIM6 and TIM9 alone, so most of the trigger
//    space is dead silicon there. The base-address macro of each timer
//    is the probe.
//
// AND WHAT THE HEADER CANNOT BE ASKED, keyed on the device-select define
// like the frequency ladders above: which channel the temperature sensor
// is on, what the VBAT bridge divides by, and how many channels the DAC
// really has. ST declares ONE set of bit names for the whole family - the
// F410's header carries DAC_CR_EN2 though RM0401 gives that part one
// output - exactly as it declares the RTC's second tamper input
// everywhere. So these are the reference manual's numbers, stated only
// for the classes whose manual is on the desk, and a class outside them
// gets `known == false` and the conservative answer rather than a guess:
// a temperature read on the wrong channel is a wrong number in silence,
// and a second DAC channel that does not exist drives nothing.

/// Register block base of ADC instance n (1..3), 0 when this device does
/// not have it.
constexpr uint32_t adc_base(uint8_t n) {
    switch (n) {
#if defined(ADC1_BASE)
        case 1: return ADC1_BASE;
#endif
#if defined(ADC2_BASE)
        case 2: return ADC2_BASE;
#endif
#if defined(ADC3_BASE)
        case 3: return ADC3_BASE;
#endif
        default: return 0;
    }
}

constexpr bool adc_present(uint8_t n) { return adc_base(n) != 0u; }

/// How many converters this device bonds - 1 or 3; the multi-ADC modes
/// need at least two and the triple ones all three.
constexpr uint8_t adc_instances() {
    uint8_t n = 0;
    for (uint8_t i = 1; i <= 3; ++i) {
        if (adc_present(i)) {
            ++n;
        }
    }
    return n;
}

/// The COMMON block (CSR, CCR, CDR) - one per device, shared by every
/// converter, at ADC1's base + 0x300. The pack names the macro after the
/// converters that share it, so both spellings are probed.
constexpr uint32_t adc_common_base() {
#if defined(ADC123_COMMON_BASE)
    return ADC123_COMMON_BASE;
#elif defined(ADC1_COMMON_BASE)
    return ADC1_COMMON_BASE;
#else
    return 0;
#endif
}

/// Instance n's enable bit in RCC_APB2ENR (one per converter, 7.3.14);
/// 0 when absent.
constexpr uint32_t adc_clock_mask(uint8_t n) {
    switch (n) {
#if defined(RCC_APB2ENR_ADC1EN)
        case 1: return RCC_APB2ENR_ADC1EN;
#endif
#if defined(RCC_APB2ENR_ADC2EN)
        case 2: return RCC_APB2ENR_ADC2EN;
#endif
#if defined(RCC_APB2ENR_ADC3EN)
        case 3: return RCC_APB2ENR_ADC3EN;
#endif
        default: return 0;
    }
}

/// The reset line, in RCC_APB2RSTR - ONE bit for every converter and the
/// common block together (7.3.7: there is no per-instance ADC reset on
/// this family), which is why the reset verb belongs to the block.
inline constexpr uint32_t adc_reset_mask = RCC_APB2RSTR_ADCRST;

/// The NVIC line, SHARED BY EVERY CONVERTER (one "ADC global interrupt"
/// on every part of the family), so a handler bound to it is a
/// dispatcher.
constexpr IRQn_Type adc_irq() { return ADC_IRQn; }

/// The channel numbers this converter's fields hold: 0..18, the range
/// AWDCH[4:0] and the SQx/JSQx fields document (13.13.2's own note).
inline constexpr uint8_t adc_channel_count = 19;

/**
 * The internal channels' numbers and the battery bridge's ratio - the
 * REFERENCE MANUAL'S, keyed on the part class (see the note above).
 *
 * RM0090 13.3.4 and 13.11: the F405 class puts the temperature sensor on
 * ADC1_IN16 and divides VBAT by two; the F42x/F43x move the sensor onto
 * ADC1_IN18, where it SHARES the channel with VBAT, and divide by four.
 * RM0390 13.3.4 and RM0383 11.3.4 give the F446 and the F411 the
 * F42x/F43x's arrangement. VREFINT is ADC1_IN17 on every one of them, and
 * all three internal channels are the MASTER converter's alone.
 */
struct AdcInternalFacts {
    bool known = false;
    uint8_t vrefint_channel = 0xFFu;
    uint8_t temperature_channel = 0xFFu;
    uint8_t vbat_channel = 0xFFu;
    uint8_t vbat_divider = 0;      ///< VBAT / this reaches the channel
    /// Whether the sensor and the battery share one channel, which makes
    /// TSVREFE and VBATE mutually exclusive (VBAT wins where both are set).
    bool sensor_shares_vbat = false;
};

constexpr AdcInternalFacts adc_internal_facts() {
    AdcInternalFacts f{};
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)
    f.known = true;
    f.vrefint_channel = 17;
    f.temperature_channel = 16;
    f.vbat_channel = 18;
    f.vbat_divider = 2;
#elif defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
    defined(STM32F446xx) || defined(STM32F411xE)
    f.known = true;
    f.vrefint_channel = 17;
    f.temperature_channel = 18;
    f.vbat_channel = 18;
    f.vbat_divider = 4;
    f.sensor_shares_vbat = true;
#endif
    return f;
}

/**
 * How many output channels the DAC really has, and whether that is known.
 *
 * RM0090 14.1, RM0390 14.1: two on the F405 class, the F42x/F43x and the
 * F446. The F411 (RM0383) has no DAC at all and its header says so.
 * Everywhere else the header declares a DAC and both channels' bits and
 * the manual has not been read, so one channel is offered and the second
 * is refused - the RTC's second tamper input, again.
 */
struct DacChannelFacts {
    bool known = false;
    uint8_t channels = 1;
};

constexpr DacChannelFacts dac_channel_facts() {
    DacChannelFacts f{};
#if defined(DAC_BASE)
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
    defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
    defined(STM32F446xx)
    f.known = true;
    f.channels = 2;
#endif
#else
    f.channels = 0;
#endif
    return f;
}

/// Whether this device has the DAC block at all (the F401, F411 and F412
/// have none, and their headers declare no DAC_BASE).
constexpr bool dac_present() {
#if defined(DAC_BASE)
    return true;
#else
    return false;
#endif
}

constexpr uint32_t dac_base() {
#if defined(DAC_BASE)
    return DAC_BASE;
#else
    return 0;
#endif
}

/// The DAC's enable and reset bits in RCC_APB1ENR / RCC_APB1RSTR
/// (7.3.13, 7.3.6); 0 on a part with no DAC.
constexpr uint32_t dac_clock_mask() {
#if defined(RCC_APB1ENR_DACEN)
    return RCC_APB1ENR_DACEN;
#else
    return 0;
#endif
}

constexpr uint32_t dac_reset_mask() {
#if defined(RCC_APB1RSTR_DACRST)
    return RCC_APB1RSTR_DACRST;
#else
    return 0;
#endif
}

/// The DAC's underrun interrupt line - TIM6's vector, which the pack
/// spells TIM6_DAC_IRQn on exactly the parts that have a DAC and TIM6_IRQn
/// on the rest. A handler bound here answers for both peripherals.
constexpr IRQn_Type dac_irq() {
#if defined(DAC_BASE)
    return TIM6_DAC_IRQn;
#else
    return NonMaskableInt_IRQn;   // unreachable: callers check dac_present first
#endif
}


/**
 * WHERE A CONVERTER'S DMA REQUEST SITS IN THE FABRIC - the ANALOG slice
 * of the request mapping tables, in the shape and for the reason the
 * serial slice above states: no device header carries a request mapping
 * and the tables are the reference manual's, keyed per part class.
 *
 * RM0090 table 44, RM0390 table 29 and RM0383 table 28 agree on every
 * converter they share: ADC1 is DMA2's stream 0 and stream 4 on channel
 * 0, ADC2 is stream 2 and stream 3 on channel 1, ADC3 is stream 0 and
 * stream 1 on channel 2 - so ADC1 and ADC3 CONTEND for stream 0 with two
 * different channels, which is what makes a placement a cell and not a
 * number. Every ADC request is DMA2's; none of them is DMA1's.
 */
constexpr DmaPlacements adc_dma_placements(uint8_t instance) {
    DmaPlacements p{};
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || \
    defined(STM32F417xx) || defined(STM32F427xx) || defined(STM32F437xx) || \
    defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
    defined(STM32F411xE)
    p.known = true;
    if (!adc_present(instance)) {
        return p;   // the class's table is read, this part has no such converter
    }
    switch (instance) {
        case 1: p.count = 2; p.at[0] = {2, 0, 0}; p.at[1] = {2, 4, 0}; break;
        case 2: p.count = 2; p.at[0] = {2, 2, 1}; p.at[1] = {2, 3, 1}; break;
        case 3: p.count = 2; p.at[0] = {2, 0, 2}; p.at[1] = {2, 1, 2}; break;
        default: break;
    }
#else
    (void)instance;
#endif
    return p;
}

/**
 * The DAC's two channels (RM0090 table 43, RM0390 table 28): DAC1 on
 * DMA1's stream 5 and DAC2 on stream 6, channel 7 both, one cell each.
 * The channel index here is this stratum's 0-based one, as everywhere
 * else in stm32f4/dac.hpp.
 */
constexpr DmaPlacements dac_dma_placements(uint8_t channel) {
    DmaPlacements p{};
#if defined(DAC_BASE)
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || \
    defined(STM32F417xx) || defined(STM32F427xx) || defined(STM32F437xx) || \
    defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F446xx)
    p.known = true;
    if (channel >= dac_channel_facts().channels) {
        return p;
    }
    p.count = 1;
    p.at[0] = channel == 0u ? DmaPlacement{1, 5, 7} : DmaPlacement{1, 6, 7};
#else
    (void)channel;
#endif
#else
    (void)channel;
#endif
    return p;
}

/// Whether (controller, stream, channel) is a cell converter `instance`'s
/// request is wired to. False on a part whose table was not read - a
/// refusal and never a guess, as with the serial instances.
constexpr bool adc_dma_placement_valid(uint8_t instance, uint8_t controller, uint8_t stream,
                                       uint8_t channel) {
    const DmaPlacements p = adc_dma_placements(instance);
    if (!p.known) {
        return false;
    }
    for (uint8_t i = 0; i < p.count; ++i) {
        if (p.at[i].controller == controller && p.at[i].stream == stream &&
            p.at[i].channel == channel) {
            return true;
        }
    }
    return false;
}

constexpr bool dac_dma_placement_valid(uint8_t dac_channel, uint8_t controller, uint8_t stream,
                                       uint8_t channel) {
    const DmaPlacements p = dac_dma_placements(dac_channel);
    if (!p.known) {
        return false;
    }
    for (uint8_t i = 0; i < p.count; ++i) {
        if (p.at[i].controller == controller && p.at[i].stream == stream &&
            p.at[i].channel == channel) {
            return true;
        }
    }
    return false;
}

// ---- SPI and I2S --------------------------------------------------------------
//
// WHAT THE HEADER ANSWERS AND WHAT IT DOES NOT, for RM0090 ch. 28 (RM0390
// ch. 26, RM0383 ch. 20):
//  - WHICH INSTANCES EXIST, which bus each sits on, which gate and which
//    vector: the header, three ways over (SPIn_BASE, RCC_APBxENR_SPInEN,
//    SPIn_IRQn). This family is regular here - SPI1, SPI4, SPI5 and SPI6
//    on APB2, SPI2 and SPI3 on APB1, ONE vector each, shared with nothing.
//  - WHETHER AN INSTANCE HAS THE I2S FACE: not the header's. Every
//    SPI_TypeDef of the pack declares I2SCFGR and I2SPR, on instances the
//    manual gives no I2S at all, so the register block cannot be asked.
//    The manuals' memory maps are the authority and they DISAGREE ACROSS
//    CLASSES, so the fact is keyed on the part class like the frequency
//    ladders, and a class whose manual was not read reports `known` false.
//  - HOW THE I2S CLOCK IS SELECTED: the header, by which selector it
//    declares - RCC_CFGR.I2SSRC (one bit, PLLI2S_R or the I2S_CKIN pad),
//    RCC_DCKCFGR's I2S1SRC/I2S2SRC pair (two bits each, four sources) or
//    RCC_DCKCFGR.I2SSRC alone on a part with no PLLI2S at all.
//  - WHERE THE DMA REQUESTS SIT: the request mapping tables, keyed per
//    part class as every other chapter's slice of them is.

constexpr uint32_t spi_base(uint8_t n) {
    switch (n) {
#if defined(SPI1_BASE)
        case 1: return SPI1_BASE;
#endif
#if defined(SPI2_BASE)
        case 2: return SPI2_BASE;
#endif
#if defined(SPI3_BASE)
        case 3: return SPI3_BASE;
#endif
#if defined(SPI4_BASE)
        case 4: return SPI4_BASE;
#endif
#if defined(SPI5_BASE)
        case 5: return SPI5_BASE;
#endif
#if defined(SPI6_BASE)
        case 6: return SPI6_BASE;
#endif
        default: return 0;
    }
}

constexpr bool spi_present(uint8_t n) { return spi_base(n) != 0u; }

/// The APB2 instances are SPI1, SPI4, SPI5 and SPI6; SPI2 and SPI3 are on
/// APB1. Asked of the enable bit's own register, so the answer comes from
/// the header and not from a rule written down twice.
constexpr bool spi_on_apb2(uint8_t n) {
    switch (n) {
#if defined(RCC_APB2ENR_SPI1EN)
        case 1: return true;
#endif
#if defined(RCC_APB2ENR_SPI4EN)
        case 4: return true;
#endif
#if defined(RCC_APB2ENR_SPI5EN)
        case 5: return true;
#endif
#if defined(RCC_APB2ENR_SPI6EN)
        case 6: return true;
#endif
        default: return false;
    }
}

/// The instance's enable bit; the same position resets it in the bus's
/// RCC_APBxRSTR.
constexpr uint32_t spi_clock_mask(uint8_t n) {
    switch (n) {
#if defined(RCC_APB2ENR_SPI1EN)
        case 1: return RCC_APB2ENR_SPI1EN;
#endif
#if defined(RCC_APB1ENR_SPI2EN)
        case 2: return RCC_APB1ENR_SPI2EN;
#endif
#if defined(RCC_APB1ENR_SPI3EN)
        case 3: return RCC_APB1ENR_SPI3EN;
#endif
#if defined(RCC_APB2ENR_SPI4EN)
        case 4: return RCC_APB2ENR_SPI4EN;
#endif
#if defined(RCC_APB2ENR_SPI5EN)
        case 5: return RCC_APB2ENR_SPI5EN;
#endif
#if defined(RCC_APB2ENR_SPI6EN)
        case 6: return RCC_APB2ENR_SPI6EN;
#endif
        default: return 0;
    }
}

/// ONE VECTOR PER INSTANCE. An IRQn is an enumerator the preprocessor
/// cannot probe, so each is guarded by its instance's base macro.
constexpr IRQn_Type spi_irq(uint8_t n) {
    switch (n) {
#if defined(SPI1_BASE)
        case 1: return SPI1_IRQn;
#endif
#if defined(SPI2_BASE)
        case 2: return SPI2_IRQn;
#endif
#if defined(SPI3_BASE)
        case 3: return SPI3_IRQn;
#endif
#if defined(SPI4_BASE)
        case 4: return SPI4_IRQn;
#endif
#if defined(SPI5_BASE)
        case 5: return SPI5_IRQn;
#endif
#if defined(SPI6_BASE)
        case 6: return SPI6_IRQn;
#endif
        default: return NonMaskableInt_IRQn;
    }
}

/// The full-duplex I2S extension blocks (RM0090 28.4.2): a second, always
/// SLAVE, register block sharing SPI2's or SPI3's clock and word select.
/// Absent where the header declares no base - the F446 is the part class
/// that has none and pairs two whole instances instead (RM0390 26.6.2).
constexpr uint32_t i2s_ext_base(uint8_t n) {
    switch (n) {
#if defined(I2S2ext_BASE)
        case 2: return I2S2ext_BASE;
#endif
#if defined(I2S3ext_BASE)
        case 3: return I2S3ext_BASE;
#endif
        default: return 0;
    }
}

constexpr bool i2s_ext_present(uint8_t n) { return i2s_ext_base(n) != 0u; }

/// How this part selects the I2S kernel clock.
enum class I2sClockSelect : uint8_t {
    none,          ///< no selector at all
    cfgr,          ///< RCC_CFGR.I2SSRC: one bit, PLLI2S_R or the I2S_CKIN pad
    dckcfgr_one,   ///< RCC_DCKCFGR.I2SSRC: two bits, on a part with no PLLI2S
    dckcfgr_pair,  ///< RCC_DCKCFGR's I2S1SRC and I2S2SRC: two bits EACH, and
                   ///< the numbering is BY APB BUS and not by instance -
                   ///< I2S1SRC is the APB1 instances', I2S2SRC the APB2 ones'
                   ///< (RM0390 6.3.24). A trap the names invite.
};

constexpr I2sClockSelect i2s_clock_select() {
#if defined(RCC_DCKCFGR_I2S1SRC)
    return I2sClockSelect::dckcfgr_pair;
#elif defined(RCC_DCKCFGR_I2SSRC)
    return I2sClockSelect::dckcfgr_one;
#elif defined(RCC_CFGR_I2SSRC)
    return I2sClockSelect::cfgr;
#else
    return I2sClockSelect::none;
#endif
}

/// Whether this part has the dedicated audio PLL at all (the F410 class
/// has not, and selects its I2S clock between the main PLL and the pad).
constexpr bool plli2s_present() {
#if defined(RCC_CR_PLLI2SON)
    return true;
#else
    return false;
#endif
}

/// Whether PLLI2SCFGR carries its OWN input divider: where it does not,
/// the audio PLL divides the MAIN PLL's M (RM0090 7.3.23).
constexpr bool plli2s_has_m() {
#if defined(RCC_PLLI2SCFGR_PLLI2SM_Pos)
    return true;
#else
    return false;
#endif
}

/**
 * WHICH INSTANCES WEAR THE I2S FACE, and the extension blocks, per part
 * class - the manuals' memory maps:
 *  - the F405 class and the F42x/F43x class: I2S2 and I2S3 alone
 *    (RM0090 28.2.2, "Both I2S (I2S2 and I2S3)"), with I2S2ext/I2S3ext;
 *  - the F411: SPI1/I2S1 through SPI5/I2S5, every instance (RM0383 2.3);
 *  - the F446: I2S2 and I2S3 (RM0390 2.3 names SPI1 and SPI4 with no I2S),
 *    and NO extension block - full duplex there is two instances paired.
 * A class whose manual was not read answers `known` false, and
 * stm32f4/spi.hpp refuses `I2s<n>` on it rather than guess.
 */
struct SpiI2sFacts {
    bool known = false;
    uint8_t instances = 0;      ///< bit n-1 set: instance n has the I2S face
    bool ext_blocks = false;    ///< I2S2ext / I2S3ext exist on this class
};

constexpr SpiI2sFacts spi_i2s_facts() {
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
    defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
    return SpiI2sFacts{true, 0x06u, true};    // SPI2, SPI3
#elif defined(STM32F411xE)
    return SpiI2sFacts{true, 0x1Fu, true};    // SPI1..SPI5
#elif defined(STM32F446xx)
    return SpiI2sFacts{true, 0x06u, false};   // SPI2, SPI3; no extension block
#else
    return SpiI2sFacts{};
#endif
}

constexpr bool spi_i2s_capable(uint8_t n) {
    const SpiI2sFacts f = spi_i2s_facts();
    return f.known && n >= 1u && n <= 6u && (f.instances & (1u << (n - 1u))) != 0u;
}

/**
 * WHERE AN SPI INSTANCE'S DMA REQUESTS SIT - the SPI slice of the request
 * mapping (RM0090 tables 43 and 44, RM0390 tables 28 and 29, RM0383
 * tables 27 and 28), keyed on the part class for the reason the serial
 * slice is: no device header carries a request mapping, and the tables
 * differ by part.
 *
 * UP TO THREE CELLS, where the serial and analog slices need two: the
 * F411's table gives SPI1_TX, SPI4_RX and SPI5_TX three cells each (DMA2
 * stream 2 channel 2 for SPI1_TX, stream 4 channel 4 for SPI4_RX, stream
 * 5 channel 5 for SPI5_TX are the extra ones no other manual shows) - the
 * shared `DmaPlacements` list is three wide for their sake.
 */

constexpr DmaPlacements spi_dma_placements(uint8_t n, bool transmit) {
    DmaPlacements p{};
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || \
    defined(STM32F417xx) || defined(STM32F427xx) || defined(STM32F437xx) || \
    defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
    defined(STM32F411xE)
    p.known = true;
    if (!spi_present(n)) {
        return p;   // the class's table is read, this part has no such instance
    }
    switch (n) {
        case 1:
            if (transmit) {
                p.count = 2;
                p.at[0] = {2, 3, 3};
                p.at[1] = {2, 5, 3};
#if defined(STM32F411xE)
                // RM0383 table 28, channel 2 stream 2 - the F411's alone.
                p.count = 3;
                p.at[2] = {2, 2, 2};
#endif
            } else {
                p.count = 2;
                p.at[0] = {2, 0, 3};
                p.at[1] = {2, 2, 3};
            }
            break;
        case 2:
            p.count = 1;
            p.at[0] = transmit ? DmaPlacement{1, 4, 0} : DmaPlacement{1, 3, 0};
            break;
        case 3:
            p.count = 2;
            if (transmit) { p.at[0] = {1, 5, 0}; p.at[1] = {1, 7, 0}; }
            else          { p.at[0] = {1, 0, 0}; p.at[1] = {1, 2, 0}; }
            break;
        case 4:
            if (transmit) {
                p.count = 2;
                p.at[0] = {2, 1, 4};
                p.at[1] = {2, 4, 5};
            } else {
                p.count = 2;
                p.at[0] = {2, 0, 4};
                p.at[1] = {2, 3, 5};
#if defined(STM32F411xE)
                // RM0383 table 28, channel 4 stream 4 - the F411's alone.
                p.count = 3;
                p.at[2] = {2, 4, 4};
#endif
            }
            break;
        case 5:
            if (transmit) {
                p.count = 2;
                p.at[0] = {2, 4, 2};
                p.at[1] = {2, 6, 7};
#if defined(STM32F411xE)
                // RM0383 table 28, channel 5 stream 5 - the F411's alone.
                p.count = 3;
                p.at[2] = {2, 5, 5};
#endif
            } else {
                p.count = 2;
                p.at[0] = {2, 3, 2};
                p.at[1] = {2, 5, 7};
            }
            break;
        case 6:
            p.count = 1;
            p.at[0] = transmit ? DmaPlacement{2, 5, 1} : DmaPlacement{2, 6, 1};
            break;
        default: break;
    }
#else
    (void)n;
    (void)transmit;
#endif
    return p;
}

/// Whether (controller, stream, channel) is a cell instance `n`'s request
/// is wired to. False on a part whose table was not read, and false for an
/// instance the read table has no row for - both refusals, never a guess.
constexpr bool spi_dma_placement_valid(uint8_t n, bool transmit, uint8_t controller,
                                       uint8_t stream, uint8_t channel) {
    const DmaPlacements p = spi_dma_placements(n, transmit);
    if (!p.known) {
        return false;
    }
    for (uint8_t i = 0; i < p.count; ++i) {
        if (p.at[i].controller == controller && p.at[i].stream == stream &&
            p.at[i].channel == channel) {
            return true;
        }
    }
    return false;
}

// ---- the USB OTG controllers ----------------------------------------------------
//
// This family carries up to TWO instances of the same Synopsys DWC2 core:
// OTG_FS, whose PHY is full speed and on chip, and OTG_HS, which drives a
// ULPI transceiver at high speed and ALSO has a full-speed PHY of its own
// (RM0090 34 and 35, RM0390 31 and 32, RM0383 22). Seen from device mode
// at full speed the two are one programmer's model at two base addresses,
// which is why the driver is one template over the core below and not two
// drivers. What differs is a handful of facts, and the header carries
// most of them.
//
// WHAT THE HEADER ANSWERS: whether the part has each core at all (the
// F410 has neither, the F401/F411/F412/F413 have the FS core alone, the
// F405 class, the F42x/F43x, the F446 and the F469/F479 have both), the
// base addresses, the bus each one's gate sits on, and which of the three
// GCCFG generations the part has - the one thing about VBUS a program
// must know to free the pad.
//
// WHAT THE MANUALS ANSWER, and is therefore stated here as a constant per
// CORE with its section: the FIFO RAM (1.25 Kbyte on the FS core, 4 Kbyte
// on the HS one - RM0383 22.2.1, RM0090 35.2.1) and how many device
// endpoint numbers the core instantiates. The counts below are the ones
// every manual on the desk states for its core (RM0383 22.2.3, RM0090
// 34.2.3 and 35.2.3, RM0390 31.2.3): four bidirectional numbers on the FS
// core - endpoint zero and three more - and six on the HS one. A part
// class whose manual was not read may bond more; taking the number the
// read manuals agree on can only leave an endpoint unused, never invent
// one.
//
// THE PADS are the datasheets' (DS10314 table 9, DS10693 table 11, the
// F429's table 12) and are fixed: the FS PHY is bonded to PA11 and PA12
// at AF10, the HS core's own full-speed PHY to PB14 and PB15 at AF12.

/// The two OTG cores of this family, as the manuals name them.
enum class OtgCore : uint8_t { fs, hs };

/// How this part's GCCFG says "there is no VBUS pad, take VBUS as valid".
/// The register was respun twice inside the family and the device header
/// is the authority on which one a part has:
///  - `no_vbus_sense`: the first layout (VBUSASEN / VBUSBSEN / SOFOUTEN /
///    NOVBUSSENS), on the F401, F405 class, F411 and F42x/F43x. Setting
///    NOVBUSSENS frees the pad.
///  - `vbus_detect`: the later one (PWRDWN and VBDEN alone, the F446 and
///    F469/F479; the F412/F413 add the battery-charging detector to it).
///    Clearing VBDEN frees the pad, and the session-valid OVERRIDE pair
///    in GOTGCTL is what then tells the core a session is on.
///  - `none`: the part has no OTG core.
enum class OtgVbusStyle : uint8_t { none, no_vbus_sense, vbus_detect };

/// Everything a driver needs to reach one core, or `present == false`.
struct OtgFacts {
    bool present = false;
    uint32_t base = 0;
    bool gate_on_ahb1 = false;   ///< the FS core's gate is on AHB2, the HS core's on AHB1
    uint32_t clock_mask = 0;
    uint32_t reset_mask = 0;
    uint32_t ulpi_clock_mask = 0;   ///< the HS core's ULPI clock gate, left CLOSED on the embedded PHY
    uint16_t fifo_words = 0;        ///< the dedicated FIFO RAM, in 32-bit words
    uint8_t endpoints = 0;          ///< device endpoint NUMBERS, endpoint zero counted
    bool phy_select_writable = false;   ///< GUSBCFG.PHYSEL: read-only 1 on the FS core, written on the HS one
    char dm_port = 0;
    uint8_t dm_pin = 0;
    char dp_port = 0;
    uint8_t dp_pin = 0;
    uint8_t pad_function = 0;   ///< the AF number both pads take
};

constexpr OtgFacts otg_facts(OtgCore core) {
    if (core == OtgCore::fs) {
#if defined(USB_OTG_FS_PERIPH_BASE)
        return OtgFacts{true,
                        USB_OTG_FS_PERIPH_BASE,
                        false,
                        RCC_AHB2ENR_OTGFSEN,
                        RCC_AHB2RSTR_OTGFSRST,
                        0,
                        320,
                        4,
                        false,
                        'A', 11, 'A', 12, 10};
#else
        return OtgFacts{};
#endif
    }
#if defined(USB_OTG_HS_PERIPH_BASE)
    return OtgFacts{true,
                    USB_OTG_HS_PERIPH_BASE,
                    true,
                    RCC_AHB1ENR_OTGHSEN,
                    RCC_AHB1RSTR_OTGHRST,
                    RCC_AHB1ENR_OTGHSULPIEN,
                    1024,
                    6,
                    true,
                    'B', 14, 'B', 15, 12};
#else
    return OtgFacts{};
#endif
}

constexpr bool otg_present(OtgCore core) { return otg_facts(core).present; }

/// The core's global interrupt line. NonMaskableInt_IRQn where the part
/// has no such core - the enumerators are not preprocessor-visible, so
/// every arm of this switch is guarded by the instance's base macro, as
/// everywhere else in this file.
constexpr IRQn_Type otg_irq(OtgCore core) {
    if (core == OtgCore::fs) {
#if defined(USB_OTG_FS_PERIPH_BASE)
        return OTG_FS_IRQn;
#else
        return NonMaskableInt_IRQn;
#endif
    }
#if defined(USB_OTG_HS_PERIPH_BASE)
    return OTG_HS_IRQn;
#else
    return NonMaskableInt_IRQn;
#endif
}

/// The core's wake-up line through the EXTI (line 18 for the FS core,
/// line 20 for the HS one - exti_line_irq() answers the same vector).
constexpr IRQn_Type otg_wakeup_irq(OtgCore core) {
    if (core == OtgCore::fs) {
#if defined(USB_OTG_FS_PERIPH_BASE)
        return OTG_FS_WKUP_IRQn;
#else
        return NonMaskableInt_IRQn;
#endif
    }
#if defined(USB_OTG_HS_PERIPH_BASE)
    return OTG_HS_WKUP_IRQn;
#else
    return NonMaskableInt_IRQn;
#endif
}

constexpr uint32_t otg_wakeup_exti_line(OtgCore core) { return core == OtgCore::fs ? 18u : 20u; }

/// Which GCCFG layout this part has. The three generations are told apart
/// by the bit names the header declares and by nothing else.
constexpr OtgVbusStyle otg_vbus_style() {
#if defined(USB_OTG_GCCFG_NOVBUSSENS)
    return OtgVbusStyle::no_vbus_sense;
#elif defined(USB_OTG_GCCFG_VBDEN)
    return OtgVbusStyle::vbus_detect;
#else
    return OtgVbusStyle::none;
#endif
}

/// Whether GOTGCTL carries the B-session-valid OVERRIDE pair (BVALOEN and
/// BVALOVAL), which is how a part of the later generation is told a
/// session is on with the VBUS pad given away.
constexpr bool otg_has_session_override() {
#if defined(USB_OTG_GOTGCTL_BVALOEN)
    return true;
#else
    return false;
#endif
}

/// The two GOTGCTL bits that force B-session-valid, or 0 where the part
/// has no such override.
constexpr uint32_t otg_session_override_bits() {
#if defined(USB_OTG_GOTGCTL_BVALOEN)
    return USB_OTG_GOTGCTL_BVALOEN | USB_OTG_GOTGCTL_BVALOVAL;
#else
    return 0;
#endif
}

/// GOTGCTL's "a B-session is valid" status bit. The two GCCFG generations
/// brought two SPELLINGS of the same bit (BSVLD and BSESVLD) at the same
/// position, so the name is a fact of the header and not of the silicon.
constexpr uint32_t otg_session_valid_bit() {
#if defined(USB_OTG_GOTGCTL_BSVLD)
    return USB_OTG_GOTGCTL_BSVLD;
#elif defined(USB_OTG_GOTGCTL_BSESVLD)
    return USB_OTG_GOTGCTL_BSESVLD;
#else
    return 0;
#endif
}

/// The whole GCCFG value a DEVICE wants: the transceiver powered up, and
/// either the VBUS pad given away (the core then takes VBUS as valid at
/// all times) or the B-device comparator watching it. Which bits say that
/// is the part's GCCFG generation, so the value is computed here and the
/// driver writes it without asking the preprocessor anything.
constexpr uint32_t otg_gccfg_device(bool sense_vbus) {
#if defined(USB_OTG_GCCFG_NOVBUSSENS)
    return USB_OTG_GCCFG_PWRDWN |
           (sense_vbus ? USB_OTG_GCCFG_VBUSBSEN : USB_OTG_GCCFG_NOVBUSSENS);
#elif defined(USB_OTG_GCCFG_VBDEN)
    return USB_OTG_GCCFG_PWRDWN | (sense_vbus ? USB_OTG_GCCFG_VBDEN : 0u);
#else
    (void)sense_vbus;
    return 0;
#endif
}

/// GUSBCFG.TRDT for an AHB rate, RM0383 table 132 (RM0090 and RM0390 carry
/// the same table): the turnaround the core adds so a slow AHB still
/// answers an IN token in time. 0 means the rate is below the 14.2 MHz
/// floor the chapter's own caution states (RM0383 22.3.3), where no TRDT
/// value serves.
constexpr uint8_t otg_turnaround_for(uint32_t ahb_hz) {
    if (ahb_hz >= 32'000'000u) { return 0x6; }
    if (ahb_hz >= 27'500'000u) { return 0x7; }
    if (ahb_hz >= 24'000'000u) { return 0x8; }
    if (ahb_hz >= 21'800'000u) { return 0x9; }
    if (ahb_hz >= 20'000'000u) { return 0xA; }
    if (ahb_hz >= 18'500'000u) { return 0xB; }
    if (ahb_hz >= 17'200'000u) { return 0xC; }
    if (ahb_hz >= 16'000'000u) { return 0xD; }
    if (ahb_hz >= 15'000'000u) { return 0xE; }
    if (ahb_hz >= 14'200'000u) { return 0xF; }
    return 0;
}

// ---- power --------------------------------------------------------------------
//
// The regulator's own facts - the VOS width, the over-drive pair and the
// frequency ladders - are up in "the regulator and the clock ladders",
// where the clock task needs them. What is here is the rest of the PWR
// chapter: which optional fields this part's PWR_CR and PWR_CSR carry, and
// the two things the header cannot be asked for (the PVD's thresholds in
// volts, and which pad each wake-up pin is).

/// PWR_CR's under-drive field (UDEN[1:0]) and PWR_CSR's UDRDY: the
/// F42x/F43x, F446 and F469/F479 classes, the same set as the over-drive
/// pair - the two are the halves of one option.
constexpr bool pwr_has_under_drive() {
#if defined(PWR_CR_UDEN)
    return true;
#else
    return false;
#endif
}

/// PWR_CR's low-voltage-in-deep-sleep pair. RM0383 5.4.1 spells them
/// MRLVDS and LPLVDS, RM0390 5.4.1 spells the same two bits MRUDS and
/// LPUDS; ST's headers use the first spelling everywhere the bits exist,
/// which is every part but the F405/F407/F415/F417 class.
constexpr bool pwr_has_low_voltage_stop() {
#if defined(PWR_CR_MRLVDS)
    return true;
#else
    return false;
#endif
}

/// PWR_CR's FISSR and FMSSR - stopping the flash interface, or the flash
/// itself, WHILE THE SYSTEM RUNS. Absent on the F401, the F405 class and
/// the F42x/F43x; present from the F410 up and on the F446.
constexpr bool pwr_has_flash_stop_while_run() {
#if defined(PWR_CR_FISSR)
    return true;
#else
    return false;
#endif
}

/// PWR_CR.ADCDC1, the bit AN4073 owns: present everywhere but the F405
/// class.
constexpr bool pwr_has_adcdc1() {
#if defined(PWR_CR_ADCDC1)
    return true;
#else
    return false;
#endif
}

/// PWR_CSR's enable bit for wake-up pin `n` (1-based), 0 when this part
/// has no such pin. The parts with ONE pin call the bit EWUP and put it
/// at bit 8, which is exactly where the parts with several put EWUP1 - so
/// pin 1 is the same pin under both spellings, and the header's own
/// symbols answer for the rest.
constexpr uint32_t pwr_wakeup_pin_mask(uint8_t n) {
    switch (n) {
        case 1:
#if defined(PWR_CSR_EWUP1)
            return PWR_CSR_EWUP1;
#elif defined(PWR_CSR_EWUP)
            return PWR_CSR_EWUP;
#else
            return 0u;
#endif
        case 2:
#if defined(PWR_CSR_EWUP2)
            return PWR_CSR_EWUP2;
#else
            return 0u;
#endif
        case 3:
#if defined(PWR_CSR_EWUP3)
            return PWR_CSR_EWUP3;
#else
            return 0u;
#endif
        default:
            return 0u;
    }
}

/// How many WKUPx pins this part bonds: one on the F401, the F405 class,
/// the F411 and the F42x/F43x and F469/F479; two on the F446; three on
/// the F410, F412 and F413/F423.
constexpr uint8_t pwr_wakeup_pin_count() {
    uint8_t n = 0;
    while (n < 3u && pwr_wakeup_pin_mask(static_cast<uint8_t>(n + 1u)) != 0u) {
        ++n;
    }
    return n;
}

/**
 * Which PAD a wake-up pin is, AND THE DEVICE HEADER CANNOT BE ASKED - the
 * bonding is the datasheet's, so this is keyed on the device-select define
 * like the frequency ladders and the RTC's pads.
 *
 * WKUP1 is PA0 on every part read, and the datasheets label that pad
 * PA0-WKUP. The second pin of the F446 is PC13 (DS10693 table 11 names it
 * WKUP1 there, counting from zero where PWR_CSR's EWUP2 counts from one -
 * the register is what this file follows). A class whose datasheet is not
 * on the desk gets `known == false` for every pin past the first: an
 * enable written for a pad nobody identified would arm an input the
 * application cannot reason about.
 */
struct PwrWakeupPad {
    bool known = false;
    char port = 0;
    uint8_t pin = 0;
};

constexpr PwrWakeupPad pwr_wakeup_pad(uint8_t n) {
    PwrWakeupPad p{};
    if (pwr_wakeup_pin_mask(n) == 0u) {
        return p;
    }
    if (n == 1u) {
        // Every datasheet of this family labels the pad PA0-WKUP.
        p.known = true;
        p.port = 'A';
        p.pin = 0;
        return p;
    }
#if defined(STM32F446xx)
    if (n == 2u) {
        p.known = true;
        p.port = 'C';
        p.pin = 13;
    }
#endif
    return p;
}

/**
 * The eight PVD thresholds PLS[2:0] selects, in MILLIVOLTS - the
 * reference manual's table and not the header's, and it differs by part
 * class: RM0090 5.4.1 and 5.5.1 and RM0390 5.4.1 give 2.0, 2.1, 2.3, 2.5,
 * 2.6, 2.7, 2.8 and 2.9 V; RM0383 5.4.1 gives 2.2, 2.3, 2.4, 2.5, 2.6,
 * 2.7, 2.8 and 2.9 V. A class whose manual is not on the desk gets
 * `known == false` and no millivolts: the CODE is still writable there
 * (the field is the same three bits on every part), but what it means in
 * volts is refused rather than guessed.
 */
struct PvdLevels {
    bool known = false;
    uint16_t mv[8] = {};
};

constexpr PvdLevels pwr_pvd_levels() {
    PvdLevels l{};
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
    defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
    defined(STM32F446xx)
    l.known = true;
    l.mv[0] = 2000; l.mv[1] = 2100; l.mv[2] = 2300; l.mv[3] = 2500;
    l.mv[4] = 2600; l.mv[5] = 2700; l.mv[6] = 2800; l.mv[7] = 2900;
#elif defined(STM32F411xE)
    l.known = true;
    l.mv[0] = 2200; l.mv[1] = 2300; l.mv[2] = 2400; l.mv[3] = 2500;
    l.mv[4] = 2600; l.mv[5] = 2700; l.mv[6] = 2800; l.mv[7] = 2900;
#endif
    return l;
}

/// The EXTI line the PVD's output raises - a CONFIGURABLE line on this
/// family (RM0090 12.2.5, RM0390 11.2.5, RM0383 10.2.5), so a sense must
/// be chosen before anything is pending. Published here per the stratum's
/// rule that a peripheral owns its own line number.
inline constexpr uint8_t pwr_pvd_exti_line = 16;

/// RCC_AHB1ENR's gate on the 4 KB backup SRAM the backup regulator keeps
/// alive; 0 where the part has no backup SRAM (the F401, F410, F411 and
/// F412/F413 classes), whose PWR_CSR still carries BRE and BRR.
constexpr uint32_t pwr_backup_sram_clock_mask() {
#if defined(RCC_AHB1ENR_BKPSRAMEN)
    return RCC_AHB1ENR_BKPSRAMEN;
#else
    return 0u;
#endif
}

} // namespace brio
