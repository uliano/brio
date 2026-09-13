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

} // namespace brio
