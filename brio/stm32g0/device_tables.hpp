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
 *  - The FLASH's second bank: the G0B1/G0C1 headers declare the bank-2
 *    control bits, status bit, ECC register and protection registers;
 *    every smaller part declares none of them, and ECC2R is not even a
 *    member of that part's FLASH_TypeDef.
 *  - The analog block: every part has the one ADC with its nineteen
 *    channels, but the DAC is absent on the G031/G041 (16.3) and the
 *    COMPARATORS are absent there too, while the third of them is the
 *    G0B1/G0C1's alone (18.1) - and the ADC's own vector is shared with
 *    all three comparators where they exist and is the ADC's alone where
 *    they do not.
 *  - The EXTI's lines: sixteen GPIO lines everywhere, but which of the
 *    peripheral lines above them exist - and which of those are
 *    configurable rather than direct - is per part, and so is the
 *    second register group (RTSR2 and its four neighbours are members
 *    on the G0B1/G0C1 only, IMR2/EMR2 from the G071 class up).
 */

#pragma once

#include <stddef.h>
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

/// Whether USARTn carries the FULL feature set of table 183/184 - the
/// FIFOs, the PRESC prescaler, the kernel-clock multiplexer and the wake
/// from Stop, synchronous mode, smartcard, IrDA, LIN, auto-baud, the
/// receiver time-out and Modbus. A BASIC instance has none of them and
/// the rest of the chapter alike.
///
/// THIS IS A STATED TABLE AND NOT A HEADER PROBE, and the reason is that
/// the device header expresses the split as POINTER-COMPARISON macros -
/// IS_UART_FIFO_INSTANCE(x), IS_UART_AUTOBAUDRATE_DETECTION_INSTANCE(x)
/// and their kin, each of them `((x) == USART1)` and none of them a
/// constant expression. So this mirrors table 183 off the DEVICE SELECT
/// macro, exactly as usart_irq() and tim_geometry() do, and the SILICON
/// is the check: test_stm32_serial's letter a writes FIFOEN and PRESC on
/// every present instance and compares what sticks against this
/// function. (ES0548 2.11.2 is the documentation erratum saying some
/// manual revisions omit the prescaler's own per-instance split; the
/// implementation section carries it and so does this table.)
constexpr bool usart_is_full(uint8_t n) {
    if (!usart_present(n)) {
        return false;
    }
#if defined(STM32G0B1xx) || defined(STM32G0C1xx) || defined(STM32G0B0xx)
    return n <= 3;      // USART1..3 FULL, USART4..6 BASIC
#elif defined(STM32G071xx) || defined(STM32G081xx) || defined(STM32G070xx)
    return n <= 2;      // USART1..2 FULL, USART3..4 BASIC
#else
    return n == 1;      // USART1 FULL, USART2 BASIC
#endif
}

/// The EXTI line USARTn's wake-up event raises (table 65: 24, 25 and 26
/// for USART3, USART1 and USART2, all DIRECT - no trigger selection, no
/// pending bit of the EXTI's own, the peripheral's WUF being the pending
/// state). 0xFF for an instance with no wake at all - which is every
/// BASIC one, table 184's last row. The NUMBERS are the MANUAL'S, like
/// lptim_exti_line()'s and rtc_exti_line's.
constexpr uint8_t usart_exti_line(uint8_t n) {
    if (!usart_is_full(n)) {
        return 0xFF;
    }
    switch (n) {
        case 1: return 25;
        case 2: return 26;
        case 3: return 24;
        default: return 0xFF;
    }
}

// ---- LPUART instances -------------------------------------------------------
//
// A DIFFERENT PERIPHERAL sharing the USART's register layout (the header
// gives both a USART_TypeDef): chapter 34 is chapter 33 minus the
// synchronous, smartcard, IrDA, LIN, Modbus, receiver-timeout and
// auto-baud halves, minus OVER8 - and PLUS a baud generator of its own
// (256 x fck / LPUARTDIV, 34.4.7), which is why it gets its own driver
// and its own probes rather than a template argument of the USART's.

/// Register block base of LPUARTn (n = 1..2), 0 when absent. LPUART1 is
/// on every G0; LPUART2 is the G0B1/G0C1's alone.
constexpr uint32_t lpuart_base(uint8_t n) {
    switch (n) {
#if defined(LPUART1_BASE)
        case 1: return LPUART1_BASE;
#endif
#if defined(LPUART2_BASE)
        case 2: return LPUART2_BASE;
#endif
        default: return 0;
    }
}

constexpr bool lpuart_present(uint8_t n) { return lpuart_base(n) != 0u; }

/// Which APB enable register carries LPUARTn, and which bit. BOTH sit on
/// APBENR1 on this family (LPUART1 at bit 20, LPUART2 at bit 7) - the
/// header is the authority and the answer is not what the USART's
/// APB2/APB1 split invites one to guess.
constexpr UsartBusClock lpuart_bus_clock(uint8_t n) {
    switch (n) {
#if defined(RCC_APBENR1_LPUART1EN)
        case 1: return {false, RCC_APBENR1_LPUART1EN};
#endif
#if defined(RCC_APBENR1_LPUART2EN)
        case 2: return {false, RCC_APBENR1_LPUART2EN};
#endif
        default: return {};
    }
}

/// Position of LPUARTn's kernel-clock select field in RCC_CCIPR
/// (LPUART1SEL at bit 10, LPUART2SEL at bit 8), or 0xFF when absent.
/// EVERY LPUART HAS ONE - unlike the USARTs, where only the FULL
/// instances do.
constexpr uint8_t lpuart_clock_select_pos(uint8_t n) {
    switch (n) {
#if defined(RCC_CCIPR_LPUART1SEL_Pos)
        case 1: return RCC_CCIPR_LPUART1SEL_Pos;
#endif
#if defined(RCC_CCIPR_LPUART2SEL_Pos)
        case 2: return RCC_CCIPR_LPUART2SEL_Pos;
#endif
        default: return 0xFF;
    }
}

/// The NVIC line of LPUARTn. Read off the DEVICE SELECT macro for the
/// same reason usart_irq() is: IRQn_Type values are enumerators. On the
/// G0B1 class LPUART1 shares USART3..6's line and LPUART2 shares
/// USART2's; on the G071 class LPUART1 shares USART3/4's; on the G031
/// class LPUART1 has a line of its own.
constexpr IRQn_Type lpuart_irq(uint8_t n) {
#if defined(STM32G0B1xx) || defined(STM32G0C1xx) || defined(STM32G0B0xx)
    return n == 2 ? USART2_LPUART2_IRQn : USART3_4_5_6_LPUART1_IRQn;
#elif defined(STM32G071xx) || defined(STM32G081xx) || defined(STM32G070xx)
    (void)n;
    return USART3_4_LPUART1_IRQn;
#else
    (void)n;
    return LPUART1_IRQn;
#endif
}

/// The EXTI line LPUARTn's wake-up event raises (table 65: 28 for
/// LPUART1, 35 for LPUART2 - the second one in the SECOND register
/// group, which only the G0B1 class has). The manual's numbers again.
constexpr uint8_t lpuart_exti_line(uint8_t n) {
    if (!lpuart_present(n)) {
        return 0xFF;
    }
    return n == 1 ? 28 : 35;
}

// ---- IRTIM (ch. 27) ---------------------------------------------------------

/// Which USART instance SYSCFG_CFGR1.IR_MOD code 10 selects as the
/// infrared modulation envelope: USART4 on the G071/G081/G0B1/G0C1,
/// USART2 on the G031/G041/G051/G061 (ch. 27's own note, repeated in
/// 6.1.3's IR_MOD description). 0 where the part has neither - which
/// cannot happen, every G0 having at least USART2. Code 01 is USART1
/// everywhere and needs no table.
constexpr uint8_t irtim_second_usart() {
#if defined(STM32G071xx) || defined(STM32G081xx) || defined(STM32G070xx) || \
    defined(STM32G0B1xx) || defined(STM32G0C1xx) || defined(STM32G0B0xx)
    return 4;
#else
    return 2;
#endif
}

// ---- FLASH ------------------------------------------------------------------
//
// What differs across the family in chapter 3 is the SECOND BANK and the
// two protection units that come with the bigger parts. The device header
// says so three ways and all three are probed here rather than in
// flash.hpp: a feature macro (FLASH_DBANK_SUPPORT and friends), the
// presence of a bit mask (FLASH_CR_BKER exists only where a bank may be
// selected), and - the one a mask cannot express - the presence of a
// STRUCT MEMBER: FLASH_TypeDef carries ECC2R and the four bank-2
// protection registers on dual-bank parts only, so a driver naming
// FLASH->ECC2R would not compile on a G031 at all. Those are exported as
// POINTERS, null where the register does not exist, which is the value
// form of "does this exist" and keeps the driver free of #ifdef.

/// Dual-bank capability: the 512 KB parts always run in two banks, the
/// 256 KB ones can, and no smaller part has a second bank at all.
constexpr bool flash_dual_bank_capable =
#if defined(FLASH_DBANK_SUPPORT)
    true;
#else
    false;
#endif

/// Proprietary code readout protection areas (PCROP), read-only here.
constexpr bool flash_pcrop_capable =
#if defined(FLASH_PCROP_SUPPORT)
    true;
#else
    false;
#endif

/// The securable memory area (FLASH_SECR.SEC_SIZE + FLASH_CR.SEC_PROT).
constexpr bool flash_securable_capable =
#if defined(FLASH_SECURABLE_MEMORY_SUPPORT)
    true;
#else
    false;
#endif

/// FLASH_CR.BKER - which physical bank an erase acts on. Absent (0) on
/// single-bank parts, where 3.7.5 says the bit "has no effect".
constexpr uint32_t flash_cr_bank_select =
#if defined(FLASH_CR_BKER)
    FLASH_CR_BKER;
#else
    0u;
#endif

/// FLASH_CR.MER2 - mass erase of bank 2.
constexpr uint32_t flash_cr_mass_erase2 =
#if defined(FLASH_CR_MER2)
    FLASH_CR_MER2;
#else
    0u;
#endif

/// FLASH_SR.BSY2 - bank 2 busy.
constexpr uint32_t flash_sr_bank2_busy =
#if defined(FLASH_SR_BSY2)
    FLASH_SR_BSY2;
#else
    0u;
#endif

/// FLASH_OPTR.DUAL_BANK - the option bit that puts a 256 KB part in two
/// banks (and reads as configured, but without effect, on a 512 KB one).
constexpr uint32_t flash_optr_dual_bank =
#if defined(FLASH_OPTR_DUAL_BANK)
    FLASH_OPTR_DUAL_BANK;
#else
    0u;
#endif

/// FLASH_OPTR.nSWAP_BANK - 1 = no swap (physical bank 1 at 0x08000000).
constexpr uint32_t flash_optr_swap_bank =
#if defined(FLASH_OPTR_nSWAP_BANK)
    FLASH_OPTR_nSWAP_BANK;
#else
    0u;
#endif

/// FLASH_SECR.SEC_SIZE2 - the securable area of bank 2.
constexpr uint32_t flash_secr_sec_size2 =
#if defined(FLASH_SECR_SEC_SIZE2)
    FLASH_SECR_SEC_SIZE2;
#else
    0u;
#endif

/// FLASH_ECC2R, the bank-2 ECC register - a STRUCT MEMBER that only the
/// dual-bank headers declare. Null where there is no second bank.
inline volatile uint32_t* flash_ecc2r() {
#if defined(FLASH_ECC2R_ECCC)
    return &FLASH->ECC2R;
#else
    return nullptr;
#endif
}

/// The four bank-2 protection registers, same story as ECC2R.
inline volatile uint32_t* flash_wrp2ar() {
#if defined(FLASH_WRP2AR_WRP2A_STRT)
    return &FLASH->WRP2AR;
#else
    return nullptr;
#endif
}
inline volatile uint32_t* flash_wrp2br() {
#if defined(FLASH_WRP2BR_WRP2B_STRT)
    return &FLASH->WRP2BR;
#else
    return nullptr;
#endif
}
inline volatile uint32_t* flash_pcrop2a_start() {
#if defined(FLASH_PCROP2ASR_PCROP2A_STRT)
    return &FLASH->PCROP2ASR;
#else
    return nullptr;
#endif
}
inline volatile uint32_t* flash_pcrop2a_end() {
#if defined(FLASH_PCROP2AER_PCROP2A_END)
    return &FLASH->PCROP2AER;
#else
    return nullptr;
#endif
}
inline volatile uint32_t* flash_pcrop2b_start() {
#if defined(FLASH_PCROP2BSR_PCROP2B_STRT)
    return &FLASH->PCROP2BSR;
#else
    return nullptr;
#endif
}
inline volatile uint32_t* flash_pcrop2b_end() {
#if defined(FLASH_PCROP2BER_PCROP2B_END)
    return &FLASH->PCROP2BER;
#else
    return nullptr;
#endif
}

// ---- EXTI (RM0444 ch. 13) ---------------------------------------------------
//
// WHAT DIFFERS ACROSS THE FAMILY here is WHICH LINES EXIST and WHICH OF
// THEM ARE CONFIGURABLE - and the device header states both, so nothing
// below is a copied table:
//  - EXTI_IMR1_IM_Msk is a PER-VARIANT mask and not a blanket one
//    (0xF2A9FFFF on the G031, 0xFEAFFFFF on the G071, 0xFFFFFFFF on the
//    G0B1), which makes it the authority on which of lines 0..31 the
//    part implements at all; EXTI_IMR2_IM_Msk is its twin for lines
//    32..36 and does not exist where there is no second group.
//  - a CONFIGURABLE line (13.3, table 64) is one with a trigger
//    selection bit, so the RTSR masks are what say which lines those
//    are; a DIRECT line has no RTSR/FTSR/SWIER/RPR/FPR bit at all and
//    is enabled in the peripheral that owns it.
//  - the second register group is a STRUCT MEMBER question, the
//    FLASH_ECC2R situation again: RTSR2/FTSR2/SWIER2/RPR2/FPR2 are
//    members on the G0B1/G0C1 only, IMR2/EMR2 from the G071 class up,
//    and a driver naming EXTI->IMR2 would not compile on a G031. They
//    are exported as POINTERS, null where the register does not exist.
// The GPIO half is uniform: EXTICR is four registers of four 8-bit
// fields on every part, so there are always sixteen GPIO lines, and
// they are always lines 0..15.

/// Sixteen, read off the header's own EXTICR array rather than from
/// 13.3.3's prose: four registers, four port-selection fields each.
constexpr uint8_t exti_gpio_lines =
    static_cast<uint8_t>(sizeof(EXTI_TypeDef::EXTICR) / sizeof(uint32_t) * 4u);

/// Which of lines 0..31 this device implements (EXTI_IMR1_IM_Msk).
constexpr uint32_t exti_implemented_mask1 = EXTI_IMR1_IM_Msk;

/// Which of lines 32..63 it implements, bit 0 = line 32. Zero where the
/// part has no second group at all.
constexpr uint32_t exti_implemented_mask2 =
#if defined(EXTI_IMR2_IM_Msk)
    EXTI_IMR2_IM_Msk;
#else
    0u;
#endif

/// Which of lines 0..31 are CONFIGURABLE (they have a rising-trigger
/// bit); the rest are direct lines, or nothing at all.
constexpr uint32_t exti_configurable_mask1 = 0u
#if defined(EXTI_RTSR1_RT0_Msk)
    | EXTI_RTSR1_RT0_Msk
#endif
#if defined(EXTI_RTSR1_RT1_Msk)
    | EXTI_RTSR1_RT1_Msk
#endif
#if defined(EXTI_RTSR1_RT2_Msk)
    | EXTI_RTSR1_RT2_Msk
#endif
#if defined(EXTI_RTSR1_RT3_Msk)
    | EXTI_RTSR1_RT3_Msk
#endif
#if defined(EXTI_RTSR1_RT4_Msk)
    | EXTI_RTSR1_RT4_Msk
#endif
#if defined(EXTI_RTSR1_RT5_Msk)
    | EXTI_RTSR1_RT5_Msk
#endif
#if defined(EXTI_RTSR1_RT6_Msk)
    | EXTI_RTSR1_RT6_Msk
#endif
#if defined(EXTI_RTSR1_RT7_Msk)
    | EXTI_RTSR1_RT7_Msk
#endif
#if defined(EXTI_RTSR1_RT8_Msk)
    | EXTI_RTSR1_RT8_Msk
#endif
#if defined(EXTI_RTSR1_RT9_Msk)
    | EXTI_RTSR1_RT9_Msk
#endif
#if defined(EXTI_RTSR1_RT10_Msk)
    | EXTI_RTSR1_RT10_Msk
#endif
#if defined(EXTI_RTSR1_RT11_Msk)
    | EXTI_RTSR1_RT11_Msk
#endif
#if defined(EXTI_RTSR1_RT12_Msk)
    | EXTI_RTSR1_RT12_Msk
#endif
#if defined(EXTI_RTSR1_RT13_Msk)
    | EXTI_RTSR1_RT13_Msk
#endif
#if defined(EXTI_RTSR1_RT14_Msk)
    | EXTI_RTSR1_RT14_Msk
#endif
#if defined(EXTI_RTSR1_RT15_Msk)
    | EXTI_RTSR1_RT15_Msk
#endif
#if defined(EXTI_RTSR1_RT16_Msk)
    | EXTI_RTSR1_RT16_Msk
#endif
#if defined(EXTI_RTSR1_RT17_Msk)
    | EXTI_RTSR1_RT17_Msk
#endif
#if defined(EXTI_RTSR1_RT18_Msk)
    | EXTI_RTSR1_RT18_Msk
#endif
#if defined(EXTI_RTSR1_RT19_Msk)
    | EXTI_RTSR1_RT19_Msk
#endif
#if defined(EXTI_RTSR1_RT20_Msk)
    | EXTI_RTSR1_RT20_Msk
#endif
#if defined(EXTI_RTSR1_RT21_Msk)
    | EXTI_RTSR1_RT21_Msk
#endif
#if defined(EXTI_RTSR1_RT22_Msk)
    | EXTI_RTSR1_RT22_Msk
#endif
#if defined(EXTI_RTSR1_RT23_Msk)
    | EXTI_RTSR1_RT23_Msk
#endif
#if defined(EXTI_RTSR1_RT24_Msk)
    | EXTI_RTSR1_RT24_Msk
#endif
#if defined(EXTI_RTSR1_RT25_Msk)
    | EXTI_RTSR1_RT25_Msk
#endif
#if defined(EXTI_RTSR1_RT26_Msk)
    | EXTI_RTSR1_RT26_Msk
#endif
#if defined(EXTI_RTSR1_RT27_Msk)
    | EXTI_RTSR1_RT27_Msk
#endif
#if defined(EXTI_RTSR1_RT28_Msk)
    | EXTI_RTSR1_RT28_Msk
#endif
#if defined(EXTI_RTSR1_RT29_Msk)
    | EXTI_RTSR1_RT29_Msk
#endif
#if defined(EXTI_RTSR1_RT30_Msk)
    | EXTI_RTSR1_RT30_Msk
#endif
#if defined(EXTI_RTSR1_RT31_Msk)
    | EXTI_RTSR1_RT31_Msk
#endif
    ;

/// The same for lines 32..63, bit 0 = line 32.
constexpr uint32_t exti_configurable_mask2 = 0u
#if defined(EXTI_RTSR2_RT32_Msk)
    | EXTI_RTSR2_RT32_Msk
#endif
#if defined(EXTI_RTSR2_RT33_Msk)
    | EXTI_RTSR2_RT33_Msk
#endif
#if defined(EXTI_RTSR2_RT34_Msk)
    | EXTI_RTSR2_RT34_Msk
#endif
#if defined(EXTI_RTSR2_RT35_Msk)
    | EXTI_RTSR2_RT35_Msk
#endif
#if defined(EXTI_RTSR2_RT36_Msk)
    | EXTI_RTSR2_RT36_Msk
#endif
    ;

constexpr bool exti_line_implemented(uint8_t line) {
    if (line < 32u) {
        return (exti_implemented_mask1 & (1u << line)) != 0u;
    }
    return line < 64u &&
           (exti_implemented_mask2 & (1u << (line - 32u))) != 0u;
}

constexpr bool exti_line_configurable(uint8_t line) {
    if (line < 32u) {
        return (exti_configurable_mask1 & (1u << line)) != 0u;
    }
    return line < 64u &&
           (exti_configurable_mask2 & (1u << (line - 32u))) != 0u;
}

/// The EXTICR code that selects port `letter` as the source of a GPIO
/// line: 0 for A up to 5 for F (13.5.11), and 0xFF for a port this
/// device does not bond - the presence question is the header's
/// (gpio_port_base), the ENCODING is the manual's, there being no
/// per-port enumerator in the header to read it off.
constexpr uint8_t exti_port_code(char letter) {
    if (!gpio_port_present(letter) || letter < 'A' || letter > 'F') {
        return 0xFFu;
    }
    return static_cast<uint8_t>(letter - 'A');
}

/// The second register group as pointers, null where the register is
/// not a member of this part's EXTI_TypeDef (see the note above).
inline volatile uint32_t* exti_rtsr2() {
#if defined(EXTI_RTSR2_RT34_Msk)
    return &EXTI->RTSR2;
#else
    return nullptr;
#endif
}
inline volatile uint32_t* exti_ftsr2() {
#if defined(EXTI_FTSR2_FT34_Msk)
    return &EXTI->FTSR2;
#else
    return nullptr;
#endif
}
inline volatile uint32_t* exti_swier2() {
#if defined(EXTI_SWIER2_SWI34_Msk)
    return &EXTI->SWIER2;
#else
    return nullptr;
#endif
}
inline volatile uint32_t* exti_rpr2() {
#if defined(EXTI_RPR2_RPIF34_Msk)
    return &EXTI->RPR2;
#else
    return nullptr;
#endif
}
inline volatile uint32_t* exti_fpr2() {
#if defined(EXTI_FPR2_FPIF34_Msk)
    return &EXTI->FPR2;
#else
    return nullptr;
#endif
}
inline volatile uint32_t* exti_imr2() {
#if defined(EXTI_IMR2_IM_Msk)
    return &EXTI->IMR2;
#else
    return nullptr;
#endif
}
inline volatile uint32_t* exti_emr2() {
#if defined(EXTI_EMR2_EM_Msk)
    return &EXTI->EMR2;
#else
    return nullptr;
#endif
}

/// The NVIC line a GPIO EXTI line interrupts on. Three vectors serve
/// the sixteen (table 61: EXTI0_1, EXTI2_3, EXTI4_15), and all three
/// exist on every part of the family - but IRQn values are ENUMERATORS
/// and the preprocessor cannot probe them, so this lives here with
/// usart_irq() and not in a driver. Lines ABOVE 15 have no EXTI vector
/// of their own at all: a direct line interrupts through the vector of
/// the peripheral that owns it, and the configurable non-GPIO lines
/// share one too (PVD_VDDIO2 for 16 and 34, ADC1_COMP for 17/18/20) -
/// which is why this verb is spelled `gpio`.
constexpr IRQn_Type exti_gpio_irq(uint8_t line) {
    if (line < 2u) {
        return EXTI0_1_IRQn;
    }
    if (line < 4u) {
        return EXTI2_3_IRQn;
    }
    return EXTI4_15_IRQn;
}

/// The lines one of those three vectors serves, as a mask - what an ISR
/// body is handed so that a handler answers for its own lines only.
constexpr uint32_t exti_vector_lines(IRQn_Type irq) {
    if (irq == EXTI0_1_IRQn) {
        return 0x0003u;
    }
    if (irq == EXTI2_3_IRQn) {
        return 0x000Cu;
    }
    if (irq == EXTI4_15_IRQn) {
        return 0xFFF0u;
    }
    return 0u;
}

// ---- TIM instances (RM0444 ch. 21..25) --------------------------------------
//
// WHAT THE HEADER CAN ANSWER AND WHAT IT CANNOT, stated here because this
// chapter is the first in the stratum where the two halves are really
// different:
//  - WHICH TIMERS EXIST is the header's, three ways over: TIMn_BASE, the
//    RCC enable/reset masks, and the IRQn enumerators. The G0B1/G0C1 has
//    TIM1, 2, 3, 4, 6, 7, 14, 15, 16, 17; the G071 class has all of those
//    but TIM4; the G031 class has 1, 2, 3, 14, 16, 17 and neither basic
//    timer nor TIM15 - every one of those statements is a probe below and
//    not a list.
//  - WHAT A TIMER IS - counter width, how many channels, whether it has a
//    slave controller, a break/dead-time unit, a repetition counter,
//    centre-aligned counting - IS NOT IN THE DEVICE HEADER AT ALL. There
//    is ONE TIM_TypeDef for every instance (every register is a member on
//    every timer), so a driver naming TIM14->SMCR compiles and writes a
//    hole. Those facts are the DOCUMENTS' (DS13560 table 7, RM0444 21.2 /
//    22.2 / 23.2 / 24.2 / 25.2) and they are spelled out below as what
//    they are: a small table keyed by instance number, uniform across the
//    whole STM32G0 family, and reached only for an instance the HEADER
//    says exists. That is the honest shape - the alternative is a driver
//    that silently writes registers the silicon does not implement.
//  - THE PAD MAP IS THE DATASHEET'S TOO (DS13560 tables 13..24) and is
//    not here for the reason stm32g0/pin.hpp gives once for the whole
//    stratum: no symbol of this device header carries it, so a driver's
//    AF claim can be checked at the bench and nowhere else. A timer pad
//    is therefore named by the caller as a PinSel, never derived.

/// Register block base of TIMn, 0 when the device does not have it.
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
#if defined(TIM6_BASE)
        case 6: return TIM6_BASE;
#endif
#if defined(TIM7_BASE)
        case 7: return TIM7_BASE;
#endif
#if defined(TIM14_BASE)
        case 14: return TIM14_BASE;
#endif
#if defined(TIM15_BASE)
        case 15: return TIM15_BASE;
#endif
#if defined(TIM16_BASE)
        case 16: return TIM16_BASE;
#endif
#if defined(TIM17_BASE)
        case 17: return TIM17_BASE;
#endif
        default: return 0;
    }
}

constexpr bool tim_present(uint8_t n) { return tim_base(n) != 0u; }

/// Which APB enable/reset register carries TIMn, and which bit of each.
/// TIM1 and TIM14..TIM17 are the APB2 instances of this family; TIM2, 3,
/// 4, 6 and 7 sit on APB1. `mask` 0 means "no such instance".
struct TimBusClock {
    bool apb2 = false;
    uint32_t enable_mask = 0;
    uint32_t reset_mask = 0;
};

constexpr TimBusClock tim_bus_clock(uint8_t n) {
    switch (n) {
#if defined(RCC_APBENR2_TIM1EN)
        case 1: return {true, RCC_APBENR2_TIM1EN, RCC_APBRSTR2_TIM1RST};
#endif
#if defined(RCC_APBENR1_TIM2EN)
        case 2: return {false, RCC_APBENR1_TIM2EN, RCC_APBRSTR1_TIM2RST};
#endif
#if defined(RCC_APBENR1_TIM3EN)
        case 3: return {false, RCC_APBENR1_TIM3EN, RCC_APBRSTR1_TIM3RST};
#endif
#if defined(RCC_APBENR1_TIM4EN)
        case 4: return {false, RCC_APBENR1_TIM4EN, RCC_APBRSTR1_TIM4RST};
#endif
#if defined(RCC_APBENR1_TIM6EN)
        case 6: return {false, RCC_APBENR1_TIM6EN, RCC_APBRSTR1_TIM6RST};
#endif
#if defined(RCC_APBENR1_TIM7EN)
        case 7: return {false, RCC_APBENR1_TIM7EN, RCC_APBRSTR1_TIM7RST};
#endif
#if defined(RCC_APBENR2_TIM14EN)
        case 14: return {true, RCC_APBENR2_TIM14EN, RCC_APBRSTR2_TIM14RST};
#endif
#if defined(RCC_APBENR2_TIM15EN)
        case 15: return {true, RCC_APBENR2_TIM15EN, RCC_APBRSTR2_TIM15RST};
#endif
#if defined(RCC_APBENR2_TIM16EN)
        case 16: return {true, RCC_APBENR2_TIM16EN, RCC_APBRSTR2_TIM16RST};
#endif
#if defined(RCC_APBENR2_TIM17EN)
        case 17: return {true, RCC_APBENR2_TIM17EN, RCC_APBRSTR2_TIM17RST};
#endif
        default: return {};
    }
}

/// Counter width in BITS (DS13560 table 7): TIM2 is the family's one
/// 32-bit counter, every other timer is 16-bit. 0 for an instance this
/// device does not have.
constexpr uint8_t tim_counter_bits(uint8_t n) {
    if (!tim_present(n)) {
        return 0;
    }
    return n == 2u ? 32u : 16u;
}

/// The largest ARR/CNT/CCR value the counter holds.
constexpr uint32_t tim_max_period(uint8_t n) {
    return tim_counter_bits(n) == 32u ? 0xFFFFFFFFUL
                                      : (tim_counter_bits(n) == 16u ? 0xFFFFUL : 0UL);
}

/// Capture/compare channels (DS13560 table 7). The basic timers have
/// none at all - they are a time base and a TRGO, which is why the
/// driver's channel verbs refuse on them rather than writing a CCMR
/// that is not implemented.
constexpr uint8_t tim_channels(uint8_t n) {
    switch (n) {
        case 1: case 2: case 3: case 4: return tim_present(n) ? 4u : 0u;
        case 15: return tim_present(n) ? 2u : 0u;
        case 14: case 16: case 17: return tim_present(n) ? 1u : 0u;
        default: return 0u;   // TIM6, TIM7 and anything absent
    }
}

/// COMPLEMENTARY outputs (CCxN): three on TIM1 (channels 1..3), one on
/// TIM15/16/17 (channel 1). A pair needs the break/dead-time unit, which
/// is exactly the set of timers that have one.
constexpr uint8_t tim_complementary_channels(uint8_t n) {
    switch (n) {
        case 1: return tim_present(n) ? 3u : 0u;
        case 15: case 16: case 17: return tim_present(n) ? 1u : 0u;
        default: return 0u;
    }
}

/// TIMx_SMCR - the slave controller (external clock mode 1, reset,
/// gated, trigger and the combined modes, and the ITRx multiplexer).
/// TIM14, TIM16 and TIM17 have no SMCR at all (24.4, 25.6), the basic
/// timers have none either (23.4).
constexpr bool tim_has_slave_mode(uint8_t n) {
    switch (n) {
        case 1: case 2: case 3: case 4: case 15: return tim_present(n);
        default: return false;
    }
}

/// TIMx_CR2.MMS - the master mode that drives TRGO. The basic timers
/// have it (that is what they are FOR: 23.4.2's MMS feeds the DAC and
/// the other timers); TIM14, TIM16 and TIM17 have no CR2.MMS - 25.4.24
/// says so in words and offers OC1 as the trigger instead.
constexpr bool tim_has_master_mode(uint8_t n) {
    switch (n) {
        case 1: case 2: case 3: case 4: case 6: case 7: case 15:
            return tim_present(n);
        default: return false;
    }
}

/// TIMx_BDTR - break inputs, the dead-time generator, MOE and the off
/// states. Exactly the timers with a complementary output (21.4.18,
/// 25.5.16, 25.6.14).
constexpr bool tim_has_break(uint8_t n) { return tim_complementary_channels(n) != 0u; }

/// TIMx_RCR - the repetition counter (an update event every RCR + 1
/// periods). The same four timers again.
constexpr bool tim_has_repetition(uint8_t n) { return tim_has_break(n); }

/// CR1.DIR and CR1.CMS: up/down and centre-aligned counting. DS13560
/// table 7's "counter type" column - only TIM1..TIM4 count anything but
/// up.
constexpr bool tim_has_direction(uint8_t n) {
    switch (n) {
        case 1: case 2: case 3: case 4: return tim_present(n);
        default: return false;
    }
}
constexpr bool tim_has_center_aligned(uint8_t n) { return tim_has_direction(n); }

/// TIMx_TISEL, the input multiplexer that makes a capture channel
/// reachable with no pad at all (an internal clock, a comparator). Every
/// timer with a channel has one; the basic timers have no input.
constexpr bool tim_has_tisel(uint8_t n) { return tim_channels(n) != 0u; }

/// TIMx_DCR/TIMx_DMAR, the DMA burst engine (DS13560 table 7's "DMA
/// request generation"): everything but TIM14.
constexpr bool tim_has_dma_burst(uint8_t n) {
    switch (n) {
        case 1: case 2: case 3: case 4: case 6: case 7: case 15: case 16: case 17:
            return tim_present(n);
        default: return false;
    }
}

/// TIMx_ETR, the external trigger input (SMCR's ECE/ETP/ETPS/ETF half).
/// The timers with a slave controller have it; TIM15 does not (25.5.3
/// gives it ITRx and the TI inputs and no ETR).
constexpr bool tim_has_external_trigger(uint8_t n) {
    switch (n) {
        case 1: case 2: case 3: case 4: return tim_present(n);
        default: return false;
    }
}

/// The NVIC line TIMn's UPDATE, capture/compare, trigger and break
/// events reach. SHARED LINES ARE THE RULE HERE (table 61): TIM3 shares
/// with TIM4 on the G0B1 class, TIM6 with the DAC and LPTIM1, TIM7 with
/// LPTIM2, TIM16 and TIM17 with the two FDCAN interrupt lines - and TIM1
/// alone has TWO vectors, the capture/compare one being separate (see
/// tim_cc_irq). IRQn values are enumerators the preprocessor cannot
/// probe, so this reads the DEVICE SELECT macro, exactly as usart_irq()
/// does; a wrong line here is a silent Default_Handler spin, which is
/// what the family fixture instantiating every present instance on every
/// header is for.
constexpr IRQn_Type tim_irq(uint8_t n) {
#if defined(STM32G0B1xx) || defined(STM32G0C1xx) || defined(STM32G0B0xx)
    switch (n) {
        case 1: return TIM1_BRK_UP_TRG_COM_IRQn;
        case 2: return TIM2_IRQn;
        case 3: case 4: return TIM3_TIM4_IRQn;
        case 6: return TIM6_DAC_LPTIM1_IRQn;
        case 7: return TIM7_LPTIM2_IRQn;
        case 14: return TIM14_IRQn;
        case 15: return TIM15_IRQn;
        case 16: return TIM16_FDCAN_IT0_IRQn;
        default: return TIM17_FDCAN_IT1_IRQn;
    }
#elif defined(STM32G071xx) || defined(STM32G081xx) || defined(STM32G070xx)
    switch (n) {
        case 1: return TIM1_BRK_UP_TRG_COM_IRQn;
        case 2: return TIM2_IRQn;
        case 3: return TIM3_IRQn;
        case 6: return TIM6_DAC_LPTIM1_IRQn;
        case 7: return TIM7_LPTIM2_IRQn;
        case 14: return TIM14_IRQn;
        case 15: return TIM15_IRQn;
        case 16: return TIM16_IRQn;
        default: return TIM17_IRQn;
    }
#else
    // G031/G041/G030/G051/G061/G050: TIM1, TIM2, TIM3, TIM14, TIM16,
    // TIM17, each on a line of its own (LPTIM1/LPTIM2 take positions 17
    // and 18, where the bigger parts put TIM6 and TIM7).
    switch (n) {
        case 1: return TIM1_BRK_UP_TRG_COM_IRQn;
        case 2: return TIM2_IRQn;
        case 3: return TIM3_IRQn;
        case 14: return TIM14_IRQn;
        case 16: return TIM16_IRQn;
        default: return TIM17_IRQn;
    }
#endif
}

/// TIM1's SECOND vector, the capture/compare one (table 61 position 14).
/// Every other timer of this family reports everything on one line, so
/// this equals tim_irq(n) there - which is what lets a handler bind both
/// without asking whether they are two.
constexpr IRQn_Type tim_cc_irq(uint8_t n) {
    return n == 1u ? TIM1_CC_IRQn : tim_irq(n);
}

/// Do TIMn's update/trigger/break and its capture/compare events reach
/// two DIFFERENT vectors? True for TIM1 alone.
constexpr bool tim_has_split_vector(uint8_t n) { return n == 1u && tim_present(n); }

// ---- LPTIM instances (RM0444 ch. 26) ----------------------------------------

/// Register block base of LPTIMn (n = 1, 2), 0 when the device does not
/// have it. Every STM32G0x1 of this pack carries both, but the probe is
/// still the header's and not a claim of this file's.
constexpr uint32_t lptim_base(uint8_t n) {
    switch (n) {
#if defined(LPTIM1_BASE)
        case 1: return LPTIM1_BASE;
#endif
#if defined(LPTIM2_BASE)
        case 2: return LPTIM2_BASE;
#endif
        default: return 0;
    }
}

constexpr bool lptim_present(uint8_t n) { return lptim_base(n) != 0u; }

/// RCC_APBENR1 bit that clocks LPTIMn. Both instances are APB1's on
/// this family (5.4.19), so there is no bus question to answer here as
/// there is for the timers and the USARTs.
constexpr uint32_t lptim_bus_clock_mask(uint8_t n) {
    switch (n) {
#if defined(RCC_APBENR1_LPTIM1EN)
        case 1: return RCC_APBENR1_LPTIM1EN;
#endif
#if defined(RCC_APBENR1_LPTIM2EN)
        case 2: return RCC_APBENR1_LPTIM2EN;
#endif
        default: return 0;
    }
}

/// RCC_APBRSTR1 bit that resets LPTIMn. This one is not a convenience:
/// ES0548 2.8.1's workaround makes the RCC reset THE ONLY WAY the driver
/// turns an LPTIM off, so a missing bit here would be a driver with no
/// disable at all - which is why it is probed and not assumed.
constexpr uint32_t lptim_reset_mask(uint8_t n) {
    switch (n) {
#if defined(RCC_APBRSTR1_LPTIM1RST)
        case 1: return RCC_APBRSTR1_LPTIM1RST;
#endif
#if defined(RCC_APBRSTR1_LPTIM2RST)
        case 2: return RCC_APBRSTR1_LPTIM2RST;
#endif
        default: return 0;
    }
}

/// Position of LPTIMn's kernel-clock select field in RCC_CCIPR (5.4.21),
/// or 0xFF when the instance has no multiplexer. BOTH instances have one
/// on every part of this pack - unlike the USARTs, where the field
/// exists for some instances only - and the codes are the same four for
/// both: 00 PCLK, 01 LSI, 10 HSI16, 11 LSE.
constexpr uint8_t lptim_clock_select_pos(uint8_t n) {
    switch (n) {
#if defined(RCC_CCIPR_LPTIM1SEL_Pos)
        case 1: return RCC_CCIPR_LPTIM1SEL_Pos;
#endif
#if defined(RCC_CCIPR_LPTIM2SEL_Pos)
        case 2: return RCC_CCIPR_LPTIM2SEL_Pos;
#endif
        default: return 0xFF;
    }
}

/// The NVIC line of LPTIMn, and it is the reason this probe exists at
/// all: on the G0B1/G0C1 and the G071 class each LPTIM SHARES a vector
/// (LPTIM1 with TIM6 and the DAC, LPTIM2 with TIM7 - table 61), while on
/// the G031 class, which has neither TIM6 nor TIM7 nor a DAC, each has a
/// line of its own under a different enumerator name. IRQn values are
/// enumerators the preprocessor cannot probe, so this reads the DEVICE
/// SELECT macro exactly as usart_irq() and tim_irq() do.
/// The presence gate is not decoration: the value lines (G070, G0B0,
/// G030, G050) have no LPTIM AT ALL, so neither enumerator exists there
/// and an ungated body would fail to compile on a header this stratum is
/// otherwise happy with. Those parts answer NonMaskableInt_IRQn, which
/// no caller can reach - Lptim<n> static_asserts on lptim_present(n).
constexpr IRQn_Type lptim_irq(uint8_t n) {
#if defined(LPTIM1_BASE)
#if defined(STM32G0B1xx) || defined(STM32G0C1xx) || defined(STM32G071xx) || \
    defined(STM32G081xx)
    return n == 1u ? TIM6_DAC_LPTIM1_IRQn : TIM7_LPTIM2_IRQn;
#else
    return n == 1u ? LPTIM1_IRQn : LPTIM2_IRQn;
#endif
#else
    (void)n;
    return NonMaskableInt_IRQn;
#endif
}

/// The EXTI line LPTIMn's wake-up event raises (table 65: 29 and 30,
/// both DIRECT - no trigger selection, no pending bit of the EXTI's own,
/// the peripheral's flag being the pending state). The NUMBERS are the
/// MANUAL'S: no header of this pack spells them, which is why they live
/// here beside comp_exti_line() and rtc_exti_line and not in a driver.
/// 0xFF for an instance this device has not got.
constexpr uint8_t lptim_exti_line(uint8_t n) {
    if (!lptim_present(n)) {
        return 0xFF;
    }
    switch (n) {
        case 1: return 29;
        case 2: return 30;
        default: return 0xFF;
    }
}

/// Encoder mode: LPTIM1's alone (table 135 - "the full set of features is
/// implemented in LPTIM1", and the one row of that table is the encoder).
///
/// THIS HALF IS THE MANUAL'S AND NOT THE HEADER'S, and the distinction
/// matters: the device header declares LPTIM_CFGR_ENC, LPTIM_ISR_UP and
/// their neighbours ONCE for the one LPTIM_TypeDef both instances share,
/// so `LPTIM2->CFGR |= LPTIM_CFGR_ENC` compiles and writes a bit 26.7.4
/// marks Reserved on that instance. There is no symbol to read the
/// difference off, so it is stated here with its citation - the tim.hpp
/// geometry precedent, where DS13560 table 7 plays the same role.
constexpr bool lptim_has_encoder(uint8_t n) {
    return lptim_present(n) && n == 1u;
}

/// A SECOND input channel: LPTIM1's alone. Figure 271's own footnote -
/// "LPTIM2 has only the input channel 1, no input channel 2" - and
/// 26.7.9's note makes CFGR2's IN2SEL field Reserved where encoder mode
/// is absent. Same authority and same caveat as lptim_has_encoder():
/// stated from the manual, because the shared struct says nothing.
constexpr bool lptim_has_input2(uint8_t n) {
    return lptim_present(n) && n == 1u;
}

/// The DMAMUX TRIGGER input the LPTIM's output drives (table 56: 20 for
/// LPTIM1_OUT, 21 for LPTIM2_OUT). A trigger input, NOT a request line -
/// an LPTIM has no DMA request of its own on this family - so what it
/// feeds is a request GENERATOR, which is how a DMA channel counts an
/// LPTIM's output edges with no CPU in the loop. 0xFF past the count.
constexpr uint8_t lptim_dmamux_trigger(uint8_t n) {
    if (!lptim_present(n)) {
        return 0xFF;
    }
    switch (n) {
        case 1: return 20;
        case 2: return 21;
        default: return 0xFF;
    }
}

// ---- CRC (RM0444 ch. 14) ----------------------------------------------------

/// Register block base of the CRC calculation unit, 0 where the device
/// has none. Every STM32G0 carries exactly one, so this is a presence
/// probe like adc_base() and not an instance table.
constexpr uint32_t crc_base() {
#if defined(CRC_BASE)
    return CRC_BASE;
#else
    return 0;
#endif
}

constexpr bool crc_present() { return crc_base() != 0u; }

/// RCC_AHBENR bit that clocks it - the CRC is an AHB peripheral, which
/// is what makes its computation cost whole HCLK cycles rather than APB
/// ones (14.3.3).
constexpr uint32_t crc_clock_mask() {
#if defined(RCC_AHBENR_CRCEN)
    return RCC_AHBENR_CRCEN;
#else
    return 0;
#endif
}

constexpr uint32_t crc_reset_mask() {
#if defined(RCC_AHBRSTR_CRCRST)
    return RCC_AHBRSTR_CRCRST;
#else
    return 0;
#endif
}

// ---- DMA and DMAMUX (RM0444 ch. 10, 11) --------------------------------------

/// Register block base of DMAn (n = 1, 2), 0 when the device does not
/// have it. Only the G0B1/G0C1 class carries a second controller.
constexpr uint32_t dma_base(uint8_t n) {
    switch (n) {
#if defined(DMA1_BASE)
        case 1: return DMA1_BASE;
#endif
#if defined(DMA2_BASE)
        case 2: return DMA2_BASE;
#endif
        default: return 0u;
    }
}

constexpr bool dma_present(uint8_t n) { return dma_base(n) != 0u; }

/// Register block base of channel `ch` (1-based, as the silicon and the
/// manual number them) of DMAn; 0 when that channel does not exist. The
/// header spells one macro per channel, so each is probed by name - and
/// the probe is what says the G031 class has five channels where the
/// G071 class has seven.
constexpr uint32_t dma_channel_base(uint8_t n, uint8_t ch) {
    if (n == 1u) {
        switch (ch) {
#if defined(DMA1_Channel1_BASE)
            case 1: return DMA1_Channel1_BASE;
#endif
#if defined(DMA1_Channel2_BASE)
            case 2: return DMA1_Channel2_BASE;
#endif
#if defined(DMA1_Channel3_BASE)
            case 3: return DMA1_Channel3_BASE;
#endif
#if defined(DMA1_Channel4_BASE)
            case 4: return DMA1_Channel4_BASE;
#endif
#if defined(DMA1_Channel5_BASE)
            case 5: return DMA1_Channel5_BASE;
#endif
#if defined(DMA1_Channel6_BASE)
            case 6: return DMA1_Channel6_BASE;
#endif
#if defined(DMA1_Channel7_BASE)
            case 7: return DMA1_Channel7_BASE;
#endif
            default: return 0u;
        }
    }
    if (n == 2u) {
        switch (ch) {
#if defined(DMA2_Channel1_BASE)
            case 1: return DMA2_Channel1_BASE;
#endif
#if defined(DMA2_Channel2_BASE)
            case 2: return DMA2_Channel2_BASE;
#endif
#if defined(DMA2_Channel3_BASE)
            case 3: return DMA2_Channel3_BASE;
#endif
#if defined(DMA2_Channel4_BASE)
            case 4: return DMA2_Channel4_BASE;
#endif
#if defined(DMA2_Channel5_BASE)
            case 5: return DMA2_Channel5_BASE;
#endif
            default: return 0u;
        }
    }
    return 0u;
}

constexpr bool dma_channel_present(uint8_t n, uint8_t ch) {
    return dma_channel_base(n, ch) != 0u;
}

/// How many channels DMAn has, counted by the probes above (7 and 5 on
/// the G0B1 class, 7 and none on the G071 class, 5 and none on the G031
/// class). Counting rather than tabulating is the point: a variant the
/// pack adds tomorrow is described by its own header.
constexpr uint8_t dma_channels(uint8_t n) {
    uint8_t count = 0;
    for (uint8_t ch = 1; ch <= 8u; ++ch) {
        if (dma_channel_present(n, ch)) {
            count = ch;
        }
    }
    return count;
}

/// RCC_AHBENR bit that clocks DMAn - and, with it, the DMAMUX: 17.4.2
/// says the multiplexer is clocked as long as at least one DMA is.
constexpr uint32_t dma_clock_mask(uint8_t n) {
    switch (n) {
#if defined(RCC_AHBENR_DMA1EN)
        case 1: return RCC_AHBENR_DMA1EN;
#endif
#if defined(RCC_AHBENR_DMA2EN)
        case 2: return RCC_AHBENR_DMA2EN;
#endif
        default: return 0u;
    }
}

/// RCC_AHBRSTR bit that resets DMAn (and the DMAMUX with it - the reset
/// bit's own description says so, which is why a driver that resets one
/// controller while the other is streaming would be wrong).
constexpr uint32_t dma_reset_mask(uint8_t n) {
    switch (n) {
#if defined(RCC_AHBRSTR_DMA1RST)
        case 1: return RCC_AHBRSTR_DMA1RST;
#endif
#if defined(RCC_AHBRSTR_DMA2RST)
        case 2: return RCC_AHBRSTR_DMA2RST;
#endif
        default: return 0u;
    }
}

/// The NVIC line a DMA channel interrupts on. THREE VECTORS SERVE ALL
/// TWELVE (table 61): channel 1 alone, channels 2 and 3 together, and
/// one line for everything else - DMA1's channels 4..7, every DMA2
/// channel, AND the DMAMUX overrun. IRQn values are enumerators the
/// preprocessor cannot probe, so this reads the DEVICE SELECT macro,
/// exactly as usart_irq() and tim_irq() do; the spelling of the third
/// line differs on all three headers, which is the whole reason this
/// verb exists.
constexpr IRQn_Type dma_channel_irq(uint8_t n, uint8_t ch) {
    if (n == 1u && ch == 1u) {
        return DMA1_Channel1_IRQn;
    }
    if (n == 1u && (ch == 2u || ch == 3u)) {
        return DMA1_Channel2_3_IRQn;
    }
#if defined(STM32G0B1xx) || defined(STM32G0C1xx)
    return DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn;
#elif defined(STM32G071xx) || defined(STM32G081xx) || defined(STM32G051xx) || \
    defined(STM32G061xx) || defined(STM32G070xx)
    return DMA1_Ch4_7_DMAMUX1_OVR_IRQn;
#else
    return DMA1_Ch4_5_DMAMUX1_OVR_IRQn;
#endif
}

/// The DMAMUX overrun interrupt shares the third channel vector; named
/// separately because an application binding it is answering for the
/// multiplexer and not for a channel.
constexpr IRQn_Type dmamux_irq() { return dma_channel_irq(1u, 4u); }

/// Register block base of DMAMUX request-multiplexer channel x (0-based,
/// as 11.6.1 numbers them); 0 past the count.
constexpr uint32_t dmamux_channel_base(uint8_t x) {
    switch (x) {
#if defined(DMAMUX1_Channel0_BASE)
        case 0: return DMAMUX1_Channel0_BASE;
#endif
#if defined(DMAMUX1_Channel1_BASE)
        case 1: return DMAMUX1_Channel1_BASE;
#endif
#if defined(DMAMUX1_Channel2_BASE)
        case 2: return DMAMUX1_Channel2_BASE;
#endif
#if defined(DMAMUX1_Channel3_BASE)
        case 3: return DMAMUX1_Channel3_BASE;
#endif
#if defined(DMAMUX1_Channel4_BASE)
        case 4: return DMAMUX1_Channel4_BASE;
#endif
#if defined(DMAMUX1_Channel5_BASE)
        case 5: return DMAMUX1_Channel5_BASE;
#endif
#if defined(DMAMUX1_Channel6_BASE)
        case 6: return DMAMUX1_Channel6_BASE;
#endif
#if defined(DMAMUX1_Channel7_BASE)
        case 7: return DMAMUX1_Channel7_BASE;
#endif
#if defined(DMAMUX1_Channel8_BASE)
        case 8: return DMAMUX1_Channel8_BASE;
#endif
#if defined(DMAMUX1_Channel9_BASE)
        case 9: return DMAMUX1_Channel9_BASE;
#endif
#if defined(DMAMUX1_Channel10_BASE)
        case 10: return DMAMUX1_Channel10_BASE;
#endif
#if defined(DMAMUX1_Channel11_BASE)
        case 11: return DMAMUX1_Channel11_BASE;
#endif
        default: return 0u;
    }
}

/// How many request-multiplexer channels this device has: twelve on the
/// G0B1 class, seven on the G071 class, five on the G031 class - table
/// 54's own three numbers, counted off the header instead of copied.
constexpr uint8_t dmamux_channels() {
    uint8_t count = 0;
    for (uint8_t x = 0; x < 12u; ++x) {
        if (dmamux_channel_base(x) != 0u) {
            count = static_cast<uint8_t>(x + 1u);
        }
    }
    return count;
}

/// Which DMAMUX channel drives DMAn's channel `ch` (11.3.2: DMAMUX
/// channels 0..6 are DMA1's channels 1..7, channels 7..11 are DMA2's
/// 1..5). 0xFF for a channel this device does not have - the mapping is
/// hardwired, so it is arithmetic and not a table, but the PRESENCE
/// question is still the header's.
constexpr uint8_t dmamux_channel_of(uint8_t n, uint8_t ch) {
    if (!dma_channel_present(n, ch)) {
        return 0xFF;
    }
    return n == 1u ? static_cast<uint8_t>(ch - 1u) : static_cast<uint8_t>(6u + ch);
}

/// Register block base of DMAMUX request-generator channel x; 0 past the
/// count. Four on every part of the family (table 54).
constexpr uint32_t dmamux_generator_base(uint8_t x) {
    switch (x) {
#if defined(DMAMUX1_RequestGenerator0_BASE)
        case 0: return DMAMUX1_RequestGenerator0_BASE;
#endif
#if defined(DMAMUX1_RequestGenerator1_BASE)
        case 1: return DMAMUX1_RequestGenerator1_BASE;
#endif
#if defined(DMAMUX1_RequestGenerator2_BASE)
        case 2: return DMAMUX1_RequestGenerator2_BASE;
#endif
#if defined(DMAMUX1_RequestGenerator3_BASE)
        case 3: return DMAMUX1_RequestGenerator3_BASE;
#endif
        default: return 0u;
    }
}

constexpr uint8_t dmamux_generators() {
    uint8_t count = 0;
    for (uint8_t x = 0; x < 4u; ++x) {
        if (dmamux_generator_base(x) != 0u) {
            count = static_cast<uint8_t>(x + 1u);
        }
    }
    return count;
}

/// The two DMAMUX status blocks, which the header declares as their own
/// structures at fixed offsets (0x080 and 0x140).
constexpr uint32_t dmamux_channel_status_base() { return DMAMUX1_ChannelStatus_BASE; }
constexpr uint32_t dmamux_generator_status_base() { return DMAMUX1_RequestGenStatus_BASE; }

// ---- ADC, DAC, COMP, VREFBUF (RM0444 ch. 15..18) -----------------------------

/// Register block base of the ADC, 0 where the device has none. Every
/// STM32G0 carries exactly one (15.1), so this is a presence probe and
/// not an instance table - but it is still the HEADER that says so.
constexpr uint32_t adc_base() {
#if defined(ADC1_BASE)
    return ADC1_BASE;
#else
    return 0;
#endif
}

constexpr bool adc_present() { return adc_base() != 0u; }

/// The ADC's COMMON register block - one register, ADC_CCR, at its own
/// base (the header puts it in a separate ADC_Common_TypeDef): the
/// asynchronous prescaler and the three internal-source enables. A
/// single-converter family still has it, because the block is shared
/// with converters this part does not carry.
constexpr uint32_t adc_common_base() {
#if defined(ADC1_COMMON_BASE)
    return ADC1_COMMON_BASE;
#else
    return 0;
#endif
}

/// RCC_APBENR2/APBRSTR2 bit of the ADC - the one analog block of this
/// family on APB2 (the DAC is on APB1, and the comparators have no
/// enable bit of their own at all: 18.3.3 shares SYSCFG's).
constexpr uint32_t adc_clock_mask() {
#if defined(RCC_APBENR2_ADCEN)
    return RCC_APBENR2_ADCEN;
#else
    return 0;
#endif
}

constexpr uint32_t adc_reset_mask() {
#if defined(RCC_APBRSTR2_ADCRST)
    return RCC_APBRSTR2_ADCRST;
#else
    return 0;
#endif
}

/// How many multiplexed channels the ADC has, counted off the header's
/// own CHSELx bits rather than copied from 15.3.8's "up to 19": the
/// count is what says which internal channel numbers are legal, and a
/// part that bonds fewer would describe itself.
constexpr uint8_t adc_channels() {
    uint8_t count = 0;
#if defined(ADC_CHSELR_CHSEL0_Msk)
    count = 1;
#endif
#if defined(ADC_CHSELR_CHSEL11_Msk)
    count = 12;
#endif
#if defined(ADC_CHSELR_CHSEL14_Msk)
    count = 15;
#endif
#if defined(ADC_CHSELR_CHSEL18_Msk)
    count = 19;
#endif
    return count;
}

/// The three channels that are not pads (15.3.8): the temperature
/// sensor, the internal reference and the divided battery pin. Each is
/// probed through the ADC_CCR enable bit that WAKES it, because that bit
/// is the thing a driver must set and the thing a part without the
/// source does not declare. 0xFF where the source is absent.
constexpr uint8_t adc_temperature_channel() {
#if defined(ADC_CCR_TSEN_Msk)
    return 12;
#else
    return 0xFF;
#endif
}

constexpr uint8_t adc_vrefint_channel() {
#if defined(ADC_CCR_VREFEN_Msk)
    return 13;
#else
    return 0xFF;
#endif
}

constexpr uint8_t adc_vbat_channel() {
#if defined(ADC_CCR_VBATEN_Msk)
    return 14;
#else
    return 0xFF;
#endif
}

/// The NVIC line of the ADC. On the G0B1/G0C1 and the G071 class it is
/// SHARED WITH THE THREE COMPARATORS and with EXTI lines 17/18/20 with
/// them (table 61); on the G031 class, which has no comparator at all,
/// the ADC has a line to itself under a different name. IRQn values are
/// enumerators the preprocessor cannot probe, so - as for usart_irq(),
/// tim_irq() and dma_channel_irq() - this reads the DEVICE SELECT macro.
constexpr IRQn_Type adc_irq() {
#if defined(STM32G0B1xx) || defined(STM32G0C1xx) || defined(STM32G071xx) || \
    defined(STM32G081xx) || defined(STM32G070xx)
    return ADC1_COMP_IRQn;
#else
    return ADC1_IRQn;
#endif
}

/// Register block base of the DAC, 0 where there is none: 16.3's own
/// table says the G031/G041 have no DAC, and their header declares no
/// DAC1_BASE, which is the form this file trusts.
constexpr uint32_t dac_base() {
#if defined(DAC1_BASE)
    return DAC1_BASE;
#else
    return 0;
#endif
}

constexpr bool dac_present() { return dac_base() != 0u; }

/// Output channels of the DAC: two where the header declares the second
/// channel's enable bit, one where it does not, none where there is no
/// DAC.
constexpr uint8_t dac_channels() {
    if (!dac_present()) {
        return 0;
    }
#if defined(DAC_CR_EN2_Msk)
    return 2;
#else
    return 1;
#endif
}

constexpr uint32_t dac_clock_mask() {
#if defined(RCC_APBENR1_DAC1EN)
    return RCC_APBENR1_DAC1EN;
#else
    return 0;
#endif
}

constexpr uint32_t dac_reset_mask() {
#if defined(RCC_APBRSTR1_DAC1RST)
    return RCC_APBRSTR1_DAC1RST;
#else
    return 0;
#endif
}

/// The DAC's NVIC line - the underrun interrupt's only route, and it is
/// SHARED WITH TIM6 AND LPTIM1 (table 61). The enumerator's own name
/// says so, which is why this verb exists rather than a driver naming
/// the vector.
constexpr IRQn_Type dac_irq() {
#if defined(DAC1_BASE)
    return TIM6_DAC_LPTIM1_IRQn;
#else
    return NonMaskableInt_IRQn;
#endif
}

/// Register block base of COMPn (n = 1..3), 0 where the device has none.
/// 18.1: the third comparator is the G0B1/G0C1's alone, and the G031
/// class has no comparator at all - and each header says exactly that by
/// declaring or not declaring COMPn_BASE.
constexpr uint32_t comp_base(uint8_t n) {
    switch (n) {
#if defined(COMP1_BASE)
        case 1: return COMP1_BASE;
#endif
#if defined(COMP2_BASE)
        case 2: return COMP2_BASE;
#endif
#if defined(COMP3_BASE)
        case 3: return COMP3_BASE;
#endif
        default: return 0;
    }
}

constexpr bool comp_present(uint8_t n) { return comp_base(n) != 0u; }

/// How many comparators this device has, counted off those probes.
constexpr uint8_t comp_count() {
    uint8_t count = 0;
    for (uint8_t n = 1; n <= 3; ++n) {
        if (comp_base(n) != 0u) {
            count = n;
        }
    }
    return count;
}

/// The EXTI line COMPn's output raises (13.5.1's line table: 17, 18 and
/// 20, all three CONFIGURABLE lines, so a sense must be chosen before
/// anything is pending). 0xFF for a comparator this device has not got.
/// The NUMBERS are the manual's - no header of this pack spells them -
/// which is why they live here beside exti_port_code() and not in a
/// driver.
constexpr uint8_t comp_exti_line(uint8_t n) {
    if (!comp_present(n)) {
        return 0xFF;
    }
    switch (n) {
        case 1: return 17;
        case 2: return 18;
        case 3: return 20;
        default: return 0xFF;
    }
}

/// The NVIC line a comparator's EXTI interrupt arrives on: the ADC's,
/// shared by all three (table 61) - which is why a handler for one is a
/// dispatcher for four sources.
constexpr IRQn_Type comp_irq() { return adc_irq(); }

/// Register block base of the voltage reference buffer, 0 where the
/// device has none.
constexpr uint32_t vrefbuf_base() {
#if defined(VREFBUF_BASE)
    return VREFBUF_BASE;
#else
    return 0;
#endif
}

constexpr bool vrefbuf_present() { return vrefbuf_base() != 0u; }

// ---- PWR: what the low-power chapter's option space costs per part ---------
//
// Chapter 4 is the first in this stratum whose OWN register fields are
// per-variant, rather than a peripheral being present or absent:
//  - the six wake-up pins WKUPx are a SPARSE set (the G031 bonds 1, 2, 4
//    and 6; the G071 adds 5; only the G0B1/G0C1 has all six), and a
//    missing one has no EWUPx bit, no WPx polarity bit, no WUFx flag and
//    no CWUFx clear - four registers agreeing, so one probe answers for
//    all four;
//  - the Standby pull-up/pull-down registers follow the GPIO bonding
//    (PWR_PUCRE/PWR_PDCRE are the G0B1/G0C1's alone, exactly as GPIOE is),
//    and they are STRUCT MEMBERS: a part without them cannot even have
//    the address taken, which is why the accessor below returns a pointer;
//  - VDDIO2 exists only where the second I/O supply does.

/// Which of WKUP1..6 this part bonds, as a mask with bit 0 = WKUP1 -
/// the shape PWR_CR3.EWUPx, PWR_CR4.WPx, PWR_SR1.WUFx and PWR_SCR.CWUFx
/// all share.
constexpr uint8_t pwr_wakeup_pin_mask = 0u
#if defined(PWR_CR3_EWUP1)
    | (1u << 0)
#endif
#if defined(PWR_CR3_EWUP2)
    | (1u << 1)
#endif
#if defined(PWR_CR3_EWUP3)
    | (1u << 2)
#endif
#if defined(PWR_CR3_EWUP4)
    | (1u << 3)
#endif
#if defined(PWR_CR3_EWUP5)
    | (1u << 4)
#endif
#if defined(PWR_CR3_EWUP6)
    | (1u << 5)
#endif
    ;

/// Is WKUPn (n = 1..6) bonded on this part?
constexpr bool pwr_wakeup_pin_present(uint8_t n) {
    return n >= 1u && n <= 6u &&
           (pwr_wakeup_pin_mask & (1u << (n - 1u))) != 0u;
}

/// How many of them there are (for a sweep that must not walk past the
/// end of the part).
constexpr uint8_t pwr_wakeup_pin_count() {
    uint8_t n = 0;
    for (uint8_t i = 1; i <= 6u; ++i) {
        if (pwr_wakeup_pin_present(i)) {
            ++n;
        }
    }
    return n;
}

/// PWR_PUCRx for port `letter`, null where the part has no such
/// register. A POINTER and not a mask, because these are struct members
/// and the absent ones cannot be named at all (4.4.8 and its
/// neighbours: x = A, B, C, D, E, F, with E the G0B1/G0C1's alone).
inline volatile uint32_t* pwr_pullup_reg(char letter) {
    switch (letter) {
#if defined(GPIOA_BASE)
        case 'A': return &PWR->PUCRA;
#endif
#if defined(GPIOB_BASE)
        case 'B': return &PWR->PUCRB;
#endif
#if defined(GPIOC_BASE)
        case 'C': return &PWR->PUCRC;
#endif
#if defined(GPIOD_BASE)
        case 'D': return &PWR->PUCRD;
#endif
#if defined(GPIOE_BASE)
        case 'E': return &PWR->PUCRE;
#endif
#if defined(GPIOF_BASE)
        case 'F': return &PWR->PUCRF;
#endif
        default: return nullptr;
    }
}

/// PWR_PDCRx for port `letter`, null where absent.
inline volatile uint32_t* pwr_pulldown_reg(char letter) {
    switch (letter) {
#if defined(GPIOA_BASE)
        case 'A': return &PWR->PDCRA;
#endif
#if defined(GPIOB_BASE)
        case 'B': return &PWR->PDCRB;
#endif
#if defined(GPIOC_BASE)
        case 'C': return &PWR->PDCRC;
#endif
#if defined(GPIOD_BASE)
        case 'D': return &PWR->PDCRD;
#endif
#if defined(GPIOE_BASE)
        case 'E': return &PWR->PDCRE;
#endif
#if defined(GPIOF_BASE)
        case 'F': return &PWR->PDCRF;
#endif
        default: return nullptr;
    }
}

/// Does this part carry the second I/O supply, and with it PWR_CR2's
/// VDDIO2 monitor and PWR_SR2's flag for it (4.4.2, note on
/// PVM_VDDIO2)?
constexpr bool pwr_vddio2_present() {
#if defined(PWR_CR2_PVM_VDDIO2)
    return true;
#else
    return false;
#endif
}

/// The EXTI lines the RTC and the TAMP raise (table 65: 19 and 21, both
/// DIRECT - no trigger selection and no pending bit in the EXTI). The
/// NUMBERS are the manual's, like comp_exti_line()'s, which is why they
/// live here.
constexpr uint8_t rtc_exti_line = 19;
constexpr uint8_t tamp_exti_line = 21;

/// The vector both of them arrive on (table 61: one line for the whole
/// RTC domain).
constexpr IRQn_Type rtc_irq() { return RTC_TAMP_IRQn; }

/// How many TAMP backup registers this part implements: BKP0R..BKP4R on
/// every G0, counted off the struct rather than asserted.
constexpr uint8_t tamp_backup_registers() {
    return static_cast<uint8_t>(
        (sizeof(TAMP_TypeDef) - offsetof(TAMP_TypeDef, BKP0R)) /
        sizeof(uint32_t));
}

/// How many EXTERNAL tamper inputs (TAMP_INx) this part declares, and
/// IT IS NOT THE SAME NUMBER ACROSS THE FAMILY: the G0B1's header
/// declares TAMP1E..TAMP3E and the G071's and the G031's stop at two.
/// Counted off the enable bits themselves because table 1 does not say.
/// THE PACKAGE IS A SECOND QUESTION this stratum cannot ask - TAMP_IN3
/// is PE6 on this device and the LQFP64 does not bond port E (port.md's
/// standing per-package gap).
constexpr uint8_t tamp_external_inputs() {
    uint8_t n = 0;
#if defined(TAMP_CR1_TAMP1E_Msk)
    ++n;
#endif
#if defined(TAMP_CR1_TAMP2E_Msk)
    ++n;
#endif
#if defined(TAMP_CR1_TAMP3E_Msk)
    ++n;
#endif
    return n;
}

/// Does this part have the four INTERNAL tamper sources (LSE and HSE
/// monitoring, the calendar overflow, the manufacturer readout)? All
/// three headers of this pack declare ITAMP3E..ITAMP6E - unlike the
/// external inputs, which differ - and the probe stands so that a part
/// without them compiles rather than reaching for a missing mask.
constexpr bool tamp_has_internal_tampers() {
#if defined(TAMP_CR1_ITAMP3E_Msk)
    return true;
#else
    return false;
#endif
}

/// The lowest and highest internal tamper INDEX the manual numbers
/// (ITAMP3..ITAMP6), so a caller's argument is checked against the
/// chapter's own numbering rather than against a bit position.
constexpr uint8_t tamp_internal_first = 3;
constexpr uint8_t tamp_internal_last = 6;

// ---- FDCAN (ch. 36) ---------------------------------------------------------
//
// A WHOLE CHAPTER THAT IS ABSENT FROM MOST OF THE FAMILY: table 1 gives
// FDCAN1 and FDCAN2 to the G0B1/G0C1 alone, and the G071 and G031
// headers declare no FDCAN symbol at all - not a base, not a struct, not
// an interrupt enumerator. So every probe below answers "no" there and
// brio/stm32g0/fdcan.hpp compiles its BODY only where the header
// declares the block (the avrdx/opamp.hpp precedent: a chapter a part
// has not got is absent from that part's build).

/// Register block base of FDCANn (n = 1..2), 0 where the device has none.
constexpr uint32_t fdcan_base(uint8_t n) {
    switch (n) {
#if defined(FDCAN1_BASE)
        case 1: return FDCAN1_BASE;
#endif
#if defined(FDCAN2_BASE)
        case 2: return FDCAN2_BASE;
#endif
        default: return 0;
    }
}

constexpr bool fdcan_present(uint8_t n) { return fdcan_base(n) != 0u; }

/// How many FDCAN instances this device carries (two or none).
constexpr uint8_t fdcan_instances() {
    uint8_t k = 0;
    for (uint8_t n = 1; n <= 2; ++n) {
        if (fdcan_present(n)) {
            ++k;
        }
    }
    return k;
}

/// The SUBSYSTEM's configuration block (36.1, figure 392) - one CKDIV
/// register at 0x4000 6500 that divides the kernel clock for BOTH
/// instances. 36.4.37: "only FDCAN1 instance has FDCAN_CKDIV, which
/// changes clock divider for all instances", and the header agrees by
/// giving it a type of its own at a base of its own.
constexpr uint32_t fdcan_config_base() {
#if defined(FDCAN_CONFIG_BASE)
    return FDCAN_CONFIG_BASE;
#else
    return 0;
#endif
}

/// One instance's slice of the message RAM, in BYTES (36.3.6: 212 words
/// = 0x350, of which the last word is at 0x34C). The number is the
/// MANUAL's - no header of this pack spells the map - which is why it
/// lives here beside lptim_exti_line() and not in a driver.
constexpr uint32_t fdcan_ram_size_bytes = 0x350;

/// Where FDCANn's message RAM starts. 36.3.6 stacks the instances:
/// "the RAM start address for the FDCANn is computed by end address + 4
/// of FDCANn - 1", i.e. FDCAN1 at SRAMCAN + 0x000 and FDCAN2 at
/// SRAMCAN + 0x350. 0 where the device has no FDCAN.
constexpr uint32_t fdcan_ram_base(uint8_t n) {
#if defined(SRAMCAN_BASE)
    if (!fdcan_present(n)) {
        return 0;
    }
    return SRAMCAN_BASE + static_cast<uint32_t>(n - 1u) * fdcan_ram_size_bytes;
#else
    (void)n;
    return 0;
#endif
}

/// RCC_APBENR1.FDCANEN - ONE enable for the whole subsystem, both
/// instances and the configuration block behind it. 0 where absent.
constexpr uint32_t fdcan_clock_mask() {
#if defined(RCC_APBENR1_FDCANEN)
    return RCC_APBENR1_FDCANEN;
#else
    return 0;
#endif
}

/// RCC_APBRSTR1.FDCANRST - and ONE reset for the same subsystem, so a
/// reset asked for through either instance takes the other down with it
/// (measured: test_stm32_fdcan letter a).
constexpr uint32_t fdcan_reset_mask() {
#if defined(RCC_APBRSTR1_FDCANRST)
    return RCC_APBRSTR1_FDCANRST;
#else
    return 0;
#endif
}

/// Position of the FDCAN kernel-clock select in RCC_CCIPR2 (5.4.22) -
/// a DIFFERENT register from the CCIPR every other multiplexer of this
/// stratum lives in, and one the smaller headers do not even declare as
/// a struct member. 0xFF where absent.
constexpr uint8_t fdcan_clock_select_pos() {
#if defined(RCC_CCIPR2_FDCANSEL_Pos)
    return RCC_CCIPR2_FDCANSEL_Pos;
#else
    return 0xFF;
#endif
}

/// RCC_CCIPR2 itself - a STRUCT MEMBER only the G0B1/G0C1 header
/// declares (the flash_ecc2r() precedent). Null elsewhere, which is what
/// lets clock.hpp carry one verb for it on every header of the pack.
inline volatile uint32_t* rcc_ccipr2() {
#if defined(RCC_CCIPR2_FDCANSEL_Pos)
    return &RCC->CCIPR2;
#else
    return nullptr;
#endif
}

/// The NVIC lines of the FDCAN's two interrupt outputs. Table 61 puts
/// fdcan1_intr0_it AND fdcan2_intr0_it on TIM16's vector and both
/// intr1_it on TIM17's - so ONE line serves two peripherals and two
/// instances, and a handler has to ask each of them what it wants.
/// IRQn_Type values are enumerators the preprocessor cannot probe, so
/// this reads the DEVICE SELECT macro exactly as usart_irq() and
/// tim_irq() do; the names themselves only exist on the parts that have
/// an FDCAN, which is why the body is gated on the block's base.
/// `line` is 0 or 1.
constexpr IRQn_Type fdcan_irq(uint8_t line) {
#if defined(FDCAN1_BASE)
    return line == 0u ? TIM16_FDCAN_IT0_IRQn : TIM17_FDCAN_IT1_IRQn;
#else
    (void)line;
    return NonMaskableInt_IRQn;
#endif
}

} // namespace brio
