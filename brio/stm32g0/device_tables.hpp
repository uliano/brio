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
 *  - The EXTI's lines: sixteen GPIO lines everywhere, but which of the
 *    peripheral lines above them exist - and which of those are
 *    configurable rather than direct - is per part, and so is the
 *    second register group (RTSR2 and its four neighbours are members
 *    on the G0B1/G0C1 only, IMR2/EMR2 from the G071 class up).
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

} // namespace brio
