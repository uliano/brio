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

} // namespace brio
