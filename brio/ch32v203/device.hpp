/*
 * device.hpp
 *
 * The silicon's own register map, in this stratum's words. Like the
 * CH32V00x and unlike every other 32-bit family here, this target
 * compiles with NO VENDOR HEADER: WCH ships its register definitions
 * inside the EVT package, whose licence is written for software running
 * on WCH parts, so the map lives here, read off the documents of record
 * (CH32F/V20x_V30x_V31x reference manual V2.3, CH32V203 datasheet V2.8
 * and the QingKe V4 microprocessor manual V1.1) and answerable to them.
 *
 * WHAT THAT COSTS. There is no device header to ask which instances
 * exist, so per-part variability has to be STATED: the build names the
 * part (cmake/ch32v203-parts.cmake), this file asks that definition ONCE
 * and includes the part's own table from parts/, and every driver reads
 * the facts as `device::` constexpr values - the memories, the bonded
 * pins of each port, the instances, the device class. Nine parts are the
 * family the reference manual and the datasheet describe.
 *
 * THE MANUAL IS FOUR FAMILIES WIDE. Its chapters cover the CH32F20x, the
 * CH32V20x, the CH32V30x and the CH32V31x together, and a register
 * description that names a class (their D6, D8, D8C, D8W) is making a
 * statement about SOME of them: the CH32V203 is CH32V20x_D6 for every
 * part up to the C8, and CH32V20x_D8 for the RB. Where a field belongs
 * to another class it is not spelled here at all; where it belongs to
 * one of ours, the comment says which.
 *
 * Register STRUCTS mirror the chapter's own field names and order, so a
 * reader can hold the manual beside the code. They are the only place in
 * the stratum where an address appears as a number.
 */

#pragma once

#include <stdint.h>

namespace brio {

// ---- the buses (RM 1.3, figure 1-11) --------------------------------------
// PB1 (the manual's "APB1" register names) at 0x40000000, PB2 at
// 0x40010000, HB at 0x40020000, and the USBFS block alone off in its own
// window at 0x50000000.
inline constexpr uint32_t pb1_base   = 0x40000000UL;
inline constexpr uint32_t pb2_base   = 0x40010000UL;
inline constexpr uint32_t hb_base    = 0x40020000UL;
inline constexpr uint32_t usbfs_base = 0x50000000UL;

} // namespace brio

// ---- the part -------------------------------------------------------------
// The one place the build's part definition is asked (the file header).
#if defined(CH32V203F6)
#include "ch32v203/parts/ch32v203f6.hpp"
#elif defined(CH32V203F8)
#include "ch32v203/parts/ch32v203f8.hpp"
#elif defined(CH32V203G6)
#include "ch32v203/parts/ch32v203g6.hpp"
#elif defined(CH32V203G8)
#include "ch32v203/parts/ch32v203g8.hpp"
#elif defined(CH32V203K6)
#include "ch32v203/parts/ch32v203k6.hpp"
#elif defined(CH32V203K8)
#include "ch32v203/parts/ch32v203k8.hpp"
#elif defined(CH32V203C6)
#include "ch32v203/parts/ch32v203c6.hpp"
#elif defined(CH32V203C8)
#include "ch32v203/parts/ch32v203c8.hpp"
#elif defined(CH32V203RB)
#include "ch32v203/parts/ch32v203rb.hpp"
#else
#error "brio ch32v203: the build must define the part (ch32v203/cmake/ch32v203-parts.cmake derives it from CH32V203_MCU), and brio/ch32v203/parts/ must hold its table"
#endif

namespace brio {

// ---- RCC (RM ch. 3) -------------------------------------------------------
struct RccRegs {
    volatile uint32_t CTLR;       ///< 0x00 clock control: HSI, HSE, PLL
    volatile uint32_t CFGR0;      ///< 0x04 SW/SWS, the prescalers, the PLL, MCO
    volatile uint32_t INTR;       ///< 0x08 clock interrupt flags and enables
    volatile uint32_t PB2PRSTR;   ///< 0x0c PB2 peripheral reset
    volatile uint32_t PB1PRSTR;   ///< 0x10 PB1 peripheral reset
    volatile uint32_t HBPCENR;    ///< 0x14 HB clock enables (DMA, SRAM, CRC, USBFS)
    volatile uint32_t PB2PCENR;   ///< 0x18 PB2 clock enables (AFIO, ports, USART1, ...)
    volatile uint32_t PB1PCENR;   ///< 0x1c PB1 clock enables
    volatile uint32_t BDCTLR;     ///< 0x20 the backup domain: LSE, the RTC's source
    volatile uint32_t RSTSCKR;    ///< 0x24 reset causes, LSI
    volatile uint32_t HBRSTR;     ///< 0x28 HB peripheral reset
    volatile uint32_t CFGR2;      ///< 0x2c the second configuration register
};

inline RccRegs* rcc() { return reinterpret_cast<RccRegs*>(hb_base + 0x1000); }

/// RCC_CTLR (3.4.1)
inline constexpr uint32_t rcc_hsion        = 1UL << 0;
inline constexpr uint32_t rcc_hsirdy       = 1UL << 1;
inline constexpr uint32_t rcc_hsitrim_mask = 0x1FUL << 3;   ///< the user trim, 16 = centre
inline constexpr uint32_t rcc_hsical_mask  = 0xFFUL << 8;   ///< the factory calibration, read-only
inline constexpr uint32_t rcc_hseon        = 1UL << 16;
inline constexpr uint32_t rcc_hserdy       = 1UL << 17;
inline constexpr uint32_t rcc_hsebyp       = 1UL << 18;     ///< an external clock, not a crystal
inline constexpr uint32_t rcc_csson        = 1UL << 19;     ///< HSE failure detection (needs HSE)
inline constexpr uint32_t rcc_pllon        = 1UL << 24;
inline constexpr uint32_t rcc_pllrdy       = 1UL << 25;

/// The two roots this silicon carries, as rates. The HSI is the 8 MHz
/// factory-trimmed RC every part boots on; the LSI is nominal only (its
/// spread is wide, which is why anything timed by it is measured).
inline constexpr uint32_t hsi_hz = 8'000'000UL;
inline constexpr uint32_t lsi_hz = 40'000UL;

/// RCC_CFGR0 (3.4.2)
inline constexpr uint32_t rcc_sw_mask       = 0x3UL << 0;
inline constexpr uint32_t rcc_sw_hsi        = 0x0UL << 0;
inline constexpr uint32_t rcc_sw_hse        = 0x1UL << 0;
inline constexpr uint32_t rcc_sw_pll        = 0x2UL << 0;
inline constexpr uint32_t rcc_sws_mask      = 0x3UL << 2;
inline constexpr uint32_t rcc_sws_hsi       = 0x0UL << 2;
inline constexpr uint32_t rcc_sws_hse       = 0x1UL << 2;
inline constexpr uint32_t rcc_sws_pll       = 0x2UL << 2;
inline constexpr uint32_t rcc_hpre_mask     = 0xFUL << 4;    ///< HB: 0xxx none, 1000 /2 .. 1111 /512
inline constexpr uint32_t rcc_ppre1_mask    = 0x7UL << 8;    ///< PB1: 0xx none, 100 /2 .. 111 /16
inline constexpr uint32_t rcc_ppre1_shift   = 8;
inline constexpr uint32_t rcc_ppre2_mask    = 0x7UL << 11;   ///< PB2, same encoding
inline constexpr uint32_t rcc_ppre2_shift   = 11;
inline constexpr uint32_t rcc_adcpre_mask   = 0x3UL << 14;   ///< PB2 /2 /4 /6 /8, ADC clock <= 14 MHz
inline constexpr uint32_t rcc_pllsrc        = 1UL << 16;     ///< 0 the HSI, 1 the HSE
inline constexpr uint32_t rcc_pllxtpre      = 1UL << 17;     ///< the HSE halved into the PLL
inline constexpr uint32_t rcc_pllmul_mask   = 0xFUL << 18;
inline constexpr uint32_t rcc_pllmul_shift  = 18;
inline constexpr uint32_t rcc_usbpre_mask   = 0x3UL << 22;   ///< the PLL /1 /2 /3 for USB's 48 MHz
inline constexpr uint32_t rcc_usbpre_shift  = 22;
inline constexpr uint32_t rcc_mco_mask      = 0xFUL << 24;
inline constexpr uint32_t rcc_mco_none      = 0x0UL << 24;
inline constexpr uint32_t rcc_mco_sysclk    = 0x4UL << 24;
inline constexpr uint32_t rcc_mco_hsi       = 0x5UL << 24;
inline constexpr uint32_t rcc_mco_hse       = 0x6UL << 24;
inline constexpr uint32_t rcc_mco_pll_div2  = 0x7UL << 24;

/// PLLMUL codes. The ladder is x2..x16 in order and then x18 - the one
/// code that is not its own index plus two, and the one that reaches
/// this family's ceiling from the 8 MHz HSI (3.4.2, CH32V20x_D6/D8).
inline constexpr uint32_t rcc_pllmul_code(uint32_t mul) {
    return mul == 18 ? 0xFUL : (mul - 2);
}
inline constexpr uint32_t rcc_pllmul_of(uint32_t code) {
    return code == 0xF ? 18UL : (code + 2);
}

/// RCC_INTR (3.4.3): the ready flags, their enables, the write-one clears.
inline constexpr uint32_t rcc_lsirdyf  = 1UL << 0;
inline constexpr uint32_t rcc_lserdyf  = 1UL << 1;
inline constexpr uint32_t rcc_hsirdyf  = 1UL << 2;
inline constexpr uint32_t rcc_hserdyf  = 1UL << 3;
inline constexpr uint32_t rcc_pllrdyf  = 1UL << 4;
inline constexpr uint32_t rcc_cssf     = 1UL << 7;
inline constexpr uint32_t rcc_lsirdyie = 1UL << 8;
inline constexpr uint32_t rcc_lserdyie = 1UL << 9;
inline constexpr uint32_t rcc_hsirdyie = 1UL << 10;
inline constexpr uint32_t rcc_hserdyie = 1UL << 11;
inline constexpr uint32_t rcc_pllrdyie = 1UL << 12;
inline constexpr uint32_t rcc_lsirdyc  = 1UL << 16;
inline constexpr uint32_t rcc_lserdyc  = 1UL << 17;
inline constexpr uint32_t rcc_hsirdyc  = 1UL << 18;
inline constexpr uint32_t rcc_hserdyc  = 1UL << 19;
inline constexpr uint32_t rcc_pllrdyc  = 1UL << 20;
inline constexpr uint32_t rcc_cssc     = 1UL << 23;

/// RCC_RSTSCKR (3.4.10): the LSI lives with the reset flags.
inline constexpr uint32_t rcc_lsion  = 1UL << 0;
inline constexpr uint32_t rcc_lsirdy = 1UL << 1;

/// RCC_HBPCENR (3.4.6)
inline constexpr uint32_t rcc_hb_dma1  = 1UL << 0;
inline constexpr uint32_t rcc_hb_dma2  = 1UL << 1;    ///< not on this family's parts
inline constexpr uint32_t rcc_hb_sram  = 1UL << 2;
inline constexpr uint32_t rcc_hb_crc   = 1UL << 6;
inline constexpr uint32_t rcc_hb_usbfs = 1UL << 12;   ///< the host/device controller

/// RCC_PB2PCENR (3.4.7): one bit per peripheral on the PB2 bus.
inline constexpr uint32_t rcc_pb2_afio   = 1UL << 0;
inline constexpr uint32_t rcc_pb2_gpioa  = 1UL << 2;
inline constexpr uint32_t rcc_pb2_gpiob  = 1UL << 3;
inline constexpr uint32_t rcc_pb2_gpioc  = 1UL << 4;
inline constexpr uint32_t rcc_pb2_gpiod  = 1UL << 5;
inline constexpr uint32_t rcc_pb2_gpioe  = 1UL << 6;
inline constexpr uint32_t rcc_pb2_adc1   = 1UL << 9;
inline constexpr uint32_t rcc_pb2_adc2   = 1UL << 10;
inline constexpr uint32_t rcc_pb2_tim1   = 1UL << 11;
inline constexpr uint32_t rcc_pb2_spi1   = 1UL << 12;
inline constexpr uint32_t rcc_pb2_usart1 = 1UL << 14;

/// RCC_PB1PCENR (3.4.8). PWR and BKP are two of them: their registers
/// answer rubbish until their gates are open.
inline constexpr uint32_t rcc_pb1_tim2   = 1UL << 0;
inline constexpr uint32_t rcc_pb1_tim3   = 1UL << 1;
inline constexpr uint32_t rcc_pb1_tim4   = 1UL << 2;
inline constexpr uint32_t rcc_pb1_wwdg   = 1UL << 11;
inline constexpr uint32_t rcc_pb1_spi2   = 1UL << 14;
inline constexpr uint32_t rcc_pb1_usart2 = 1UL << 17;
inline constexpr uint32_t rcc_pb1_usart3 = 1UL << 18;
inline constexpr uint32_t rcc_pb1_uart4  = 1UL << 19;
inline constexpr uint32_t rcc_pb1_i2c1   = 1UL << 21;
inline constexpr uint32_t rcc_pb1_i2c2   = 1UL << 22;
inline constexpr uint32_t rcc_pb1_usbd   = 1UL << 23;   ///< the full-speed device controller
inline constexpr uint32_t rcc_pb1_can1   = 1UL << 25;
inline constexpr uint32_t rcc_pb1_bkp    = 1UL << 27;
inline constexpr uint32_t rcc_pb1_pwr    = 1UL << 28;

// ---- GPIO (RM ch. 10) -----------------------------------------------------
// The F1 shape whole: sixteen pins a port, four configuration bits each
// (CNF above MODE) split over two registers.
struct GpioRegs {
    volatile uint32_t CFGLR;      ///< 0x00 pins 0..7, four bits each
    volatile uint32_t CFGHR;      ///< 0x04 pins 8..15
    volatile uint32_t INDR;       ///< 0x08 input data
    volatile uint32_t OUTDR;      ///< 0x0c output data (and the pull's direction)
    volatile uint32_t BSHR;       ///< 0x10 set (low half) / reset (high half)
    volatile uint32_t BCR;        ///< 0x14 reset
    volatile uint32_t LCKR;       ///< 0x18 configuration lock
};

/// The ports this family addresses, in letter order. Which of them a
/// PART bonds, and which pins of each, is the part's own table
/// (device::port_pins) - the block answers either way.
inline constexpr uint32_t gpio_base_for(char port) {
    return port == 'A' ? pb2_base + 0x0800 :
           port == 'B' ? pb2_base + 0x0c00 :
           port == 'C' ? pb2_base + 0x1000 :
           port == 'D' ? pb2_base + 0x1400 :
           port == 'E' ? pb2_base + 0x1800 : 0;
}

inline constexpr uint32_t gpio_clock_for(char port) {
    return port == 'A' ? rcc_pb2_gpioa :
           port == 'B' ? rcc_pb2_gpiob :
           port == 'C' ? rcc_pb2_gpioc :
           port == 'D' ? rcc_pb2_gpiod :
           port == 'E' ? rcc_pb2_gpioe : 0;
}

// ---- AFIO (RM 10.2) -------------------------------------------------------
struct AfioRegs {
    volatile uint32_t ECR;        ///< 0x00 the event output
    volatile uint32_t PCFR1;      ///< 0x04 the remaps
    volatile uint32_t EXTICR[4];  ///< 0x08 which port feeds each EXTI line
    uint32_t RESERVED0;           ///< 0x18
    volatile uint32_t PCFR2;      ///< 0x1c the second remap register
};

inline AfioRegs* afio() { return reinterpret_cast<AfioRegs*>(pb2_base + 0x0000); }

// ---- USART (RM ch. 18) ----------------------------------------------------
// Sixteen-bit registers on word-aligned addresses, the F1 shape. The
// chapter's CTLR4 (MARK and SPACE parity) belongs to the CH32F20x_D8 and
// the CH32V30x and is not part of this family's register file.
struct UsartRegs {
    volatile uint16_t STATR;  uint16_t RESERVED0;   ///< 0x00
    volatile uint16_t DATAR;  uint16_t RESERVED1;   ///< 0x04
    volatile uint16_t BRR;    uint16_t RESERVED2;   ///< 0x08
    volatile uint16_t CTLR1;  uint16_t RESERVED3;   ///< 0x0c
    volatile uint16_t CTLR2;  uint16_t RESERVED4;   ///< 0x10
    volatile uint16_t CTLR3;  uint16_t RESERVED5;   ///< 0x14
    volatile uint16_t GPR;    uint16_t RESERVED6;   ///< 0x18
};

/// The instances this family addresses. USART1 is the PB2 one and runs
/// at PCLK2; the other three are PB1's. WHICH of them a PART offers is
/// its own table (device::has_usart), and on the smallest part that is
/// not the first n of them.
inline constexpr uint32_t usart_base_for(int n) {
    return n == 1 ? pb2_base + 0x3800 :
           n == 2 ? pb1_base + 0x4400 :
           n == 3 ? pb1_base + 0x4800 :
           n == 4 ? pb1_base + 0x4c00 : 0;
}

/// USART_STATR (18.10.1). PE, FE, NE, ORE and IDLE are read-only and
/// cleared by the STATR-then-DATAR read sequence; RXNE is cleared by a
/// DATAR read and TXE by a DATAR write; RXNE, TC, LBD and CTS are also
/// write-zero-to-clear (`usart_statr_rw0`).
inline constexpr uint16_t usart_pe   = 1U << 0;
inline constexpr uint16_t usart_fe   = 1U << 1;
inline constexpr uint16_t usart_ne   = 1U << 2;
inline constexpr uint16_t usart_ore  = 1U << 3;
inline constexpr uint16_t usart_idle = 1U << 4;
inline constexpr uint16_t usart_rxne = 1U << 5;
inline constexpr uint16_t usart_tc   = 1U << 6;
inline constexpr uint16_t usart_txe  = 1U << 7;
inline constexpr uint16_t usart_lbd  = 1U << 8;
inline constexpr uint16_t usart_cts  = 1U << 9;
inline constexpr uint16_t usart_statr_rw0 = usart_rxne | usart_tc | usart_lbd | usart_cts;

/// USART_CTLR1 (18.10.4)
inline constexpr uint16_t usart_sbk    = 1U << 0;    ///< send a break (self-clearing)
inline constexpr uint16_t usart_rwu    = 1U << 1;    ///< receiver in mute
inline constexpr uint16_t usart_re     = 1U << 2;
inline constexpr uint16_t usart_te     = 1U << 3;
inline constexpr uint16_t usart_idleie = 1U << 4;
inline constexpr uint16_t usart_rxneie = 1U << 5;
inline constexpr uint16_t usart_tcie   = 1U << 6;
inline constexpr uint16_t usart_txeie  = 1U << 7;
inline constexpr uint16_t usart_peie   = 1U << 8;
inline constexpr uint16_t usart_ps     = 1U << 9;    ///< 1: odd parity
inline constexpr uint16_t usart_pce    = 1U << 10;
inline constexpr uint16_t usart_wake   = 1U << 11;   ///< 1: address mark wakes, 0: idle line
inline constexpr uint16_t usart_m      = 1U << 12;   ///< 1: 9-bit word
inline constexpr uint16_t usart_ue     = 1U << 13;

/// USART_CTLR2 (18.10.5)
inline constexpr uint16_t usart_add_mask   = 0xFU << 0;   ///< the mute address, 4 bits
inline constexpr uint16_t usart_lbdl       = 1U << 5;     ///< 1: 11-bit break detection, 0: 10
inline constexpr uint16_t usart_lbdie      = 1U << 6;
inline constexpr uint16_t usart_lbcl       = 1U << 8;     ///< the clock pulse of the last data bit too
inline constexpr uint16_t usart_cpha       = 1U << 9;     ///< 1: capture on the second edge
inline constexpr uint16_t usart_cpol       = 1U << 10;    ///< 1: CK idles high
inline constexpr uint16_t usart_clken      = 1U << 11;    ///< the CK pad driven
inline constexpr uint16_t usart_stop_mask  = 0x3U << 12;  ///< 00 one, 01 half, 10 two, 11 one and a half
inline constexpr uint16_t usart_stop_shift = 12;
inline constexpr uint16_t usart_linen      = 1U << 14;

/// USART_CTLR3 (18.10.6)
inline constexpr uint16_t usart_eie    = 1U << 0;    ///< FE/ORE/NE interrupt, under DMAR
inline constexpr uint16_t usart_iren   = 1U << 1;
inline constexpr uint16_t usart_irlp   = 1U << 2;
inline constexpr uint16_t usart_hdsel  = 1U << 3;
inline constexpr uint16_t usart_nack   = 1U << 4;    ///< smartcard
inline constexpr uint16_t usart_scen   = 1U << 5;
inline constexpr uint16_t usart_dmar   = 1U << 6;
inline constexpr uint16_t usart_dmat   = 1U << 7;
inline constexpr uint16_t usart_rtse   = 1U << 8;
inline constexpr uint16_t usart_ctse   = 1U << 9;
inline constexpr uint16_t usart_ctsie  = 1U << 10;

/// USART_GPR (18.10.7): the IrDA prescaler, and the smartcard's guard
/// time above it.
inline constexpr uint16_t usart_psc_mask = 0x00FFU;
inline constexpr uint16_t usart_gt_mask  = 0xFF00U;

// ---- FLASH (RM ch. 32) ----------------------------------------------------
// The engine, for the chapter that will use it. There is no wait-state
// field on this family: the array is split into a zero-wait and a
// non-zero-wait region by the part, and CTLR's SCKMOD chooses whether
// the flash is accessed at the system clock or at half of it.
struct FlashRegs {
    volatile uint32_t ACTLR;         ///< 0x00
    volatile uint32_t KEYR;          ///< 0x04 the FPEC unlock keys (write-only)
    volatile uint32_t OBKEYR;        ///< 0x08 the option-byte unlock keys (write-only)
    volatile uint32_t STATR;         ///< 0x0c BSY, WRPRTERR, EOP
    volatile uint32_t CTLR;          ///< 0x10 PG, PER, MER, OBER, STRT, LOCK, the fast page verbs
    volatile uint32_t ADDR;          ///< 0x14 the page to program or erase
    uint32_t RESERVED0;              ///< 0x18
    volatile uint32_t OBR;           ///< 0x1c the option bytes as loaded
    volatile uint32_t WPR;           ///< 0x20 write protection, one bit per 4 KB
    volatile uint32_t MODEKEYR;      ///< 0x24 the fast-programming unlock keys (write-only)
};

inline FlashRegs* flash_ctl() { return reinterpret_cast<FlashRegs*>(hb_base + 0x2000); }

/// The two keys, written in this order and consecutively to KEYR (the
/// FPEC lock) and to MODEKEYR (the fast-programming lock).
inline constexpr uint32_t flash_key1 = 0x45670123UL;
inline constexpr uint32_t flash_key2 = 0xCDEF89ABUL;

/// The flash as the core reads it: the array answers at its own address
/// and at the alias the core boots from, and the image is linked at the
/// alias (ld/<part>.ld).
inline constexpr uint32_t flash_array_base = 0x08000000UL;
inline constexpr uint32_t flash_alias_base = 0x00000000UL;

// ---- ESIG (RM ch. 31) -----------------------------------------------------
/// What the factory wrote about this die: the flash capacity in
/// kilobytes and the 96-bit unique identifier.
inline uint16_t esig_flash_kbytes() {
    return *reinterpret_cast<volatile const uint16_t*>(0x1FFFF7E0UL);
}
inline uint32_t esig_uid_word(int n) {
    return *reinterpret_cast<volatile const uint32_t*>(0x1FFFF7E8UL + 4UL * static_cast<uint32_t>(n));
}

// ---- EXTEN (RM ch. 33) ----------------------------------------------------
/// One register on the HB bus, reset only by a system reset, holding
/// what did not fit anywhere else - and one thing that matters to the
/// clock tree: HSIPRE, which says whether the PLL is fed the HSI or half
/// of it. The PLL's own source bit is RCC's; its divider is here.
struct ExtenRegs {
    volatile uint32_t CTR;        ///< 0x00 EXTEN_CTR, reset 0x00000A40
    uint32_t RESERVED0;           ///< 0x04
    volatile uint32_t CTR2;       ///< 0x08
};

inline ExtenRegs* exten() { return reinterpret_cast<ExtenRegs*>(hb_base + 0x3800); }

inline constexpr uint32_t exten_usbd_ls        = 1UL << 0;   ///< the device controller in low speed
inline constexpr uint32_t exten_usbd_pullup    = 1UL << 1;   ///< its internal 1.5k on D+
inline constexpr uint32_t exten_eth_10m        = 1UL << 2;   ///< CH32V203RB alone
inline constexpr uint32_t exten_hsipre         = 1UL << 4;   ///< 1: the HSI whole into the PLL, 0: halved
inline constexpr uint32_t exten_lkupen         = 1UL << 6;   ///< the lock-up monitor, ON at reset
inline constexpr uint32_t exten_lkuprst        = 1UL << 7;   ///< write-1-clear: keep it out of a read-modify-write
inline constexpr uint32_t exten_ulldotrim_mask = 0x3UL << 8;
inline constexpr uint32_t exten_ldotrim_mask   = 0x3UL << 10;  ///< the core voltage, 10b = 1.1 V at reset

// ---- the core's own two blocks (QingKe V4 manual ch. 3 and 5) -------------
/// The system timer, which on this core is SIXTY-FOUR bits wide in two
/// register pairs - where the CH32V00x's V2C has one 32-bit counter and
/// one compare. Same block, same address, a different shape.
struct StkRegs {
    volatile uint32_t CTLR;       ///< 0xE000F000 SWIE, INIT, MODE, STRE, STCLK, STIE, STE
    volatile uint32_t SR;         ///< 0xE000F004 CNTIF (write 0 to clear)
    volatile uint32_t CNTL;       ///< 0xE000F008 the counter, low half
    volatile uint32_t CNTH;       ///< 0xE000F00c and high
    volatile uint32_t CMPLR;      ///< 0xE000F010 the comparison value, low half
    volatile uint32_t CMPHR;      ///< 0xE000F014 and high
};

inline StkRegs* stk() { return reinterpret_cast<StkRegs*>(0xE000F000UL); }

inline constexpr uint32_t stk_ste   = 1UL << 0;
inline constexpr uint32_t stk_stie  = 1UL << 1;
inline constexpr uint32_t stk_stclk = 1UL << 2;   ///< 1: HCLK, 0: HCLK/8
inline constexpr uint32_t stk_stre  = 1UL << 3;   ///< count again from 0 at the compare
inline constexpr uint32_t stk_mode  = 1UL << 4;   ///< 1: count down
inline constexpr uint32_t stk_init  = 1UL << 5;   ///< write-one: load the counter's initial value
inline constexpr uint32_t stk_swie  = 1UL << 31;  ///< raise the software interrupt
inline constexpr uint32_t stk_cntif = 1UL << 0;   ///< in SR, write 0 to clear

/// The interrupt controller (QingKe V4 manual 3.1). Mind the names: the
/// manual's ISR is the ENABLE status and its IPR the PENDING status -
/// the opposite of what the letters suggest.
struct PficRegs {
    volatile uint32_t ISR[8];     ///< 0x000 ENABLE status, read-only
    volatile uint32_t IPR[8];     ///< 0x020 PENDING status, read-only
    volatile uint32_t ITHRESDR;   ///< 0x040 priority threshold
    uint32_t RESERVED0;           ///< 0x044
    volatile uint32_t CFGR;       ///< 0x048
    volatile uint32_t GISR;       ///< 0x04c global status
    uint8_t RESERVED1[0x10];      ///< 0x050
    volatile uint32_t VTFADDRR[4];///< 0x060 the four free vectored entries
    uint8_t RESERVED2[0x90];      ///< 0x070
    volatile uint32_t IENR[8];    ///< 0x100 write 1 to ENABLE a line
    uint8_t RESERVED3[0x60];
    volatile uint32_t IRER[8];    ///< 0x180 write 1 to DISABLE a line
    uint8_t RESERVED4[0x60];
    volatile uint32_t IPSR[8];    ///< 0x200 write 1 to SET a line pending
    uint8_t RESERVED5[0x60];
    volatile uint32_t IPRR[8];    ///< 0x280 write 1 to CLEAR a pending line
    uint8_t RESERVED6[0x60];
    volatile uint32_t IACTR[8];   ///< 0x300 active (being serviced), read-only
    uint8_t RESERVED7[0xE0];
    volatile uint8_t IPRIOR[64];  ///< 0x400 one priority byte per line
};

inline PficRegs* pfic() { return reinterpret_cast<PficRegs*>(0xE000E000UL); }

/// PFIC_SCTLR, apart from the block: what a WFI means on this core.
inline volatile uint32_t& pfic_sctlr() {
    return *reinterpret_cast<volatile uint32_t*>(0xE000ED10UL);
}

inline constexpr uint32_t sctlr_sleeponexit = 1UL << 1;
inline constexpr uint32_t sctlr_sleepdeep   = 1UL << 2;
inline constexpr uint32_t sctlr_wfitowfe    = 1UL << 3;   ///< the NEXT wfi acts as a WFE
inline constexpr uint32_t sctlr_sevonpend   = 1UL << 4;   ///< any interrupt turning pending is a wake event
inline constexpr uint32_t sctlr_setevent    = 1UL << 5;   ///< write-one: latch an event by hand

/// Interrupt numbers, which on this core ARE the vector table's word
/// indices (QingKe V4 manual 3.2): the word at address 0 is an
/// instruction, the words after it are handler addresses. The tail from
/// 59 up is the DEVICE CLASS's, not the family's - the reference
/// manual's own table 9-2 is the union of this family with the CH32V30x
/// and names that range after the larger one's TIM8 (see
/// ch32v203/src/glue/startup_ch32v203.S). Two of the lines below
/// therefore MOVE with the class: on the CH32V20x_D8 the Ethernet pair
/// and two reserved words sit between the USBFS pair and them, so UART4
/// is 66 and the eighth DMA channel 67. The D8's own extra lines (the
/// Ethernet pair, TIM5 and the 32 kHz oscillator's two) are named here
/// when a driver of this stratum arms one.
enum class Irq : uint8_t {
    non_maskable     = 2,
    exception        = 3,
    ecall_machine    = 5,
    ecall_user       = 8,
    breakpoint       = 9,
    systick          = 12,
    software         = 14,
    wwdg             = 16,
    pvd              = 17,
    tamper           = 18,
    rtc              = 19,
    flash            = 20,
    rcc              = 21,
    exti0            = 22,
    exti1            = 23,
    exti2            = 24,
    exti3            = 25,
    exti4            = 26,
    dma1_channel1    = 27,
    dma1_channel2    = 28,
    dma1_channel3    = 29,
    dma1_channel4    = 30,
    dma1_channel5    = 31,
    dma1_channel6    = 32,
    dma1_channel7    = 33,
    adc1_2           = 34,
    usb_hp_can1_tx   = 35,
    usb_lp_can1_rx0  = 36,
    can1_rx1         = 37,
    can1_sce         = 38,
    exti9_5          = 39,
    tim1_brk         = 40,
    tim1_up          = 41,
    tim1_trg_com     = 42,
    tim1_cc          = 43,
    tim2             = 44,
    tim3             = 45,
    tim4             = 46,
    i2c1_ev          = 47,
    i2c1_er          = 48,
    i2c2_ev          = 49,
    i2c2_er          = 50,
    spi1             = 51,
    spi2             = 52,
    usart1           = 53,
    usart2           = 54,
    usart3           = 55,
    exti15_10        = 56,
    rtc_alarm        = 57,
    usb_wakeup       = 58,
    usbfs            = 59,   ///< the tail proper starts here (device::vector_count)
    usbfs_wakeup     = 60,
    uart4            = device::is_d8_class ? 66 : 61,
    dma1_channel8    = device::is_d8_class ? 67 : 62,
};

} // namespace brio
