/*
 * device.hpp
 *
 * The silicon's own register map, in this stratum's words. Like the two
 * other WCH families here, this target compiles with NO VENDOR HEADER:
 * WCH ships its register definitions inside the EVT package, whose
 * licence is written for software running on WCH parts, so the map lives
 * here, read off the documents of record (the CH32X035 reference manual
 * V1.8, the CH32X035/X033 datasheet V1.7 and the QingKe V4 microprocessor
 * manual V1.1) and answerable to them. The EVT's own header was read
 * beside the manual as a cross-check and nothing else;
 * docs/ch32x035/README.md lists where the two disagree.
 *
 * WHAT THAT COSTS. There is no device header to ask which pads a package
 * bonds, so per-part variability has to be STATED: the build names the
 * part (cmake/ch32x035-parts.cmake in the ch32x035 project), this file
 * asks that definition ONCE and includes the part's own table from
 * parts/, and every driver reads the facts as `device::` constexpr
 * values. Seven parts are the series: six CH32X035 and the CH32X033F8P6,
 * one die, one memory size, seven packages.
 *
 * WHAT THIS FILE MAPS. The blocks the stratum's drivers reach - RCC, GPIO
 * and AFIO, the USART, the core's STK and PFIC - and three it only
 * describes: the EXTI's six registers, PWR's two, and FLASH with its
 * option bytes and the electronic signature, decoded READ-ONLY (no verb
 * of this stratum programs, erases or unlocks anything). A block this
 * stratum has no driver for (DMA, ADC, the timers, I2C, SPI, USBFS, USBPD,
 * PIOC, OPA, AWU, the watchdogs) is not mapped at all; the gap list of
 * docs/ch32x035/README.md names each with its reason.
 *
 * Register STRUCTS mirror the chapter's own field names and order, so a
 * reader can hold the manual beside the code. They are the only place in
 * the stratum where an address appears as a number.
 */

#pragma once

#include <stdint.h>

namespace brio {

// ---- the buses (RM 1.2, figure 1-2) ----------------------------------------
// PB1 at 0x40000000, PB2 at 0x40010000, HB at 0x40020000 - the manual's
// section titles name the registers of the three "APB1", "APB2" and
// "AHB", and its prose says PB1, PB2 and HB; the addresses are one.
inline constexpr uint32_t pb1_base = 0x40000000UL;
inline constexpr uint32_t pb2_base = 0x40010000UL;
inline constexpr uint32_t hb_base  = 0x40020000UL;

/// The packages of the series (datasheet V1.7, the model table). The part
/// table names its own; the pin tables of chapter 2 are keyed by these.
enum class Package : uint8_t { lqfp64m, lqfp48, qfn28, qsop28, qfn20, tssop20 };

} // namespace brio

// ---- the part -------------------------------------------------------------
// The one place the build's part definition is asked (the file header).
#if defined(CH32X035R8)
#include "ch32x035/parts/ch32x035r8.hpp"
#elif defined(CH32X035C8)
#include "ch32x035/parts/ch32x035c8.hpp"
#elif defined(CH32X035G8U)
#include "ch32x035/parts/ch32x035g8u.hpp"
#elif defined(CH32X035G8R)
#include "ch32x035/parts/ch32x035g8r.hpp"
#elif defined(CH32X035F8)
#include "ch32x035/parts/ch32x035f8.hpp"
#elif defined(CH32X035F7)
#include "ch32x035/parts/ch32x035f7.hpp"
#elif defined(CH32X033F8)
#include "ch32x035/parts/ch32x033f8.hpp"
#else
#error "brio ch32x035: the build must define the part (ch32x035/cmake/ch32x035-parts.cmake derives it from CH32X035_MCU), and brio/ch32x035/parts/ must hold its table"
#endif

namespace brio {

// ---- RCC (RM ch. 3) -------------------------------------------------------
// ONE root and no switch: the 48 MHz HSI is SYSCLK, and HCLK is SYSCLK
// through HPRE. So the block is small - no PLL, no HSE, no LSI (the
// watchdog's and the auto-wakeup's clock is the HSI divided by 1024), no
// clock-interrupt register at 0x08, no backup domain at 0x20.
struct RccRegs {
    volatile uint32_t CTLR;        ///< 0x00 HSION, HSIRDY, HSITRIM, HSICAL
    volatile uint32_t CFGR0;       ///< 0x04 HPRE and MCO
    uint32_t RESERVED0;            ///< 0x08
    volatile uint32_t APB2PRSTR;   ///< 0x0c PB2 peripheral reset
    volatile uint32_t APB1PRSTR;   ///< 0x10 PB1 peripheral reset
    volatile uint32_t AHBPCENR;    ///< 0x14 HB clock enables (DMA, SRAM, USBFS, USBPD)
    volatile uint32_t APB2PCENR;   ///< 0x18 PB2 clock enables (AFIO, ports, ADC, TIM1, SPI1, USART1)
    volatile uint32_t APB1PCENR;   ///< 0x1c PB1 clock enables (TIM2, TIM3, WWDG, USART2..4, I2C1, PWR)
    uint32_t RESERVED1;            ///< 0x20
    volatile uint32_t RSTSCKR;     ///< 0x24 the reset flags
    volatile uint32_t AHBRSTR;     ///< 0x28 HB peripheral reset (USBFS, PIOC, USBPD)
};

inline RccRegs* rcc() { return reinterpret_cast<RccRegs*>(hb_base + 0x1000); }

/// RCC_CTLR (3.4.1). Reset value 0x0000xx83: the HSI on and ready, the
/// trim at the centre, the factory calibration loaded.
inline constexpr uint32_t rcc_hsion        = 1UL << 0;
inline constexpr uint32_t rcc_hsirdy       = 1UL << 1;
inline constexpr uint32_t rcc_hsitrim_mask = 0x1FUL << 3;   ///< the user trim, 16 = centre
inline constexpr uint32_t rcc_hsical_mask  = 0xFFUL << 8;   ///< the factory calibration, read-only

/// RCC_CFGR0 (3.4.2). Reset value 0x00000050: HPRE = 0101, SYSCLK / 6.
inline constexpr uint32_t rcc_hpre_mask  = 0xFUL << 4;
inline constexpr uint32_t rcc_hpre_shift = 4;
inline constexpr uint32_t rcc_mco_mask   = 0x7UL << 24;    ///< 100 SYSCLK, 101 HSI, other codes no clock
inline constexpr uint32_t rcc_mco_shift  = 24;

/// RCC_APB2PRSTR and RCC_APB2PCENR (3.4.3, 3.4.6): one bit per block of
/// the PB2 bus, the same position in the reset and the enable register.
inline constexpr uint32_t rcc_pb2_afio   = 1UL << 0;
inline constexpr uint32_t rcc_pb2_gpioa  = 1UL << 2;
inline constexpr uint32_t rcc_pb2_gpiob  = 1UL << 3;
inline constexpr uint32_t rcc_pb2_gpioc  = 1UL << 4;
inline constexpr uint32_t rcc_pb2_adc1   = 1UL << 9;
inline constexpr uint32_t rcc_pb2_tim1   = 1UL << 11;
inline constexpr uint32_t rcc_pb2_spi1   = 1UL << 12;
inline constexpr uint32_t rcc_pb2_usart1 = 1UL << 14;

/// RCC_APB1PRSTR and RCC_APB1PCENR (3.4.4, 3.4.7): the PB1 bus.
inline constexpr uint32_t rcc_pb1_tim2   = 1UL << 0;
inline constexpr uint32_t rcc_pb1_tim3   = 1UL << 1;
inline constexpr uint32_t rcc_pb1_wwdg   = 1UL << 11;
inline constexpr uint32_t rcc_pb1_usart2 = 1UL << 17;
inline constexpr uint32_t rcc_pb1_usart3 = 1UL << 18;
inline constexpr uint32_t rcc_pb1_usart4 = 1UL << 19;
inline constexpr uint32_t rcc_pb1_i2c1   = 1UL << 21;
inline constexpr uint32_t rcc_pb1_pwr    = 1UL << 28;

/// RCC_AHBPCENR (3.4.5). Its reset value is 0x00021004: the USBPD's and
/// the USBFS's clocks are ON out of reset, and so is SRAMEN, which is the
/// SRAM's clock IN SLEEP MODE (1: on) and not the SRAM's clock as such.
inline constexpr uint32_t rcc_hb_dma1  = 1UL << 0;
inline constexpr uint32_t rcc_hb_sram  = 1UL << 2;
inline constexpr uint32_t rcc_hb_usbfs = 1UL << 12;
inline constexpr uint32_t rcc_hb_usbpd = 1UL << 17;

/// RCC_AHBRSTR (3.4.9): the HB blocks with a reset line - the DMA and the
/// SRAM have none.
inline constexpr uint32_t rcc_hbrst_usbfs = 1UL << 12;
inline constexpr uint32_t rcc_hbrst_pioc  = 1UL << 13;
inline constexpr uint32_t rcc_hbrst_usbpd = 1UL << 17;

/// RCC_RSTSCKR (3.4.8): the reset flags, which accumulate until RMVF is
/// written, and nothing else - the register the sibling families share
/// with the LSI holds no oscillator here. THE MANUAL GIVES TWO RESET
/// VALUES: table 3-1 says 0x0C000000, PORRSTF and PINRSTF both set by a
/// power-on, and 3.4.8's own bit table gives PINRSTF a reset value of 0.
/// Nothing here depends on which; what a power-on leaves is a reading.
inline constexpr uint32_t rcc_rmvf     = 1UL << 24;
inline constexpr uint32_t rcc_oparstf  = 1UL << 25;
inline constexpr uint32_t rcc_pinrstf  = 1UL << 26;
inline constexpr uint32_t rcc_porrstf  = 1UL << 27;
inline constexpr uint32_t rcc_sftrstf  = 1UL << 28;
inline constexpr uint32_t rcc_iwdgrstf = 1UL << 29;
inline constexpr uint32_t rcc_wwdgrstf = 1UL << 30;
inline constexpr uint32_t rcc_lpwrrstf = 1UL << 31;
inline constexpr uint32_t rcc_reset_flags = 0xFE000000UL;

/// The one root: the internal RC, factory-trimmed to 48 MHz (3.3.2; the
/// datasheet's table 3-9 gives -1.7 .. +1.6 per cent at 0..70 C).
inline constexpr uint32_t hsi_hz = 48'000'000UL;

// ---- GPIO (RM 8.3.1) ------------------------------------------------------
// The F1's nibble shape with THREE configuration registers, because a port
// here runs to 24 pins: CFGLR for 0..7, CFGHR for 8..15, CFGXR for 16..23.
// BSHR sets and resets pins 0..15 only, and BSXR does the same for 16..23
// in its two bytes; BCR resets all 24 at once. All of them must be
// accessed as words (8.3.1).
struct GpioRegs {
    volatile uint32_t CFGLR;      ///< 0x00 pins 0..7, four bits each
    volatile uint32_t CFGHR;      ///< 0x04 pins 8..15
    volatile uint32_t INDR;       ///< 0x08 input data, 24 bits
    volatile uint32_t OUTDR;      ///< 0x0c output data (and a pulled input's direction)
    volatile uint32_t BSHR;       ///< 0x10 set pins 0..15 (low half), reset them (high half)
    volatile uint32_t BCR;        ///< 0x14 reset pins 0..23
    volatile uint32_t LCKR;       ///< 0x18 configuration lock: LCK0..23, LCKK at bit 24
    volatile uint32_t CFGXR;      ///< 0x1c pins 16..23
    volatile uint32_t BSXR;       ///< 0x20 set pins 16..23 (bits 7:0), reset them (bits 23:16)
};

/// The ports this series addresses: A, B and C. Which pads of each a
/// PART bonds is its own table (device::port_pins) - the block answers
/// either way.
inline constexpr uint32_t gpio_base_for(char port) {
    return port == 'A' ? pb2_base + 0x0800 :
           port == 'B' ? pb2_base + 0x0c00 :
           port == 'C' ? pb2_base + 0x1000 : 0;
}

inline constexpr uint32_t gpio_clock_for(char port) {
    return port == 'A' ? rcc_pb2_gpioa :
           port == 'B' ? rcc_pb2_gpiob :
           port == 'C' ? rcc_pb2_gpioc : 0;
}

/// The pads that exist on the DIE, per port (RM figure 1-1 and the
/// datasheet's pin tables): PA0..PA23, PB0..PB21, PC0..PC7, PC10, PC11
/// and PC14..PC19. PC10 and PC11 have no package pin of their own: every
/// package that brings them out puts them on PC17's and PC16's pins
/// (datasheet table 2-1, note 4).
inline constexpr uint32_t gpio_die_pads(char port) {
    return port == 'A' ? 0x00FFFFFFUL :
           port == 'B' ? 0x003FFFFFUL :
           port == 'C' ? 0x000FCCFFUL : 0UL;
}

/// The pads with a switchable PULL-DOWN (8, 8.2; the datasheet's 1.4.19):
/// PA0..PA15, PC16 and PC17. Every pad has the pull-up.
inline constexpr uint32_t gpio_pull_down_pads(char port) {
    return port == 'A' ? 0x0000FFFFUL :
           port == 'C' ? 0x00030000UL : 0UL;
}

/// The two pads the debug port owns from reset (7.4.4's EXTI 24 and 25,
/// the datasheet's DIO and DCK): PC18 = SWDIO and PC19 = SWCLK, until
/// AFIO_PCFR1.SW_CFG hands them back to GPIO.
inline constexpr char debug_swdio_port = 'C';
inline constexpr uint8_t debug_swdio_pin = 18;
inline constexpr char debug_swclk_port = 'C';
inline constexpr uint8_t debug_swclk_pin = 19;

// ---- AFIO (RM 8.3.2) ------------------------------------------------------
// Four registers: the remaps (PCFR1), the EXTI port multiplexer in TWO
// registers of two bits a line (EXTICR1 for lines 0..15, EXTICR2 for
// 16..23), and CTLR - the USB and USB PD pads' pull-up modes, their
// source voltages and comparators, and four pads' input filters. There
// is no ECR at 0x00 and no second remap register.
struct AfioRegs {
    uint32_t RESERVED0;           ///< 0x00
    volatile uint32_t PCFR1;      ///< 0x04 the remaps and SW_CFG
    volatile uint32_t EXTICR[2];  ///< 0x08 lines 0..15, 0x0c lines 16..23
    uint32_t RESERVED1[2];        ///< 0x10
    volatile uint32_t CTLR;       ///< 0x18 reset value 0x00000045
};

inline AfioRegs* afio() { return reinterpret_cast<AfioRegs*>(pb2_base + 0x0000); }

/// AFIO_EXTICRx's two-bit port codes (8.3.2.2): 00 port A, 10 port B, 11
/// port C - and 01 reserved, which is NOT the sibling families' order.
inline constexpr uint8_t afio_exti_port_a = 0x0;
inline constexpr uint8_t afio_exti_port_b = 0x2;
inline constexpr uint8_t afio_exti_port_c = 0x3;

/// AFIO_CTLR (8.3.2.4), named for the map: the USB pads' pull-up modes
/// (UDP_PUE, UDM_PUE, reset 01), USB_PHY_V33 (reset 1), USB_IOEN, the PD
/// transceiver's USBPD_PHY_V33 and USBPD_IN_HVT, the BC protocol's source
/// voltages and comparator outputs, and the input filters of PA3, PA4,
/// PB5 and PB6. No verb of this stratum writes it.
inline constexpr uint32_t afio_ctlr_udm_pue_mask  = 0x3UL << 0;
inline constexpr uint32_t afio_ctlr_udp_pue_mask  = 0x3UL << 2;
inline constexpr uint32_t afio_ctlr_usb_phy_v33   = 1UL << 6;
inline constexpr uint32_t afio_ctlr_usb_ioen      = 1UL << 7;
inline constexpr uint32_t afio_ctlr_usbpd_phy_v33 = 1UL << 8;
inline constexpr uint32_t afio_ctlr_usbpd_in_hvt  = 1UL << 9;

// ---- EXTI (RM 7.4, 7.5.1) -------------------------------------------------
// Thirty lines: 0..23 the pads (one port per line, AFIO_EXTICRx), 24 and
// 25 the debug port's two pads, 26 the PVD, 27 the AUTO-WAKEUP, 28 the
// USB's wake-up and 29 the USB PD's (table 7-2). Mapped for the reader;
// the external interrupts have no driver in this stratum.
struct ExtiRegs {
    volatile uint32_t INTENR;     ///< 0x00
    volatile uint32_t EVENR;      ///< 0x04
    volatile uint32_t RTENR;      ///< 0x08
    volatile uint32_t FTENR;      ///< 0x0c
    volatile uint32_t SWIEVR;     ///< 0x10
    volatile uint32_t INTFR;      ///< 0x14 write 1 to clear
};

inline ExtiRegs* exti() { return reinterpret_cast<ExtiRegs*>(pb2_base + 0x0400); }

inline constexpr uint8_t exti_line_count = 30;
inline constexpr uint8_t exti_pad_lines  = 24;

// ---- USART (RM ch. 14) ----------------------------------------------------
// Sixteen-bit registers on word-aligned addresses, the F1 shape, and no
// fourth control register: the frame is M alone, eight or nine bits.
struct UsartRegs {
    volatile uint16_t STATR;  uint16_t RESERVED0;   ///< 0x00
    volatile uint16_t DATAR;  uint16_t RESERVED1;   ///< 0x04
    volatile uint16_t BRR;    uint16_t RESERVED2;   ///< 0x08
    volatile uint16_t CTLR1;  uint16_t RESERVED3;   ///< 0x0c
    volatile uint16_t CTLR2;  uint16_t RESERVED4;   ///< 0x10
    volatile uint16_t CTLR3;  uint16_t RESERVED5;   ///< 0x14
    volatile uint16_t GPR;    uint16_t RESERVED6;   ///< 0x18
};

/// The four instances. USART1 answers on PB2 and the other three on PB1,
/// but every one of them counts its divisor in HCLK (14.3): this series
/// has no peripheral-bus prescaler at all.
inline constexpr uint32_t usart_base_for(int n) {
    return n == 1 ? pb2_base + 0x3800 :
           n == 2 ? pb1_base + 0x4400 :
           n == 3 ? pb1_base + 0x4800 :
           n == 4 ? pb1_base + 0x4c00 : 0;
}

/// USART_STATR (14.10.1). PE, FE, NE, ORE and IDLE are read-only and
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

/// USART_CTLR1 (14.10.4)
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

/// USART_CTLR2 (14.10.5)
inline constexpr uint16_t usart_add_mask   = 0xFU << 0;   ///< the mute address, 4 bits
inline constexpr uint16_t usart_lbdl       = 1U << 5;     ///< 1: 11-bit break detection, 0: 10
inline constexpr uint16_t usart_lbdie      = 1U << 6;
inline constexpr uint16_t usart_lbcl       = 1U << 8;     ///< 1: NO clock pulse for the last data bit
inline constexpr uint16_t usart_cpha       = 1U << 9;     ///< 1: capture on the second edge
inline constexpr uint16_t usart_cpol       = 1U << 10;    ///< 1: CK idles high
inline constexpr uint16_t usart_clken      = 1U << 11;    ///< the CK pad driven
inline constexpr uint16_t usart_stop_mask  = 0x3U << 12;  ///< 00 one, 01 half, 10 two, 11 one and a half
inline constexpr uint16_t usart_stop_shift = 12;
inline constexpr uint16_t usart_linen      = 1U << 14;

/// USART_CTLR3 (14.10.6)
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

/// USART_GPR (14.10.7): the IrDA prescaler, and the smartcard's guard
/// time above it.
inline constexpr uint16_t usart_psc_mask = 0x00FFU;
inline constexpr uint16_t usart_gt_mask  = 0xFF00U;

// ---- PWR (RM ch. 2) -------------------------------------------------------
// Mapped for the reader: the sleep modes' PDDS, the PVD's threshold and
// its output, the flash's low-power bits. The power chapter has no
// driver in this stratum.
struct PwrRegs {
    volatile uint32_t CTLR;   ///< 0x00 PDDS, PLS[1:0], LP_REG, LP[1:0]; reset 0x00000400
    volatile uint32_t CSR;    ///< 0x04 PVD0, FLASH_ACK
};

inline PwrRegs* pwr() { return reinterpret_cast<PwrRegs*>(pb1_base + 0x7000); }

// ---- FLASH (RM ch. 20), decoded READ-ONLY -----------------------------------
// The interface's registers, of which this stratum WRITES one: ACTLR's
// wait states, which the clock owns. The keys, the program and erase
// verbs and the option-byte writes are mapped and never used here.
struct FlashRegs {
    volatile uint32_t ACTLR;         ///< 0x00 LATENCY[1:0]
    volatile uint32_t KEYR;          ///< 0x04 the FPEC unlock keys (write-only)
    volatile uint32_t OBKEYR;        ///< 0x08 the option-byte unlock keys (write-only)
    volatile uint32_t STATR;         ///< 0x0c BSY, WRPRTERR, EOP, FWAKE_FLAG, TURBO, the BOOT bits
    volatile uint32_t CTLR;          ///< 0x10 PER, MER, OBER, STRT, LOCK, FLOCK, FTPG, FTER, BUFLOAD, BUFRST, BER32
    volatile uint32_t ADDR;          ///< 0x14 the address to program or erase (write-only)
    uint32_t RESERVED0;              ///< 0x18
    volatile uint32_t OBR;           ///< 0x1c the option bytes as loaded
    volatile uint32_t WPR;           ///< 0x20 write protection, one bit per 2 KB
    volatile uint32_t MODEKEYR;      ///< 0x24 the fast-programming unlock keys (write-only)
    volatile uint32_t BOOT_MODEKEYR; ///< 0x28 the BOOT area's unlock keys (write-only)
};

inline FlashRegs* flash_ctl() { return reinterpret_cast<FlashRegs*>(hb_base + 0x2000); }

/// FLASH_ACTLR (20.3.1): the wait states a rate needs - 0 to 12 MHz, 1 to
/// 24, 2 to 48. The whole register is LATENCY: its other bits are
/// reserved, so a store of the field alone is the register.
inline constexpr uint32_t flash_latency_mask = 0x3UL;
constexpr uint32_t flash_latency_for(uint32_t hz) {
    return hz <= 12'000'000UL ? 0UL : hz <= 24'000'000UL ? 1UL : 2UL;
}

/// FLASH_STATR (20.3.4). BOOT_LOCK and the BOOT_MODE/BOOT_STATUS/BOOT_AVA
/// trio say where the running program was loaded from; TURBO is read-only.
inline constexpr uint32_t flash_bsy         = 1UL << 0;
inline constexpr uint32_t flash_wrprterr    = 1UL << 4;
inline constexpr uint32_t flash_eop         = 1UL << 5;
inline constexpr uint32_t flash_fwake_flag  = 1UL << 6;
inline constexpr uint32_t flash_turbo       = 1UL << 7;
inline constexpr uint32_t flash_boot_ava    = 1UL << 12;
inline constexpr uint32_t flash_boot_status = 1UL << 13;
inline constexpr uint32_t flash_boot_mode   = 1UL << 14;
inline constexpr uint32_t flash_boot_lock   = 1UL << 15;

/// FLASH_CTLR's two locks (20.3.5): LOCK over the whole interface, FLOCK
/// over the fast page verbs, both set out of reset.
inline constexpr uint32_t flash_lock  = 1UL << 7;
inline constexpr uint32_t flash_flock = 1UL << 15;

/// FLASH_OBR (20.3.7) as fields: what the user option bytes said at the
/// last system reset.
struct FlashOptionBits {
    bool option_error;        ///< OBERR: a byte and its inverse disagreed
    bool read_protected;      ///< RDPRT
    bool iwdg_software;       ///< IWDGSW: 1 = started by software (the reset value)
    bool stop_reset;          ///< STOP_RST as loaded
    bool standby_reset;       ///< STANDY_RST as loaded
    uint8_t reset_mode;       ///< RST_MODE[1:0], the reset pin's configuration delay
    uint8_t data0;            ///< the user data byte 0
    uint8_t data1;            ///< the user data byte 1
};

constexpr FlashOptionBits flash_option_bits(uint32_t obr) {
    return FlashOptionBits{
        (obr & (1UL << 0)) != 0u,
        (obr & (1UL << 1)) != 0u,
        (obr & (1UL << 2)) != 0u,
        (obr & (1UL << 3)) != 0u,
        (obr & (1UL << 4)) != 0u,
        static_cast<uint8_t>((obr >> 5) & 0x3u),
        static_cast<uint8_t>((obr >> 10) & 0xFFu),
        static_cast<uint8_t>((obr >> 18) & 0xFFu),
    };
}

inline FlashOptionBits flash_options() { return flash_option_bits(flash_ctl()->OBR); }

/// FLASH_WPR (20.3.8): one bit per 2 KB unit, 1 = NOT protected.
inline uint32_t flash_write_protection() { return flash_ctl()->WPR; }

/// The code flash as the core reads it: the array answers at its own
/// address and at the alias the core boots from, and the image is linked
/// at the alias (ld/<part>.ld).
inline constexpr uint32_t flash_array_base = 0x08000000UL;
inline constexpr uint32_t flash_alias_base = 0x00000000UL;

// ---- ESIG (RM ch. 19) -----------------------------------------------------
/// What the factory wrote about this die: the flash capacity in kilobytes
/// and the 96-bit unique identifier, readable in 8, 16 or 32 bits.
inline uint16_t esig_flash_kbytes() {
    return *reinterpret_cast<volatile const uint16_t*>(0x1FFFF7E0UL);
}
inline uint32_t esig_uid_word(int n) {
    return *reinterpret_cast<volatile const uint32_t*>(0x1FFFF7E8UL + 4UL * static_cast<uint32_t>(n));
}

/// The word at 0x1FFFF704. The reference manual does not name it; WCH's
/// peripheral library reads it as the chip identifier (DBGMCU_GetCHIPID,
/// whose comment lists one value per part, 0x035E06x1 for the
/// CH32X035F8U6) and keys a GPIO workaround on bits 7:4 of it (pin.hpp).
/// Read here so a program can PRINT it; nothing branches on it.
inline uint32_t chip_id_word() {
    return *reinterpret_cast<volatile const uint32_t*>(0x1FFFF704UL);
}

// ---- the core's own two blocks (RM 7.5.2, 7.5.5; QingKe V4 manual 3, 5) ------
/// The system timer, SIXTY-FOUR bits wide in two register pairs. The
/// manual's STK_CTLR has no INIT bit on this series ([30:5] reserved),
/// where the CH32V203's manual gives bit 5 one; nothing here uses it.
struct StkRegs {
    volatile uint32_t CTLR;       ///< 0xE000F000 SWIE, MODE, STRE, STCLK, STIE, STE
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
inline constexpr uint32_t stk_swie  = 1UL << 31;  ///< raise the software interrupt
inline constexpr uint32_t stk_cntif = 1UL << 0;   ///< in SR, write 0 to clear

/// The interrupt controller (RM 7.5.2). Mind the names: the manual's ISR
/// is the ENABLE status and its IPR the PENDING status - the opposite of
/// what the letters suggest. The manual gives four words to each bank
/// (lines 0..103) and a byte of priority to each of 256 lines.
struct PficRegs {
    volatile uint32_t ISR[4];     ///< 0x000 ENABLE status, read-only
    uint32_t RESERVED0[4];        ///< 0x010
    volatile uint32_t IPR[4];     ///< 0x020 PENDING status, read-only
    uint32_t RESERVED1[4];        ///< 0x030
    volatile uint32_t ITHRESDR;   ///< 0x040 priority threshold
    uint32_t RESERVED2;           ///< 0x044
    volatile uint32_t CFGR;       ///< 0x048 KEYCODE and SYSRST
    volatile uint32_t GISR;       ///< 0x04c global status: NESTSTA, GACTSTA, GPENDSTA
    volatile uint32_t VTFIDR;     ///< 0x050 the four free vectored entries' line numbers
    uint32_t RESERVED3[3];        ///< 0x054
    volatile uint32_t VTFADDRR[4];///< 0x060 and their addresses
    uint8_t RESERVED4[0x90];      ///< 0x070
    volatile uint32_t IENR[4];    ///< 0x100 write 1 to ENABLE a line
    uint8_t RESERVED5[0x70];
    volatile uint32_t IRER[4];    ///< 0x180 write 1 to DISABLE a line
    uint8_t RESERVED6[0x70];
    volatile uint32_t IPSR[4];    ///< 0x200 write 1 to SET a line pending
    uint8_t RESERVED7[0x70];
    volatile uint32_t IPRR[4];    ///< 0x280 write 1 to CLEAR a pending line
    uint8_t RESERVED8[0x70];
    volatile uint32_t IACTR[4];   ///< 0x300 active (being serviced), read-only
    uint8_t RESERVED9[0xF0];
    volatile uint8_t IPRIOR[256]; ///< 0x400 one priority byte per line
};

inline PficRegs* pfic() { return reinterpret_cast<PficRegs*>(0xE000E000UL); }

/// PFIC_SCTLR (7.5.2.38), apart from the block: what a WFI means on this
/// core, the deadlock reset's switch and a second way to the system
/// reset.
inline volatile uint32_t& pfic_sctlr() {
    return *reinterpret_cast<volatile uint32_t*>(0xE000ED10UL);
}

inline constexpr uint32_t sctlr_sleeponexit = 1UL << 1;
inline constexpr uint32_t sctlr_sleepdeep   = 1UL << 2;
inline constexpr uint32_t sctlr_wfitowfe    = 1UL << 3;   ///< the NEXT wfi acts as a WFE
inline constexpr uint32_t sctlr_sevonpend   = 1UL << 4;   ///< any interrupt turning pending is a wake event
inline constexpr uint32_t sctlr_setevent    = 1UL << 5;   ///< write-one: latch an event by hand
inline constexpr uint32_t sctlr_rsten       = 1UL << 6;   ///< 1: the core deadlock reset OFF
inline constexpr uint32_t sctlr_sysrst      = 1UL << 31;  ///< write-one: a system reset

/// Interrupt numbers, which on this core ARE the vector table's word
/// indices (RM table 7-1): the word at address 0 is an instruction, the
/// words after it are handler addresses. One table for the whole series
/// (ch32x035/src/glue/startup_ch32x035.S states the same fifty-five
/// words). Line 19 is reserved: this series' RCC has no interrupt.
enum class Irq : uint8_t {
    non_maskable   = 2,
    exception      = 3,
    ecall_machine  = 5,
    ecall_user     = 8,
    breakpoint     = 9,
    systick        = 12,
    software       = 14,
    wwdg           = 16,
    pvd            = 17,
    flash          = 18,
    exti7_0        = 20,
    awu            = 21,
    dma1_channel1  = 22,
    dma1_channel2  = 23,
    dma1_channel3  = 24,
    dma1_channel4  = 25,
    dma1_channel5  = 26,
    dma1_channel6  = 27,
    dma1_channel7  = 28,
    adc1           = 29,
    i2c1_ev        = 30,
    i2c1_er        = 31,
    usart1         = 32,
    spi1           = 33,
    tim1_brk       = 34,
    tim1_up        = 35,
    tim1_trg_com   = 36,
    tim1_cc        = 37,
    tim2_up        = 38,
    usart2         = 39,
    exti15_8       = 40,
    exti25_16      = 41,
    usart3         = 42,
    usart4         = 43,
    dma1_channel8  = 44,
    usbfs          = 45,
    usbfs_wakeup   = 46,
    pioc           = 47,
    opa            = 48,
    usbpd          = 49,
    usbpd_wakeup   = 50,
    tim2_cc        = 51,
    tim2_trg_com   = 52,
    tim2_brk       = 53,
    tim3           = 54,
};

/// The table's length: entries 0..54.
inline constexpr uint8_t vector_count = 55;

} // namespace brio
