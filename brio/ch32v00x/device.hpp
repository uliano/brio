/*
 * device.hpp
 *
 * The silicon's own register map, in this stratum's words. Unlike the
 * two Cortex-M0+ families, this target compiles with NO VENDOR HEADER:
 * WCH ships its register definitions inside the EVT package, whose
 * licence is written for software running on WCH parts and whose
 * headers carry no per-part tiering worth deriving from - so the map
 * lives here, read off the documents of record (CH32V00X reference
 * manual V1.5 and CH32V006 datasheet V2.0) and answerable to them.
 *
 * WHAT THAT COSTS. The other 32-bit strata ask the device header which
 * instances exist (`#if defined(TCB4)`); here there is nothing to ask,
 * so per-part variability has to be stated. Today the file states ONE
 * part, the CH32V006K8U6 on the bench, and every constant below is
 * that part's. The family tiering - which ports are bonded on which
 * package, which parts have USART2 and the OPCM - is born with the
 * second part, the same way samc21/device_tables.hpp was born with a
 * real second package to answer.
 *
 * Register STRUCTS mirror the chapter's own field names and order, so
 * a reader can hold the manual beside the code. They are the only
 * place in the stratum where an address appears as a number.
 */

#pragma once

#include <stdint.h>

namespace brio {

// ---- the buses ------------------------------------------------------------
// RM 2.2: PB1 at 0x40000000, PB2 at 0x40010000, HB at 0x40020000.
inline constexpr uint32_t pb1_base = 0x40000000UL;
inline constexpr uint32_t pb2_base = 0x40010000UL;
inline constexpr uint32_t hb_base  = 0x40020000UL;

// ---- RCC (RM ch. 3) -------------------------------------------------------
struct RccRegs {
    volatile uint32_t CTLR;       ///< 0x00 clock control: HSION/HSIRDY, PLLON/PLLRDY
    volatile uint32_t CFGR0;      ///< 0x04 SW/SWS, HPRE, ADCPRE, PLLSRC
    volatile uint32_t INTR;       ///< 0x08 clock interrupt flags
    volatile uint32_t PB2PRSTR;   ///< 0x0c PB2 peripheral reset
    volatile uint32_t PB1PRSTR;   ///< 0x10 PB1 peripheral reset
    volatile uint32_t HBPCENR;    ///< 0x14 HB clock enables (DMA, SRAM)
    volatile uint32_t PB2PCENR;   ///< 0x18 PB2 clock enables (GPIO, USART1, ...)
    volatile uint32_t PB1PCENR;   ///< 0x1c PB1 clock enables
    uint32_t RESERVED0;           ///< 0x20
    volatile uint32_t RSTSCKR;    ///< 0x24 reset causes, LSI
};

inline RccRegs* rcc() { return reinterpret_cast<RccRegs*>(hb_base + 0x1000); }

/// RCC_CTLR
inline constexpr uint32_t rcc_hsion   = 1UL << 0;
inline constexpr uint32_t rcc_hsirdy  = 1UL << 1;
inline constexpr uint32_t rcc_pllon   = 1UL << 24;
inline constexpr uint32_t rcc_pllrdy  = 1UL << 25;

/// RCC_CFGR0
inline constexpr uint32_t rcc_sw_mask   = 0x3UL << 0;
inline constexpr uint32_t rcc_sw_hsi    = 0x0UL << 0;
inline constexpr uint32_t rcc_sw_pll    = 0x2UL << 0;
inline constexpr uint32_t rcc_sws_mask  = 0x3UL << 2;
inline constexpr uint32_t rcc_sws_hsi   = 0x0UL << 2;
inline constexpr uint32_t rcc_sws_pll   = 0x2UL << 2;
inline constexpr uint32_t rcc_hpre_mask = 0xFUL << 4;
inline constexpr uint32_t rcc_pllsrc    = 1UL << 16;

/// RCC_PB2PCENR: one bit per peripheral on the PB2 bus (RM 3.4.7).
inline constexpr uint32_t rcc_pb2_afio   = 1UL << 0;
inline constexpr uint32_t rcc_pb2_gpioa  = 1UL << 2;
inline constexpr uint32_t rcc_pb2_gpiob  = 1UL << 3;
inline constexpr uint32_t rcc_pb2_gpioc  = 1UL << 4;
inline constexpr uint32_t rcc_pb2_gpiod  = 1UL << 5;
inline constexpr uint32_t rcc_pb2_adc1   = 1UL << 9;
inline constexpr uint32_t rcc_pb2_tim1   = 1UL << 11;
inline constexpr uint32_t rcc_pb2_spi1   = 1UL << 12;
inline constexpr uint32_t rcc_pb2_usart1 = 1UL << 14;

// ---- GPIO (RM ch. 10) -----------------------------------------------------
// One configuration register: this family bonds at most eight pins per
// port, so there is no CFGHR - four bits per pin, the F1 shape (CNF
// above MODE).
struct GpioRegs {
    volatile uint32_t CFGLR;      ///< 0x00 four bits per pin
    uint32_t RESERVED0;           ///< 0x04
    volatile uint32_t INDR;       ///< 0x08 input data
    volatile uint32_t OUTDR;      ///< 0x0c output data
    volatile uint32_t BSHR;       ///< 0x10 set (low half) / reset (high half)
    volatile uint32_t BCR;        ///< 0x14 reset
    volatile uint32_t LCKR;       ///< 0x18 configuration lock
};

/// The ports this stratum knows, in letter order. A letter with no
/// entry here is refused at compile time (pin.hpp).
inline constexpr uint32_t gpio_base_for(char port) {
    return port == 'A' ? pb2_base + 0x0800 :
           port == 'B' ? pb2_base + 0x0c00 :
           port == 'C' ? pb2_base + 0x1000 :
           port == 'D' ? pb2_base + 0x1400 : 0;
}

inline constexpr uint32_t gpio_clock_for(char port) {
    return port == 'A' ? rcc_pb2_gpioa :
           port == 'B' ? rcc_pb2_gpiob :
           port == 'C' ? rcc_pb2_gpioc :
           port == 'D' ? rcc_pb2_gpiod : 0;
}

// ---- USART (RM ch. 16) ----------------------------------------------------
// Sixteen-bit registers on word-aligned addresses, the F1 shape.
struct UsartRegs {
    volatile uint16_t STATR;  uint16_t RESERVED0;   ///< 0x00
    volatile uint16_t DATAR;  uint16_t RESERVED1;   ///< 0x04
    volatile uint16_t BRR;    uint16_t RESERVED2;   ///< 0x08
    volatile uint16_t CTLR1;  uint16_t RESERVED3;   ///< 0x0c
    volatile uint16_t CTLR2;  uint16_t RESERVED4;   ///< 0x10
    volatile uint16_t CTLR3;  uint16_t RESERVED5;   ///< 0x14
    volatile uint16_t GPR;    uint16_t RESERVED6;   ///< 0x18
};

/// USART_STATR
inline constexpr uint16_t usart_pe   = 1U << 0;
inline constexpr uint16_t usart_fe   = 1U << 1;
inline constexpr uint16_t usart_ne   = 1U << 2;
inline constexpr uint16_t usart_ore  = 1U << 3;
inline constexpr uint16_t usart_idle = 1U << 4;
inline constexpr uint16_t usart_rxne = 1U << 5;
inline constexpr uint16_t usart_tc   = 1U << 6;
inline constexpr uint16_t usart_txe  = 1U << 7;

/// USART_CTLR1
inline constexpr uint16_t usart_re     = 1U << 2;
inline constexpr uint16_t usart_te     = 1U << 3;
inline constexpr uint16_t usart_txeie  = 1U << 7;
inline constexpr uint16_t usart_rxneie = 1U << 5;
inline constexpr uint16_t usart_ue     = 1U << 13;

// ---- FLASH (RM ch. 18) ----------------------------------------------------
struct FlashRegs {
    volatile uint32_t ACTLR;      ///< 0x00 LATENCY[1:0]
};

inline FlashRegs* flash_ctl() { return reinterpret_cast<FlashRegs*>(hb_base + 0x2000); }

/// RM 18.3.1: 0 waits to 15 MHz, 1 wait to 24 MHz, 2 waits to 48 MHz.
constexpr uint32_t flash_latency_for(uint32_t hz) {
    return hz <= 15'000'000UL ? 0UL : hz <= 24'000'000UL ? 1UL : 2UL;
}

// ---- the core's own two blocks (QingKe V2 manual, RM ch. 6) ---------------
struct StkRegs {
    volatile uint32_t CTLR;       ///< 0xE000F000 SWIE, STRE, STCLK, STIE, STE
    volatile uint32_t SR;         ///< 0xE000F004 CNTIF (write 0 to clear)
    volatile uint32_t CNT;        ///< 0xE000F008 the 32-bit up counter
    uint32_t RESERVED0;           ///< 0xE000F00c
    volatile uint32_t CMP;        ///< 0xE000F010 compare value
    uint32_t RESERVED1;           ///< 0xE000F014
};

inline StkRegs* stk() { return reinterpret_cast<StkRegs*>(0xE000F000UL); }

inline constexpr uint32_t stk_ste   = 1UL << 0;
inline constexpr uint32_t stk_stie  = 1UL << 1;
inline constexpr uint32_t stk_stclk = 1UL << 2;   ///< 1: HCLK, 0: HCLK/8
inline constexpr uint32_t stk_stre  = 1UL << 3;   ///< reload from 0 at CMP
inline constexpr uint32_t stk_cntif = 1UL << 0;   ///< in SR

/// The interrupt controller (QingKe V2 manual 3.1). Named here: the two
/// read-only status banks, the two write-one enable banks, and the
/// system control register that decides what a WFI does. The rest of
/// the block (priorities, the two free vectored entries, the hardware
/// stack) arrives with its first user. Mind the names: the manual's
/// ISR is the ENABLE status and its IPR the PENDING status - the
/// opposite of what the letters suggest.
struct PficRegs {
    volatile uint32_t ISR[8];     ///< 0x000 ENABLE status, read-only
    volatile uint32_t IPR[8];     ///< 0x020 PENDING status, read-only
    volatile uint32_t ITHRESDR;   ///< 0x040 priority threshold
    uint32_t RESERVED0;
    volatile uint32_t CFGR;       ///< 0x048
    volatile uint32_t GISR;       ///< 0x04c global status
    uint8_t RESERVED1[0xB0];
    volatile uint32_t IENR[8];    ///< 0x100 write 1 to ENABLE a line
    uint8_t RESERVED2[0x60];
    volatile uint32_t IRER[8];    ///< 0x180 write 1 to DISABLE a line
    uint8_t RESERVED3[0x60];
    volatile uint32_t IPSR[8];    ///< 0x200 write 1 to SET a line pending
    uint8_t RESERVED4[0x60];
    volatile uint32_t IPRR[8];    ///< 0x280 write 1 to CLEAR a pending line
    uint8_t RESERVED5[0x60];
    volatile uint32_t IACTR[8];   ///< 0x300 active (being serviced), read-only
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
/// indices (RM 6.3): the word at address 0 is an instruction, the words
/// after it are handler addresses.
enum class Irq : uint8_t {
    non_maskable   = 2,
    hard_fault     = 3,
    systick        = 12,
    software       = 14,
    wwdg           = 16,
    pvd            = 17,
    flash          = 18,
    rcc            = 19,
    exti7_0        = 20,
    awu            = 21,
    dma1_channel1  = 22,
    dma1_channel7  = 28,
    adc            = 29,
    i2c1_ev        = 30,
    i2c1_er        = 31,
    usart1         = 32,
    spi1           = 33,
    tim1_brk       = 34,
    tim1_up        = 35,
    tim1_trg_com   = 36,
    tim1_cc        = 37,
    tim2           = 38,
    usart2         = 39,   ///< CH32V005/006/007 only
    opcm           = 40,   ///< CH32V005/006/007 only
};

} // namespace brio
