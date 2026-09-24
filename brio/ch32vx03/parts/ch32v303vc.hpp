/*
 * parts/ch32v303vc.hpp
 *
 * The CH32V303VC as a table of facts: 256 KB of zero-wait flash out of a
 * 480 KB array, 64 KB of SRAM, LQFP100 with all eighty pins of five ports
 * bonded, the QingKe V4F core with its floating-point unit, and WCH's
 * CH32V30x_D8 device class. The rules this file obeys are
 * parts/ch32v203c8.hpp's; read that one first.
 *
 * THE THIRD DEVICE CLASS OF ONE MANUAL. The CH32V303 is described by the same
 * reference manual as the CH32V203 - one register map per chapter, with notes
 * that name the class a field belongs to - and its peripherals are the same
 * blocks where both have them; what differs is stated in these files and
 * nowhere else (the CH32V303/305/307/317 datasheet V3.5, table 2-1-1 for the
 * resources and 3.2 for the pins; the reference manual's class notes for the
 * rest). Every part of this class has, against the CH32V203: the QingKe V4F
 * core with its single-precision floating-point unit; a SECOND DMA controller
 * of eleven channels and a first one of seven, not eight; two DACs and four
 * operational amplifiers; forty-two backup registers; an HSE divided by 128
 * on the way to the RTC and by nothing else; and a vector table 104 words
 * long whose tail is not the CH32V203's at all (startup_ch32vx03.S,
 * device.hpp's Irq table). What the 256 KB parts add on top - eight serial
 * ports, SPI3 and the I2S face, TIM5..TIM10, the RNG and the SDIO host, and
 * on the LQFP100 the FSMC - is each part's own fact below.
 *
 * THE MEMORY SPLIT IS CONFIGURABLE. A user option byte divides the 480 KB
 * array into a zero-wait window and a non-zero-wait tail, trading window for
 * SRAM (the datasheet's note 1 to table 2-1-1): 192K/128K, 224K/96K,
 * 256K/64K, 288K/32K, and 128K/192K on some lots. This file and
 * ld/ch32v303vc.ld state the split a part leaves the factory in, 256K of
 * window with 64K of SRAM; a program that moves the line moves both.
 */

#pragma once

#include <stdint.h>

namespace brio::device {

/// The name this part answers to on a console banner.
inline constexpr const char* part_name = "CH32V303VC";

// ---- the memories (datasheet table 2-1-1 and its note 1) ------------------
/// flash_bytes is the ZERO-WAIT WINDOW the core executes from at full speed,
/// which is what every linker script and every flash medium of this stratum
/// reckons with.
inline constexpr uint32_t flash_bytes = 256UL * 1024UL;
inline constexpr uint32_t sram_bytes  = 64UL * 1024UL;
/// The flash's grain: 256 bytes, the unit of the fast program and erase verbs
/// (RM 32.1), with write protection in 4 KB units above it.
inline constexpr uint32_t flash_page_bytes      = 256UL;
inline constexpr uint32_t flash_protect_bytes   = 4096UL;
/// The code flash is ONE ARRAY of 480 KB (RM 32.1's table 32-1 runs to page
/// 1919), and above the window lies its NON-ZERO-WAIT TAIL - 224 KB with the
/// factory split, "(480K-R0WAIT)" in the datasheet's note 1. The linker
/// script names the tail as a region and places nothing in it: what an access
/// there costs is not measured, and fast programming does not reach it (RM
/// 32.2.1).
inline constexpr uint32_t flash_tail_bytes  = 224UL * 1024UL;
inline constexpr uint32_t flash_array_bytes = 480UL * 1024UL;

// ---- the device class and the core ----------------------------------------
/// WCH's own family division, which several chapters are keyed by: this part
/// is CH32V30x_D8 (RM, "Specific classification abbreviations"), the class of
/// the four CH32V303. A register description that names another class is
/// about other silicon - the D8C's PLL2, PLL3, USBHS, Ethernet and second CAN
/// among them - and a driver asks the facts below and names the class only
/// where the manual's note is a list of classes.
inline constexpr DeviceClass device_class = DeviceClass::v30x_d8;
/// The core: the QingKe V4F, RV32IMAFC - the V4B's instruction set plus
/// single-precision floating point (misa 0x40901125). The crt switches the
/// unit on (mstatus.FS) before anything runs, because an FP instruction with
/// the unit off traps.
inline constexpr bool has_fpu = true;
/// The vector table's length, which follows the class: 104 entries, the last
/// being DMA2's eleventh channel (startup_ch32vx03.S).
inline constexpr uint32_t vector_count = 104;

/// The backup domain's DATA registers, which the DEVICE CLASS decides:
/// forty-two here - BKP_DATAR1..BKP_DATAR10 and BKP_DATAR11..BKP_DATAR42 in a
/// second block above the tamper registers (RM 4.3's note under table 4-1
/// names this class among those that carry the second block).
inline constexpr uint8_t bkp_data_registers = 42;

/// What RTCSEL's third choice divides the HSE by on the way to the RTC (RM
/// 3.4.9): 128 on this class, with no lot rule, so the two entries are equal.
inline constexpr uint32_t rtc_hse_div[2] = {128, 128};

// ---- the DMA controllers (RM 11.2.3) --------------------------------------
/// TWO controllers on this class: DMA1 with SEVEN channels - no eighth, RM
/// 11.3.1's notes naming only other classes for it - and DMA2 with eleven,
/// which is where TIM5..TIM10, UART4..8, SPI3, the SDIO host, the DACs and
/// ADC2 put their requests (tables 11-3 and 11-4).
inline constexpr uint8_t dma_controller_count = 2;
inline constexpr uint8_t dma1_channel_count   = 7;
inline constexpr uint8_t dma2_channel_count   = 11;

// ---- the pads (datasheet 3.2, table 3-1's LQFP100 column) ------------------
/// Which pins each port bonds, as a mask. Eighty in all: five whole ports,
/// PD0 and PD1 among them as pins of their own (81 and 82), apart from the
/// oscillator's OSC_IN and OSC_OUT (12 and 13) - the datasheet's note 4.
inline constexpr uint16_t port_pins(char port) {
    return port == 'A' ? 0xFFFFU :
           port == 'B' ? 0xFFFFU :
           port == 'C' ? 0xFFFFU :
           port == 'D' ? 0xFFFFU :
           port == 'E' ? 0xFFFFU :
           0x0000U;
}

inline constexpr bool has_port(char port) { return port_pins(port) != 0U; }

/// The two pads the debug port owns from reset: a program that takes
/// them for itself loses the probe until the next power cycle.
inline constexpr char debug_swdio_port = 'A';
inline constexpr uint8_t debug_swdio_pin = 13;
inline constexpr char debug_swclk_port = 'A';
inline constexpr uint8_t debug_swclk_pin = 14;

// ---- the instances (datasheet table 2-1-1) --------------------------------
/// The serial ports this part offers, one bit per instance number: all eight
/// - USART1..3 and UART4..8 - with the default pads of every one bonded.
inline constexpr uint16_t usart_instances = (1U << 1) | (1U << 2) | (1U << 3) | (1U << 4) |
                                            (1U << 5) | (1U << 6) | (1U << 7) | (1U << 8);
inline constexpr uint8_t usart_count = 8;

inline constexpr bool has_usart(uint8_t n) {
    return n < 16U && (usart_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

/// Which of those instances are FULL USARTs - the synchronous clock on a CK
/// pad, the smartcard and the flow-control pair - rather than asynchronous
/// ports alone: USART1..3. Chapter 18's opening counts three USARTs and five
/// UARTs for the family, and the one exception it names is the CH32V203C8's.
inline constexpr uint16_t usart_full_instances = (1U << 1) | (1U << 2) | (1U << 3);

inline constexpr bool usart_full(uint8_t n) {
    return n < 16U && (usart_full_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

inline constexpr uint8_t spi_count = 3;
/// The synchronous ports as a mask, one bit per instance number, and whether
/// SPI2 and SPI3 carry their I2S face (table 2-1-1's I2S row).
inline constexpr uint16_t spi_instances = (1U << 1) | (1U << 2) | (1U << 3);
inline constexpr bool has_i2s = true;
inline constexpr uint8_t i2c_count = 2;
inline constexpr uint8_t can_count = 1;
/// The converters, and how many of the sixteen analog channels this package
/// brings out to a pad (ADC_IN0..7 are PA0..PA7, 8 and 9 are PB0 and PB1,
/// 10..15 are PC0..PC5 - datasheet 3.2).
inline constexpr uint8_t adc_count         = 2;
inline constexpr uint8_t adc_channel_count = 16;

/// The operational amplifiers, one bit per instance number: all four, each
/// with an input pair and an output on a pad this package bonds (OPA3 on
/// PB13/PB2 into PA1, OPA4 on PB12/PB1 into PA0 - datasheet 3.2), which is
/// what makes the count a list here.
inline constexpr uint16_t opa_instances = (1U << 1) | (1U << 2) | (1U << 3) | (1U << 4);
inline constexpr uint8_t opa_count = 4;

inline constexpr bool has_opa(uint8_t n) {
    return n < 16U && (opa_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

/// The timers: four advanced-control ones (TIM1, TIM8, TIM9, TIM10), four
/// general-purpose ones (TIM2..TIM5, all sixteen bits wide on this class -
/// chapter 15's note gives thirty-two to the CH32V20x_D8's TIM5 alone) and
/// the two basic ones (TIM6, TIM7), table 2-1-1's rows.
inline constexpr uint8_t advanced_timer_count = 4;
inline constexpr uint8_t general_timer_count  = 4;
inline constexpr bool has_tim5 = true;
inline constexpr uint16_t advanced_timer_instances = (1U << 1) | (1U << 8) | (1U << 9) | (1U << 10);
inline constexpr uint16_t general_timer_instances  = (1U << 2) | (1U << 3) | (1U << 4) | (1U << 5);
inline constexpr uint16_t basic_timer_instances    = (1U << 6) | (1U << 7);
/// The blocks the CH32V203 has not got: the two DACs on every CH32V303, the
/// RNG and the SDIO host on the 256 KB parts, the FSMC on the LQFP100 alone
/// (table 2-1-1). No Ethernet on this class.
inline constexpr bool has_dac  = true;
inline constexpr bool has_rng  = true;
inline constexpr bool has_sdio = true;
inline constexpr bool has_fsmc = true;
inline constexpr bool has_ethernet = false;

/// ONE USB CONTROLLER, AND IT IS NOT THE CH32V203'S. The full-speed device
/// controller of RM ch. 21 (USBD, with its packet memory) is not on this
/// class: its clock gate, RCC_APB1PCENR bit 23, does not exist - measured on
/// the CH32V303VC, where every bit of that register written one reads back
/// 0x3A7EC9FF and bit 23 alone of the gates the manual names answers zero -
/// and the datasheet's pin table gives PA11/PA12 the OTG_FS pair and no USBD
/// function. The controller that answers is the host/device one of ch. 23
/// (USBFS/OTG_FS, the HB gate's bit 12, its control register reading its
/// reset value 0x06 at 0x50000000), on PA11/PA12, with USBFS_DM/DP named on
/// PB6/PB7 too. WCH's own examples for this family drive USBFS and USBHS and
/// never a USBD.
inline constexpr bool has_usbd  = false;
inline constexpr bool has_usbfs = true;

// ---- the clock tree's edges (datasheet 4.3.5 to 4.3.7) --------------------
/// The ceiling this part is rated for, that its package brings out the
/// oscillator pads, and the crystal range its HSE takes (tables 4-10 and
/// 4-12: 3 to 25 MHz).
inline constexpr uint32_t sysclk_max_hz = 144'000'000UL;
inline constexpr bool has_hse_pins      = true;
inline constexpr uint32_t hse_min_hz    = 3'000'000UL;
inline constexpr uint32_t hse_max_hz    = 25'000'000UL;

/// The PLL's own edges (table 4-16): 3 to 25 MHz in, 18 to 144 MHz out. This
/// class has the CH32V203's simple tree (RM figure 3-3 names it) - PLL2, PLL3
/// and PREDIV1 are the D8C's.
inline constexpr uint32_t pll_in_min_hz  = 3'000'000UL;
inline constexpr uint32_t pll_in_max_hz  = 25'000'000UL;
inline constexpr uint32_t pll_out_min_hz = 18'000'000UL;
inline constexpr uint32_t pll_out_max_hz = 144'000'000UL;

/// What PLLXTPRE selects on the way from the HSE into the PLL, by code (RM
/// 3.4.2): the whole clock or half of it, as on the CH32V20x_D6.
inline constexpr uint8_t pll_hse_div[2] = {1, 2};

/// Whether USBPRE's fourth code (the PLL divided by five) exists here: it
/// does not - RM 3.4.2 gives it to the CH32V20x_D8 and the D8W alone, so the
/// USB's 48 MHz comes from a PLL at 48, 96 or 144 MHz.
inline constexpr bool has_usb_pre_div5 = false;

/// The LSI as the datasheet measures it (table 4-15): the 40 kHz RC, 25 to
/// 60 kHz across parts and conditions.
inline constexpr uint32_t lsi_min_hz = 25'000UL;
inline constexpr uint32_t lsi_typ_hz = 39'000UL;
inline constexpr uint32_t lsi_max_hz = 60'000UL;

} // namespace brio::device
