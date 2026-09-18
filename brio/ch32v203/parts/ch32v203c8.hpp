/*
 * parts/ch32v203c8.hpp
 *
 * The CH32V203C8 as a table of facts: 64 KB of flash, 20 KB of SRAM,
 * LQFP48 with 37 bonded pins, and WCH's CH32V20x_D6 device class.
 *
 * THIS FILE IS THE RESERVE OF THIS FAMILY. There is no vendor header to
 * ask (brio/ch32v203/device.hpp explains why), so every per-part fact a
 * driver needs is stated here, read off the CH32V203 datasheet V2.8
 * (table 2-1 for the resources, the pin tables of 3.2 for the bonding)
 * and the reference manual where a chapter's applicability is per
 * class. A driver reads them as `device::` constants and branches with
 * `if constexpr`; nothing else in the stratum may state a part fact.
 *
 * A COUNT IS NOT A LIST. Table 2-1 counts what a part OFFERS, and the
 * manual says why that is not the same as what the die carries: "the
 * number of some peripherals and function of the same series product
 * may be limited by the package" (RM, the note under the series
 * overview). On this package the two readings agree - four USARTs
 * counted, four with their pads bonded - but they do not on the
 * smallest parts, so the blocks a driver names by NUMBER carry a MASK
 * here and the count is stated beside it. parts/ch32v203f6.hpp is
 * where that matters.
 *
 * WHAT A SIBLING PART WOULD CHANGE. The datasheet's table 2-1 is nine
 * columns wide and the C8 is one of them: the F6 offers one USART and
 * no I2C, the parts below the C8 offer two USARTs and one SPI, the F8
 * and the G8 have no oscillator pins at all, and the RB is the other
 * device class - 128 KB and 64 KB of RAM, a 32-bit TIM5, a 10M
 * Ethernet, a vector table with a different tail and an HSE that is
 * 32 MHz rather than the 3..25 MHz of every part here. Each of those is
 * its own file beside this one, and no driver may guess one from
 * another.
 */

#pragma once

#include <stdint.h>

namespace brio::device {

/// The name this part answers to on a console banner.
inline constexpr const char* part_name = "CH32V203C8";

// ---- the memories (datasheet table 2-1) -----------------------------------
inline constexpr uint32_t flash_bytes = 64UL * 1024UL;
inline constexpr uint32_t sram_bytes  = 20UL * 1024UL;
/// The flash's grain: 256 bytes, the unit of the fast program and erase
/// verbs (RM 32.1), with write protection in 4 KB units above it.
inline constexpr uint32_t flash_page_bytes      = 256UL;
inline constexpr uint32_t flash_protect_bytes   = 4096UL;

// ---- the device class -----------------------------------------------------
/// WCH's own family division, which several chapters are keyed by: this
/// part is CH32V20x_D6 (RM, "Specific classification abbreviations").
/// The register descriptions that name D8, D8C or D8W are about other
/// silicon and are not implemented here.
inline constexpr bool is_d8_class = false;
/// The vector table's length, which follows the class: 63 entries, the
/// last being the eighth DMA channel (startup_ch32v203.S).
inline constexpr uint32_t vector_count = 63;

/// The backup domain's DATA registers, which the DEVICE CLASS decides:
/// ten here - BKP_DATAR1..BKP_DATAR10, twenty bytes - where the
/// CH32V20x_D8 carries forty-two (RM 4.3's note under table 4-1). The
/// block's other registers (the calibration, the tamper pair) are every
/// class's.
inline constexpr uint8_t bkp_data_registers = 10;

/// What RTCSEL's third choice divides the HSE by on the way to the RTC
/// (RM 3.4.9). On this device class it is ONE OF TWO NUMBERS and the
/// register description keys them on the LOT: 512 where the penultimate
/// fifth digit of the lot number is less than one, 128 otherwise. Both
/// are stated because neither is knowable from the part number, and a
/// program that clocks the RTC from the crystal measures which of them
/// this die has; where the manual states a single value, the two
/// entries are equal.
inline constexpr uint32_t rtc_hse_div[2] = {512, 128};

// ---- the pads (datasheet 3.2, table 3-1-1's LQFP48 column) ----------------
/// Which pins each port bonds, as a mask. Thirty-seven in all: two whole
/// ports, three pins of port C and the two of port D that the package
/// brings out (the same two pads the oscillator uses, which is why they
/// are pins 5 and 6 of this package and the datasheet's note 4 is about
/// them). Port E is not bonded on any part of this series.
inline constexpr uint16_t port_pins(char port) {
    return port == 'A' ? 0xFFFFU :
           port == 'B' ? 0xFFFFU :
           port == 'C' ? 0xE000U :   // PC13, PC14, PC15
           port == 'D' ? 0x0003U :   // PD0, PD1
           0x0000U;
}

inline constexpr bool has_port(char port) { return port_pins(port) != 0U; }

/// The two pads the debug port owns from reset: a program that takes
/// them for itself loses the probe until the next power cycle.
inline constexpr char debug_swdio_port = 'A';
inline constexpr uint8_t debug_swdio_pin = 13;
inline constexpr char debug_swclk_port = 'A';
inline constexpr uint8_t debug_swclk_pin = 14;

// ---- the instances (datasheet table 2-1) ----------------------------------
/// The USARTs this part offers, one bit per instance number: all four,
/// and this package bonds the default pads of every one of them.
inline constexpr uint16_t usart_instances = (1U << 1) | (1U << 2) | (1U << 3) | (1U << 4);
inline constexpr uint8_t usart_count = 4;

inline constexpr bool has_usart(uint8_t n) {
    return n < 16U && (usart_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

/// Which of those instances are FULL USARTs - the synchronous clock on
/// a CK pad, the smartcard and the flow-control pair - rather than
/// asynchronous receivers alone. ALL FOUR ARE, ON THIS PART ALONE:
/// chapter 18's opening counts three USARTs and five UARTs for the
/// family and then names the exception, "for CH32V203C8 the serial
/// port 4 is a universal synchronous asynchronous receiver
/// transmitter", and the datasheet's pin table agrees - USART4_CK,
/// USART4_CTS and USART4_RTS are bonded here where the other class
/// brings out a UART4 with TX and RX alone.
inline constexpr uint16_t usart_full_instances = usart_instances;

inline constexpr bool usart_full(uint8_t n) {
    return n < 16U && (usart_full_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

inline constexpr uint8_t spi_count = 2;
inline constexpr uint8_t i2c_count = 2;
inline constexpr uint8_t can_count = 1;
/// The converters, and how many of the sixteen analog channels this
/// package brings out to a pad (ADC_IN0..7 are PA0..PA7, 8 and 9 are
/// PB0 and PB1, 10..15 are PC0..PC5 - datasheet 3.3).
inline constexpr uint8_t adc_count         = 2;
inline constexpr uint8_t adc_channel_count = 10;

/// The operational amplifiers, one bit per instance number: both, each
/// with an input pair and an output on a bonded pad.
inline constexpr uint16_t opa_instances = (1U << 1) | (1U << 2);
inline constexpr uint8_t opa_count = 2;

inline constexpr bool has_opa(uint8_t n) {
    return n < 16U && (opa_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

/// One advanced-control timer (TIM1) and three general-purpose ones
/// (TIM2..4); the 32-bit TIM5 is the CH32V203RB's alone.
inline constexpr uint8_t advanced_timer_count = 1;
inline constexpr uint8_t general_timer_count  = 3;
inline constexpr bool has_tim5 = false;
inline constexpr bool has_ethernet = false;

/// Both USB blocks reach a pad here: the full-speed DEVICE controller
/// (USBD, RM ch. 21) on PA11/PA12, and the host/device one (USBFS, ch.
/// 23) on PB6/PB7. Which of them a BOARD wires to a connector is the
/// board's business, not the part's.
inline constexpr bool has_usbd  = true;
inline constexpr bool has_usbfs = true;

// ---- the clock tree's edges (datasheet 2.1 and tables 4-9, 4-11) ----------
/// The ceiling this part is rated for, whether its package brings out
/// the oscillator pads at all, and the crystal range its HSE takes. The
/// RB's HSE is 32 MHz and nothing else, and two packages of this series
/// have no OSC pin, which is why all three are part facts.
inline constexpr uint32_t sysclk_max_hz = 144'000'000UL;
inline constexpr bool has_hse_pins      = true;
inline constexpr uint32_t hse_min_hz    = 3'000'000UL;
inline constexpr uint32_t hse_max_hz    = 25'000'000UL;

/// The PLL's own edges (datasheet table 4-15): what its input may be,
/// and what its output may be. Both are checked at compile time, and
/// they are NOT the same numbers on every part of the series.
inline constexpr uint32_t pll_in_min_hz  = 3'000'000UL;
inline constexpr uint32_t pll_in_max_hz  = 25'000'000UL;
inline constexpr uint32_t pll_out_min_hz = 18'000'000UL;
inline constexpr uint32_t pll_out_max_hz = 144'000'000UL;

/// What PLLXTPRE selects on the way from the HSE into the PLL, by code
/// (RM 3.4.2): the whole clock or half of it on this class, where the
/// CH32V203RB divides its 32 MHz oscillator by four or by eight.
inline constexpr uint8_t pll_hse_div[2] = {1, 2};

/// Whether USBPRE's fourth code (the PLL divided by five, from a PLL at
/// 240 MHz) exists here: it is the CH32V20x_D8's and the D8W's, and
/// even there it depends on the lot number (RM 3.4.2).
inline constexpr bool has_usb_pre_div5 = false;

/// The LSI as the datasheet measures it (table 4-14), not as the clock
/// tree's block diagram rounds it: a wide RC, which is why anything
/// timed by it is measured rather than computed.
inline constexpr uint32_t lsi_min_hz = 25'000UL;
inline constexpr uint32_t lsi_typ_hz = 39'000UL;
inline constexpr uint32_t lsi_max_hz = 60'000UL;

} // namespace brio::device
