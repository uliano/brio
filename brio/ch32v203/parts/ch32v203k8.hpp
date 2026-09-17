/*
 * parts/ch32v203k8.hpp
 *
 * The CH32V203K8 as a table of facts: 64 KB of flash, 20 KB of SRAM,
 * LQFP32 with 26 bonded pins, and WCH's CH32V20x_D6 device class. The
 * rules this file obeys are parts/ch32v203c8.hpp's; read that one
 * first.
 *
 * WHAT THIS PACKAGE DROPS. Port A is whole, port B keeps eight pins
 * (PB0, PB1 and PB3..PB8) and port C no longer reaches a pad at all -
 * so there is no VBAT pin, no TAMPER pad and no 32 kHz crystal on this
 * part, which is what a backup-domain or RTC chapter must ask before it
 * offers an LSE. PB2 is not bonded either and is pulled to ground
 * inside (datasheet note 5), so BOOT1 is fixed here.
 *
 * WHAT IT KEEPS AND THE DATASHEET STILL REFUSES. PB6 and PB7 are
 * brought out, which on the C8 is where the USBFS host/device
 * controller lives - and table 2-1 gives this part no USBHD. So USBFS
 * is absent from the die and not merely off the pads: the pinout
 * figures agree, naming those two pins plain PB6/PB7 here and
 * PB6/USB2DM, PB7/USB2DP on the parts that have the block.
 */

#pragma once

#include <stdint.h>

namespace brio::device {

/// The name this part answers to on a console banner.
inline constexpr const char* part_name = "CH32V203K8";

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

// ---- the pads (datasheet 3.2, table 3-1-1's LQFP32 column) ----------------
/// Which pins each port bonds, as a mask. Twenty-six in all: port A
/// whole, eight pins of port B, and the two of port D the package brings
/// out as the oscillator's pads (pins 2 and 3 here - the datasheet's
/// note 4). No pin of port C and none of port E.
inline constexpr uint16_t port_pins(char port) {
    return port == 'A' ? 0xFFFFU :
           port == 'B' ? 0x01FBU :   // PB0, PB1, PB3..PB8
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
/// The USARTs this part offers, one bit per instance number: two, and
/// they are USART1 (PA9/PA10) and USART2 (PA2/PA3), the two whose pads
/// this package bonds.
inline constexpr uint16_t usart_instances = (1U << 1) | (1U << 2);
inline constexpr uint8_t usart_count = 2;

inline constexpr bool has_usart(uint8_t n) {
    return n < 16U && (usart_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

inline constexpr uint8_t spi_count = 1;
inline constexpr uint8_t i2c_count = 1;
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

/// The device controller (USBD, RM ch. 21) is here on PA11/PA12; the
/// host/device one (USBFS, ch. 23) is not on this part at all - see the
/// file header.
inline constexpr bool has_usbd  = true;
inline constexpr bool has_usbfs = false;

// ---- the clock tree's edges (datasheet 2.1 and tables 4-9, 4-11) ----------
/// The ceiling this part is rated for, whether its package brings out
/// the oscillator pads at all, and the crystal range its HSE takes.
inline constexpr uint32_t sysclk_max_hz = 144'000'000UL;
inline constexpr bool has_hse_pins      = true;
inline constexpr uint32_t hse_min_hz    = 3'000'000UL;
inline constexpr uint32_t hse_max_hz    = 25'000'000UL;

} // namespace brio::device
