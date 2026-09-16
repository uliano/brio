/*
 * parts/ch32v203c8.hpp
 *
 * The CH32V203C8 as a table of facts: 64 KB of flash, 20 KB of SRAM,
 * LQFP48 with 37 bonded pins, and WCH's CH32V20x_D6 device class.
 *
 * THIS FILE IS THE RESERVE OF THIS FAMILY. There is no vendor header to
 * ask (brio/ch32v203/device.hpp explains why), so every per-part fact a
 * driver needs is stated here, read off the CH32V203 datasheet V2.8
 * (table 2-1 for the resources, the pin tables for the bonding) and the
 * reference manual where a chapter's applicability is per class. A
 * driver reads them as `device::` constants and branches with
 * `if constexpr`; nothing else in the stratum may state a part fact.
 *
 * WHAT A SIBLING PART WOULD CHANGE. The datasheet's table 2-1 is nine
 * columns wide and the C8 is one of them: the F6 has ONE usart and no
 * I2C, the parts below the C8 have two usarts and one SPI, and the RB is
 * the other device class - 128 KB and 64 KB of RAM, a 32-bit TIM5, a
 * 10M Ethernet, a vector table with a different tail and an HSE that is
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
/// part is CH32V20x_D6. The register descriptions that name D8, D8C or
/// D8W are about other silicon and are not implemented here.
inline constexpr bool is_d8_class = false;
/// The vector table's length, which follows the class: 63 entries, the
/// last being the eighth DMA channel (startup_ch32v203.S).
inline constexpr uint32_t vector_count = 63;

// ---- the pads (datasheet, the LQFP48 pin table) ---------------------------
/// Which pins each port bonds, as a mask. Thirty-seven in all: two whole
/// ports, three pins of port C and the two of port D that the package
/// brings out (the same two pads the oscillator uses, which is why they
/// are pins 5 and 6 of this package and the datasheet's note 4 is about
/// them). Port E is not bonded on any part of this package.
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
inline constexpr uint8_t usart_count = 4;   ///< USART1..3 and UART4
inline constexpr uint8_t spi_count   = 2;
inline constexpr uint8_t i2c_count   = 2;
inline constexpr uint8_t can_count   = 1;
inline constexpr uint8_t adc_count   = 2;
inline constexpr uint8_t opa_count   = 2;
/// One advanced-control timer (TIM1) and three general-purpose ones
/// (TIM2..4); the 32-bit TIM5 is the RB's alone.
inline constexpr uint8_t advanced_timer_count = 1;
inline constexpr uint8_t general_timer_count  = 3;
inline constexpr bool has_tim5 = false;
inline constexpr bool has_ethernet = false;

/// Both USB blocks are on the die: the full-speed DEVICE controller
/// (USBD, RM ch. 21) on PA11/PA12, and the host/device one (USBFS, ch.
/// 23) on PB6/PB7. Which of them a BOARD wires to a connector is the
/// board's business, not the part's.
inline constexpr bool has_usbd  = true;
inline constexpr bool has_usbfs = true;

// ---- the clock tree's edges (datasheet chapter 2) -------------------------
/// The ceiling this part is rated for, and the crystal range its HSE
/// oscillator takes. The RB's HSE is 32 MHz and nothing else, which is
/// why this is a part fact and not a family one.
inline constexpr uint32_t sysclk_max_hz = 144'000'000UL;
inline constexpr uint32_t hse_min_hz    = 3'000'000UL;
inline constexpr uint32_t hse_max_hz    = 25'000'000UL;

} // namespace brio::device
