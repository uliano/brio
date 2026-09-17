/*
 * parts/ch32v203rb.hpp
 *
 * The CH32V203RB as a table of facts: 128 KB of flash, 64 KB of SRAM,
 * LQFP64M with 49 bonded pins, and WCH's CH32V20x_D8 device class - the
 * one part of this series that is not D6. The rules this file obeys are
 * parts/ch32v203c8.hpp's; read that one first.
 *
 * THE OTHER DEVICE CLASS, AND WHAT FOLLOWS FROM IT. The reference
 * manual covers four families at once and keys many of its register
 * descriptions to a class; every other part of this series is
 * CH32V20x_D6 and this one is CH32V20x_D8 (RM, "Specific
 * classification abbreviations"), which is the classification the
 * manual derives from flash size alone - D6 is 32 or 64 KB, D8 is 128
 * or 256. Three things follow that no other part here has: a 32-bit
 * general-purpose timer (TIM5), a 10M Ethernet with its own PHY, and a
 * VECTOR TABLE WITH A DIFFERENT TAIL - seventy entries against the D6's
 * sixty-three, with the Ethernet pair, TIM5, UART4, the eighth DMA
 * channel and the 32 kHz oscillator's two lines after the USBFS pair.
 * The manual's own table 9-2 is the union of this family with the
 * CH32V30x and states neither tail, so the crt states the per-class
 * truth from the part definition (startup_ch32v203.S) and
 * device.hpp's Irq enum reads `is_d8_class` for the two numbers that
 * move.
 *
 * ITS CRYSTAL IS 32 MHz AND NOTHING ELSE (datasheet tables 4-9 and
 * 4-11, both with "applied for V203RBT6" beside the figure), where
 * every other part takes 3 to 25 - and it needs no external load
 * capacitors, which are built in.
 *
 * THE MEMORY SPLIT IS CONFIGURABLE. The datasheet's note 2 offers three
 * divisions of the same array - 128K flash with 64K SRAM, 144K with
 * 48K, 160K with 32K - chosen by a word in the configuration memory.
 * The first is what this file and ld/ch32v203rb.ld state, because it is
 * the one a part leaves the factory in; a program that moves the line
 * moves both.
 */

#pragma once

#include <stdint.h>

namespace brio::device {

/// The name this part answers to on a console banner.
inline constexpr const char* part_name = "CH32V203RB";

// ---- the memories (datasheet table 2-1 and its note 2) --------------------
inline constexpr uint32_t flash_bytes = 128UL * 1024UL;
inline constexpr uint32_t sram_bytes  = 64UL * 1024UL;
/// The flash's grain: 256 bytes, the unit of the fast program and erase
/// verbs (RM 32.1), with write protection in 4 KB units above it.
inline constexpr uint32_t flash_page_bytes      = 256UL;
inline constexpr uint32_t flash_protect_bytes   = 4096UL;

// ---- the device class -----------------------------------------------------
/// WCH's own family division, which several chapters are keyed by: this
/// part is CH32V20x_D8, the only one of this series (RM, "Specific
/// classification abbreviations"). The register descriptions that name
/// D6, D8C or D8W are about other silicon.
inline constexpr bool is_d8_class = true;
/// The vector table's length, which follows the class: 70 entries, the
/// last being the 32 kHz oscillator's wake-up (startup_ch32v203.S).
inline constexpr uint32_t vector_count = 70;

// ---- the pads (datasheet 3.2, table 3-1-4) --------------------------------
/// Which pins each port bonds, as a mask. Three whole ports and one pin
/// of port D: PD2 is pin 54, and PD0/PD1 do NOT exist on this part -
/// where every other package can hand its two oscillator pads back to
/// port D, "CH32V203RBT6 chip only has OSC_IN and OSC_OUT function
/// pins, which cannot be reused as PD0 and PD1" (the datasheet's note
/// 4). Port E is not bonded on any part of this series.
inline constexpr uint16_t port_pins(char port) {
    return port == 'A' ? 0xFFFFU :
           port == 'B' ? 0xFFFFU :
           port == 'C' ? 0xFFFFU :
           port == 'D' ? 0x0004U :   // PD2
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

inline constexpr uint8_t spi_count = 2;
inline constexpr uint8_t i2c_count = 2;
inline constexpr uint8_t can_count = 1;
/// ONE converter, not the two every other part of the series carries,
/// with all sixteen analog channels on a pad: ADC_IN0..7 are PA0..PA7,
/// 8 and 9 are PB0 and PB1, 10..15 are PC0..PC5 (datasheet 3.3, and
/// table 2-1's "16@1" against the others' "10@2").
inline constexpr uint8_t adc_count         = 1;
inline constexpr uint8_t adc_channel_count = 16;

/// The operational amplifiers, one bit per instance number: both, each
/// with an input pair and an output on a bonded pad.
inline constexpr uint16_t opa_instances = (1U << 1) | (1U << 2);
inline constexpr uint8_t opa_count = 2;

inline constexpr bool has_opa(uint8_t n) {
    return n < 16U && (opa_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

/// One advanced-control timer (TIM1), three general-purpose 16-bit ones
/// (TIM2..4) and - this part alone - the 32-bit TIM5.
inline constexpr uint8_t advanced_timer_count = 1;
inline constexpr uint8_t general_timer_count  = 3;
inline constexpr bool has_tim5 = true;
inline constexpr bool has_ethernet = true;

/// Both USB blocks reach a pad here: the full-speed DEVICE controller
/// (USBD, RM ch. 21) on PA11/PA12, and the host/device one (USBFS, ch.
/// 23) on PB6/PB7.
inline constexpr bool has_usbd  = true;
inline constexpr bool has_usbfs = true;

// ---- the clock tree's edges (datasheet 2.1 and tables 4-9, 4-11) ----------
/// The ceiling this part is rated for, that its package brings out the
/// oscillator pads, and the ONE crystal rate its HSE takes - 32 MHz,
/// stated as a range of one so that every arithmetic written against
/// the family keeps working (the file header).
inline constexpr uint32_t sysclk_max_hz = 144'000'000UL;
inline constexpr bool has_hse_pins      = true;
inline constexpr uint32_t hse_min_hz    = 32'000'000UL;
inline constexpr uint32_t hse_max_hz    = 32'000'000UL;

} // namespace brio::device
