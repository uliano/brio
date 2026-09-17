/*
 * parts/ch32v203f6.hpp
 *
 * The CH32V203F6 as a table of facts: 32 KB of flash, 10 KB of SRAM,
 * TSSOP20 with 16 bonded pins, and WCH's CH32V20x_D6 device class. The
 * smallest part of the series, and the one the build's LINK GUARD uses.
 * The rules this file obeys are parts/ch32v203c8.hpp's; read that one
 * first.
 *
 * THE PART THAT PROVES A COUNT IS NOT A LIST. Table 2-1 gives this part
 * ONE usart, and it is not USART1: the package bonds neither PA9/PA10
 * (USART1's default pair) nor PB6/PB7 (its remapped one), while PA2 and
 * PA3 - USART2's default pair - are pins 8 and 9. The count is the
 * datasheet's and the instance behind it is the pin table's, which is
 * why `usart_instances` is a mask and not a number: a driver that read
 * "one usart" as "USART1" would refuse the only serial port this part
 * has and offer the one it has not got.
 *
 * NO I2C EITHER, for the same reason: I2C1 lives on PB6/PB7 and neither
 * is bonded, so table 2-1's column reads 0. What IS here is the pair
 * PA11/PA12, which carries both the USB device controller's D-/D+ and
 * CAN1's RX/TX - the two blocks this package does bring out.
 *
 * BOOT0 SHARES PB8 (the datasheet's note 6): the two are sealed
 * together, which is why PB8 counts as bonded and why the vendor
 * recommends a pull-down on it.
 */

#pragma once

#include <stdint.h>

namespace brio::device {

/// The name this part answers to on a console banner.
inline constexpr const char* part_name = "CH32V203F6";

// ---- the memories (datasheet table 2-1) -----------------------------------
inline constexpr uint32_t flash_bytes = 32UL * 1024UL;
inline constexpr uint32_t sram_bytes  = 10UL * 1024UL;
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

// ---- the pads (datasheet 3.2, table 3-1-3's TSSOP20 column) --------------
/// Which pins each port bonds, as a mask. Sixteen in all, and no pin
/// carries two of them on this package: PA0..PA7 and PA11..PA14 of port
/// A, PB1 and PB8 of port B, and the two of port D the package brings
/// out as the oscillator's pads (pins 2 and 3 - the datasheet's note
/// 4). No pin of port C and none of port E. PA8, PA9, PA10 and PA15 are
/// absent, and so is all of port B but those two.
inline constexpr uint16_t port_pins(char port) {
    return port == 'A' ? 0x78FFU :   // PA0..PA7, PA11..PA14
           port == 'B' ? 0x0102U :   // PB1, PB8
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
/// The one USART this part offers, and it is USART2 - see the file
/// header for why the number matters more than the count here.
inline constexpr uint16_t usart_instances = (1U << 2);
inline constexpr uint8_t usart_count = 1;

inline constexpr bool has_usart(uint8_t n) {
    return n < 16U && (usart_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

inline constexpr uint8_t spi_count = 1;
inline constexpr uint8_t i2c_count = 0;
inline constexpr uint8_t can_count = 1;
/// The converters, and how many of the sixteen analog channels this
/// package brings out to a pad: nine, PA0..PA7 being ADC_IN0..7 and PB1
/// being ADC_IN9 (PB0, which is channel 8, is not bonded here).
inline constexpr uint8_t adc_count         = 2;
inline constexpr uint8_t adc_channel_count = 9;

/// The one operational amplifier this part offers, and - as with the
/// usart - the instance is not the first: OPA2's inputs (PA5, PA7) and
/// outputs (PA2, PA4) are all bonded, while OPA1's positive inputs are
/// PB0 and PB15, neither of which this package brings out.
inline constexpr uint16_t opa_instances = (1U << 2);
inline constexpr uint8_t opa_count = 1;

inline constexpr bool has_opa(uint8_t n) {
    return n < 16U && (opa_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

/// One advanced-control timer (TIM1) and three general-purpose ones
/// (TIM2..4); the 32-bit TIM5 is the CH32V203RB's alone. The datasheet
/// marks both counts with its note 3 on this package: the blocks are
/// there, and how many of their channels reach a pin is the pin table's
/// answer.
inline constexpr uint8_t advanced_timer_count = 1;
inline constexpr uint8_t general_timer_count  = 3;
inline constexpr bool has_tim5 = false;
inline constexpr bool has_ethernet = false;

/// The device controller (USBD, RM ch. 21) is here on PA11/PA12; the
/// host/device one (USBFS, ch. 23) lives on PB6/PB7, which this package
/// does not bond.
inline constexpr bool has_usbd  = true;
inline constexpr bool has_usbfs = false;

// ---- the clock tree's edges (datasheet 2.1 and tables 4-9, 4-11) ----------
/// The ceiling this part is rated for, whether its package brings out
/// the oscillator pads at all, and the crystal range its HSE takes.
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
