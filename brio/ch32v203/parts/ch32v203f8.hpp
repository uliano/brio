/*
 * parts/ch32v203f8.hpp
 *
 * The CH32V203F8 as a table of facts: 64 KB of flash, 20 KB of SRAM,
 * TSSOP20 with 17 bonded pins, and WCH's CH32V20x_D6 device class. The
 * rules this file obeys are parts/ch32v203c8.hpp's; read that one
 * first.
 *
 * THIS FILE IS THE TSSOP20, AND THE PART HAS TWO PACKAGES. Table 2-1
 * splits the F8's column in two - CH32V203F8P6 in TSSOP20 and
 * CH32V203F8U6 in QFN20 - and the two do not offer the same blocks,
 * because they do not bond the same pads: the TSSOP20 brings out
 * PB6/PB7 (I2C1 and the USBFS host/device controller) and not
 * PA11/PA12, while the QFN20 does the opposite and offers the USB
 * device controller instead. The datasheet's own headline numbers for
 * the F8 - 17 GPIO pins, nine ADC channels - are the TSSOP20's, and so
 * is everything below. A board with the QFN20 part would want a file of
 * its own beside this one rather than a fact bent to cover both.
 *
 * NO OSCILLATOR PADS. Neither package brings out OSC_IN or OSC_OUT (the
 * pin table 3-1-2 has no row for them), so there is no HSE here in
 * either form and no port D either, PD0 and PD1 being the second
 * function of those two pads. The PLL runs from the HSI and nothing
 * else; `has_hse_pins` is what says so and clock.hpp refuses the rest.
 *
 * TWO PINS CARRY TWO PORT BITS EACH. The datasheet's note 7 warns that
 * 20- and 28-pin packages short I/O pins together and that driving both
 * halves as outputs may damage them; here pin 1 is PA13 + PB6 and pin 2
 * is PA14 + PB7 - which is to say the debug port and the I2C bus are
 * the same two pins.
 */

#pragma once

#include <stdint.h>

namespace brio::device {

/// The name this part answers to on a console banner.
inline constexpr const char* part_name = "CH32V203F8";

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

// ---- the pads (datasheet 3.2, table 3-1-2's TSSOP20 column) --------------
/// Which pins each port bonds, as a mask - nineteen port bits over
/// seventeen pins, two pins carrying two bits each (the file header).
/// Port A keeps PA0..PA10 and PA13/PA14; port B keeps PB0, PB6, PB7 and
/// PB13..PB15. Ports C and D reach no pad on this package.
inline constexpr uint16_t port_pins(char port) {
    return port == 'A' ? 0x67FFU :   // PA0..PA10, PA13, PA14
           port == 'B' ? 0xE0C1U :   // PB0, PB6, PB7, PB13..PB15
           0x0000U;
}

inline constexpr bool has_port(char port) { return port_pins(port) != 0U; }

/// The two pads the debug port owns from reset: a program that takes
/// them for itself loses the probe until the next power cycle. On this
/// package they are also the I2C bus's two pins (the file header).
inline constexpr char debug_swdio_port = 'A';
inline constexpr uint8_t debug_swdio_pin = 13;
inline constexpr char debug_swclk_port = 'A';
inline constexpr uint8_t debug_swclk_pin = 14;

// ---- the instances (datasheet table 2-1) ----------------------------------
/// The USARTs this part offers, one bit per instance number: two, and
/// they are USART1 (PA9/PA10, or PB6/PB7 remapped - both pairs are
/// bonded) and USART2 (PA2/PA3).
inline constexpr uint16_t usart_instances = (1U << 1) | (1U << 2);
inline constexpr uint8_t usart_count = 2;

inline constexpr bool has_usart(uint8_t n) {
    return n < 16U && (usart_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

/// Which of those instances are FULL USARTs - the synchronous clock on
/// a CK pad, the smartcard and the flow-control pair - rather than
/// asynchronous receivers alone (RM 18, its opening note). Every
/// instance this part offers is one: the family's UART4 is the only
/// serial port that is not, and this package has not got it.
inline constexpr uint16_t usart_full_instances = usart_instances;

inline constexpr bool usart_full(uint8_t n) {
    return n < 16U && (usart_full_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

inline constexpr uint8_t spi_count = 1;
inline constexpr uint8_t i2c_count = 1;
/// No CAN on either package of this part: CAN1's pads are PA11/PA12,
/// and the TSSOP20 bonds neither.
inline constexpr uint8_t can_count = 0;
/// The converters, and how many of the sixteen analog channels this
/// package brings out to a pad: nine, PA0..PA7 being ADC_IN0..7 and PB0
/// being ADC_IN8 (PB1, which is channel 9, is not bonded here).
inline constexpr uint8_t adc_count         = 2;
inline constexpr uint8_t adc_channel_count = 9;

/// The operational amplifiers, one bit per instance number: both, each
/// with an input pair and an output on a bonded pad.
inline constexpr uint16_t opa_instances = (1U << 1) | (1U << 2);
inline constexpr uint8_t opa_count = 2;

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

/// The host/device controller (USBFS, RM ch. 23) is here on PB6/PB7,
/// which the pinout figure names UDM and UDP; the device controller
/// (USBD, ch. 21) lives on PA11/PA12, which this package does not bond
/// - on the QFN20 variant the two swap round (the file header).
inline constexpr bool has_usbd  = false;
inline constexpr bool has_usbfs = true;

// ---- the clock tree's edges (datasheet 2.1 and tables 4-9, 4-11) ----------
/// The ceiling this part is rated for, whether its package brings out
/// the oscillator pads at all - it does not, see the file header - and
/// the crystal range the family's HSE takes, which is stated so that a
/// program asking for one is refused for the right reason.
inline constexpr uint32_t sysclk_max_hz = 144'000'000UL;
inline constexpr bool has_hse_pins      = false;
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
