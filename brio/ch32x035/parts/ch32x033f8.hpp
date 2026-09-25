/*
 * parts/ch32x033f8.hpp
 *
 * The CH32X033F8P6 as a table of facts: 62 KB of code flash, 20 KB of
 * SRAM, a TSSOP20 with 18 GPIO pins - the series' one CH32X033, with no
 * USB PD and no USB host.
 *
 * THIS FILE IS THE RESERVE OF THIS SERIES (parts/ch32x035f8.hpp says
 * why), read off the CH32X035/X033 datasheet V1.7: the model table (whose
 * CH32X033F8P6 row gives two operational amplifiers and two comparators,
 * an empty PD column and an empty USB host column), table 2-2 - this
 * part's own pin table - and the pinout figure of 2.1.
 *
 * THE TSSOP20, PIN BY PIN: 1 PA6, 2 PA7 with PB0, 3 PB1, 4 PB7 (which
 * carries the reset function), 5 PC16 with PC11, 6 PC17 with PC10, 7 GND,
 * 8 PC18, 9 VDD, 10 PA9, 11 PA11, 12 PA10, 13 PC3, 14 PA0, 15 PA1, 16 PA2,
 * 17 PA3, 18 PC19, 19 PA4, 20 PA5.
 *
 * THREE PINS CARRY TWO PADS EACH, and the datasheet forbids every one of
 * those pads as an output: PA7 with PB0 (table 2-2's note 7), PC16 with
 * PC11 and PC17 with PC10 (note 4).
 */

#pragma once

#include <stdint.h>

namespace brio::device {

inline constexpr const char* part_name = "CH32X033F8P6";
inline constexpr Package package = Package::tssop20;

inline constexpr uint32_t flash_bytes = 62UL * 1024UL;
inline constexpr uint32_t sram_bytes  = 20UL * 1024UL;
inline constexpr uint32_t flash_page_bytes    = 256UL;
inline constexpr uint32_t flash_protect_bytes = 2048UL;

inline constexpr uint32_t port_pins(char port) {
    return port == 'A' ? 0x00000EFFUL :   // PA0..PA7, PA9, PA10, PA11
           port == 'B' ? 0x00000083UL :   // PB0, PB1, PB7
           port == 'C' ? 0x000F0C08UL :   // PC3, PC10, PC11, PC16..PC19
           0x00000000UL;
}

inline constexpr bool has_port(char port) { return port_pins(port) != 0UL; }

/// PA7 with PB0 (note 7), PC10 with PC17 and PC11 with PC16 (note 4).
inline constexpr uint32_t port_twinned(char port) {
    return port == 'A' ? 0x00000080UL :
           port == 'B' ? 0x00000001UL :
           port == 'C' ? 0x00030C00UL : 0x00000000UL;
}

/// All four: USART1 on PA10/PA11 (its code 1), USART2 on PA2/PA3, USART3
/// on the debug pads PC18/PC19 (its code 1), USART4 on PA5/PA9 (its code
/// 1) - its default TX, PB0, being one of the forbidden pads here.
inline constexpr uint16_t usart_instances = (1U << 1) | (1U << 2) | (1U << 3) | (1U << 4);
inline constexpr uint8_t usart_count = 4;

inline constexpr bool has_usart(uint8_t n) {
    return n < 16U && (usart_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

inline constexpr uint8_t gpio_count        = 18;
inline constexpr uint8_t adc_channel_count = 10;
inline constexpr uint8_t touch_key_count   = 10;
inline constexpr uint8_t opa_count         = 2;
inline constexpr uint8_t cmp_count         = 2;
inline constexpr bool has_usb_host   = false;
inline constexpr bool has_usb_device = true;
inline constexpr bool has_usbpd      = false;

} // namespace brio::device
