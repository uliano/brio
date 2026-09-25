/*
 * parts/ch32x035g8r.hpp
 *
 * The CH32X035G8R6 as a table of facts: 62 KB of code flash, 20 KB of
 * SRAM, a QSOP28 with 26 GPIO pins.
 *
 * THIS FILE IS THE RESERVE OF THIS SERIES (parts/ch32x035f8.hpp says
 * why), read off the CH32X035/X033 datasheet V1.7: the model table, table
 * 2-1's QSOP28 column and the pinout figure of 2.1. The same number as
 * the CH32X035G8U6 and NOT the same bonding, which is why the two keys
 * carry the package's letter.
 *
 * THE QSOP28, PIN BY PIN: 1 PC19, 2 PC16 with PC11, 3 PC17 with PC10,
 * 4 PC14 with PA12, 5 PC15 with PA13, 6 VDD, 7 GND, 8 PC3 (which carries
 * the reset function on this package), 9 PA0, 10 PA1, 11 PA2, 12 PA3,
 * 13 PA6, 14 PB0, 15 PA4, 16 PA5, 17 PA7, 18 PB3, 19 PB4, 20 PB1 with
 * PB5, 21 PB6, 22 PB7, 23 PB8, 24 PB9, 25 PB10, 26 PB11, 27 PB12,
 * 28 PC18.
 *
 * FIVE PINS CARRY TWO PADS EACH, and the datasheet forbids every one of
 * those pads as an output: PB1 with PB5 (note 5), PA12 with PC14 and PA13
 * with PC15 (note 6, this part alone), PC16 with PC11 and PC17 with PC10
 * (note 4).
 */

#pragma once

#include <stdint.h>

namespace brio::device {

inline constexpr const char* part_name = "CH32X035G8R6";
inline constexpr Package package = Package::qsop28;

inline constexpr uint32_t flash_bytes = 62UL * 1024UL;
inline constexpr uint32_t sram_bytes  = 20UL * 1024UL;
inline constexpr uint32_t flash_page_bytes    = 256UL;
inline constexpr uint32_t flash_protect_bytes = 2048UL;

inline constexpr uint32_t port_pins(char port) {
    return port == 'A' ? 0x000030FFUL :   // PA0..PA7, PA12, PA13
           port == 'B' ? 0x00001FFBUL :   // PB0, PB1, PB3..PB12
           port == 'C' ? 0x000FCC08UL :   // PC3, PC10, PC11, PC14..PC19
           0x00000000UL;
}

inline constexpr bool has_port(char port) { return port_pins(port) != 0UL; }

/// PA12, PA13 (note 6), PB1 with PB5 (note 5), PC10, PC11, PC14..PC17
/// (notes 4 and 6).
inline constexpr uint32_t port_twinned(char port) {
    return port == 'A' ? 0x00003000UL :
           port == 'B' ? 0x00000022UL :
           port == 'C' ? 0x0003CC00UL : 0x00000000UL;
}

/// All four: USART1 on PB10/PB11, USART2 on PA2/PA3, USART3 on PB3/PB4,
/// USART4 on PB0 with its RX on PB1 - an input, which the short allows.
inline constexpr uint16_t usart_instances = (1U << 1) | (1U << 2) | (1U << 3) | (1U << 4);
inline constexpr uint8_t usart_count = 4;

inline constexpr bool has_usart(uint8_t n) {
    return n < 16U && (usart_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

inline constexpr uint8_t gpio_count        = 26;
inline constexpr uint8_t adc_channel_count = 11;
inline constexpr uint8_t touch_key_count   = 11;
inline constexpr uint8_t opa_count         = 2;
inline constexpr uint8_t cmp_count         = 3;
inline constexpr bool has_usb_host   = true;
inline constexpr bool has_usb_device = true;
inline constexpr bool has_usbpd      = true;

} // namespace brio::device
