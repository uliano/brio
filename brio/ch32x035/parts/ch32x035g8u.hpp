/*
 * parts/ch32x035g8u.hpp
 *
 * The CH32X035G8U6 as a table of facts: 62 KB of code flash, 20 KB of
 * SRAM, a QFN28 with 27 GPIO pins and the exposed pad as ground.
 *
 * THIS FILE IS THE RESERVE OF THIS SERIES (parts/ch32x035f8.hpp says
 * why), read off the CH32X035/X033 datasheet V1.7: the model table, table
 * 2-1's QFN28 column and the pinout figure of 2.1.
 *
 * THE QFN28, PIN BY PIN: 0 GND, 1 PC15, 2 VDD, 3 PC0, 4 PC3, 5 PA0, 6 PA1,
 * 7 PA2, 8 PA3, 9 PA4, 10 PA5, 11 PA6, 12 PA7, 13 PB0, 14 PB3, 15 PB4,
 * 16 PB1 with PB5, 17 PB6, 18 PB7, 19 PB8, 20 PB9, 21 PB10, 22 PB11,
 * 23 PB12, 24 PC19, 25 PC18, 26 PC16 with PC11, 27 PC17 with PC10,
 * 28 PC14. No reset pin.
 *
 * THREE PINS CARRY TWO PADS EACH, and the datasheet forbids every one of
 * those pads as an output: PB1 with PB5 (table 2-1's note 5, this part
 * and the CH32X035G8R6), PC16 with PC11 and PC17 with PC10 (note 4).
 */

#pragma once

#include <stdint.h>

namespace brio::device {

inline constexpr const char* part_name = "CH32X035G8U6";
inline constexpr Package package = Package::qfn28;

inline constexpr uint32_t flash_bytes = 62UL * 1024UL;
inline constexpr uint32_t sram_bytes  = 20UL * 1024UL;
inline constexpr uint32_t flash_page_bytes    = 256UL;
inline constexpr uint32_t flash_protect_bytes = 2048UL;

inline constexpr uint32_t port_pins(char port) {
    return port == 'A' ? 0x000000FFUL :   // PA0..PA7
           port == 'B' ? 0x00001FFBUL :   // PB0, PB1, PB3..PB12
           port == 'C' ? 0x000FCC09UL :   // PC0, PC3, PC10, PC11, PC14..PC19
           0x00000000UL;
}

inline constexpr bool has_port(char port) { return port_pins(port) != 0UL; }

/// PB1 with PB5 (note 5), PC10 with PC17 and PC11 with PC16 (note 4).
inline constexpr uint32_t port_twinned(char port) {
    return port == 'B' ? 0x00000022UL :
           port == 'C' ? 0x00030C00UL : 0x00000000UL;
}

/// All four: USART1 on PB10/PB11, USART2 on PA2/PA3, USART3 on PB3/PB4,
/// USART4 on PB0 with its RX on PB1 - an input, which the short allows.
inline constexpr uint16_t usart_instances = (1U << 1) | (1U << 2) | (1U << 3) | (1U << 4);
inline constexpr uint8_t usart_count = 4;

inline constexpr bool has_usart(uint8_t n) {
    return n < 16U && (usart_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

inline constexpr uint8_t gpio_count        = 27;
inline constexpr uint8_t adc_channel_count = 10;
inline constexpr uint8_t touch_key_count   = 10;
inline constexpr uint8_t opa_count         = 2;
inline constexpr uint8_t cmp_count         = 1;
inline constexpr bool has_usb_host   = true;
inline constexpr bool has_usb_device = true;
inline constexpr bool has_usbpd      = true;

} // namespace brio::device
