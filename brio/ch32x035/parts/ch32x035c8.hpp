/*
 * parts/ch32x035c8.hpp
 *
 * The CH32X035C8T6 as a table of facts: 62 KB of code flash, 20 KB of
 * SRAM, an LQFP48 with 46 GPIO pins.
 *
 * THIS FILE IS THE RESERVE OF THIS SERIES (parts/ch32x035f8.hpp says
 * why), read off the CH32X035/X033 datasheet V1.7: the model table, table
 * 2-1's LQFP48 column and the pinout figure of 2.1.
 *
 * THE LQFP48 bonds all of port A (PA15..PA23 on pins 1..9, PA0..PA7 on
 * 10..17, PA8..PA14 on 40..46), PB0..PB13, PC6, PC7 and PC14..PC19 - with
 * PC16 and PC11 on pin 32 and PC17 and PC10 on pin 33, the shorts of table
 * 2-1's note 4, which the datasheet forbids as outputs. PA21 (pin 7)
 * carries the reset function on this package.
 */

#pragma once

#include <stdint.h>

namespace brio::device {

inline constexpr const char* part_name = "CH32X035C8T6";
inline constexpr Package package = Package::lqfp48;

inline constexpr uint32_t flash_bytes = 62UL * 1024UL;
inline constexpr uint32_t sram_bytes  = 20UL * 1024UL;
inline constexpr uint32_t flash_page_bytes    = 256UL;
inline constexpr uint32_t flash_protect_bytes = 2048UL;

inline constexpr uint32_t port_pins(char port) {
    return port == 'A' ? 0x00FFFFFFUL :   // PA0..PA23
           port == 'B' ? 0x00003FFFUL :   // PB0..PB13
           port == 'C' ? 0x000FCCC0UL :   // PC6, PC7, PC10, PC11, PC14..PC19
           0x00000000UL;
}

inline constexpr bool has_port(char port) { return port_pins(port) != 0UL; }

/// PC10 with PC17 and PC11 with PC16 (note 4).
inline constexpr uint32_t port_twinned(char port) {
    return port == 'C' ? 0x00030C00UL : 0x00000000UL;
}

inline constexpr uint16_t usart_instances = (1U << 1) | (1U << 2) | (1U << 3) | (1U << 4);
inline constexpr uint8_t usart_count = 4;

inline constexpr bool has_usart(uint8_t n) {
    return n < 16U && (usart_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

inline constexpr uint8_t gpio_count        = 46;
inline constexpr uint8_t adc_channel_count = 10;
inline constexpr uint8_t touch_key_count   = 10;
inline constexpr uint8_t opa_count         = 2;
inline constexpr uint8_t cmp_count         = 3;
inline constexpr bool has_usb_host   = true;
inline constexpr bool has_usb_device = true;
inline constexpr bool has_usbpd      = true;

} // namespace brio::device
