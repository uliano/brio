/*
 * parts/ch32x035f7.hpp
 *
 * The CH32X035F7P6 as a table of facts: 62 KB of code flash, 20 KB of
 * SRAM, a TSSOP20 with 18 GPIO pins.
 *
 * THIS FILE IS THE RESERVE OF THIS SERIES (parts/ch32x035f8.hpp says
 * why), read off the CH32X035/X033 datasheet V1.7: the model table, table
 * 2-1's TSSOP20 column and the pinout figure of 2.1.
 *
 * THE TSSOP20, PIN BY PIN: 1 PB12, 2 PC14, 3 PC15, 4 PC3 (which carries
 * the reset function on this package), 5 PC1, 6 PA0, 7 PA1, 8 PA2, 9 PA3,
 * 10 PA4, 11 PA5, 12 PA6, 13 PA7, 14 PB1, 15 GND, 16 VDD, 17 PC16 with
 * PC11, 18 PC17 with PC10, 19 PC18, 20 PC19.
 *
 * TWO PINS CARRY TWO PADS EACH. Table 2-1's note 4: on every part but the
 * CH32X035F8U6, PC17 and PC10 are shorted inside the chip, and so are
 * PC16 and PC11, and "both IOs are prohibited from being configured as
 * output functions" - so the four are inputs here, which pin.hpp enforces.
 *
 * A COUNT THE PINS DO NOT REACH. The model table counts three serial
 * ports for this part. The pins bring out USART2 on PA2/PA3 and USART3 on
 * the debug pads PC18/PC19; USART4's only columns with both pads bonded
 * put its TX on PC16 or PC17, which note 4 forbids as outputs, and USART1
 * has no bonded RX. This file follows note 4: two instances are offered,
 * and the count stands beside them as the table gives it
 * (docs/ch32x035/usart.md names the disagreement).
 */

#pragma once

#include <stdint.h>

namespace brio::device {

inline constexpr const char* part_name = "CH32X035F7P6";
inline constexpr Package package = Package::tssop20;

inline constexpr uint32_t flash_bytes = 62UL * 1024UL;
inline constexpr uint32_t sram_bytes  = 20UL * 1024UL;
inline constexpr uint32_t flash_page_bytes    = 256UL;
inline constexpr uint32_t flash_protect_bytes = 2048UL;

inline constexpr uint32_t port_pins(char port) {
    return port == 'A' ? 0x000000FFUL :   // PA0..PA7
           port == 'B' ? 0x00001002UL :   // PB1, PB12
           port == 'C' ? 0x000FCC0AUL :   // PC1, PC3, PC10, PC11, PC14..PC19
           0x00000000UL;
}

inline constexpr bool has_port(char port) { return port_pins(port) != 0UL; }

/// PC10 with PC17 and PC11 with PC16 (note 4).
inline constexpr uint32_t port_twinned(char port) {
    return port == 'C' ? 0x00030C00UL : 0x00000000UL;
}

inline constexpr uint16_t usart_instances = (1U << 2) | (1U << 3);
inline constexpr uint8_t usart_count = 3;   ///< the model table's; see the file header

inline constexpr bool has_usart(uint8_t n) {
    return n < 16U && (usart_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

inline constexpr uint8_t gpio_count        = 18;
inline constexpr uint8_t adc_channel_count = 11;
inline constexpr uint8_t touch_key_count   = 11;
inline constexpr uint8_t opa_count         = 1;
inline constexpr uint8_t cmp_count         = 1;
inline constexpr bool has_usb_host   = false;
inline constexpr bool has_usb_device = true;
inline constexpr bool has_usbpd      = true;

} // namespace brio::device
