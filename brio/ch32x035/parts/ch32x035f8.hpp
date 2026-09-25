/*
 * parts/ch32x035f8.hpp
 *
 * The CH32X035F8U6 as a table of facts: 62 KB of code flash, 20 KB of
 * SRAM, a QFN20 with 19 GPIO pins and the exposed pad as ground.
 *
 * THIS FILE IS THE RESERVE OF THIS SERIES. There is no vendor header to
 * ask (brio/ch32x035/device.hpp explains why), so every per-part fact a
 * driver needs is stated here, read off the CH32X035/X033 datasheet V1.7
 * - the model table for what the part offers, table 2-1's QFN20 column
 * and the pinout figure of 2.1 for the bonding. A driver reads them as
 * `device::` constants and branches with `if constexpr`; nothing else in
 * the stratum may state a part fact.
 *
 * THE QFN20, PIN BY PIN (table 2-1 and figure 2.1 agree): 0 GND (the
 * exposed pad), 1 VDD, 2 PA0, 3 PA1, 4 PA2, 5 PA3, 6 PA4, 7 PA5, 8 PA6,
 * 9 PA7, 10 PB0, 11 PB1, 12 PB3, 13 PB11, 14 PC18, 15 PB12, 16 PC19,
 * 17 PC16, 18 PC17, 19 PC14, 20 PC15. This is the one package of the
 * series whose PC16 and PC17 are NOT shorted to PC11 and PC10 inside
 * (note 4 excepts it by name), and it has no reset pin at all.
 *
 * A COUNT IS NOT A LIST. The model table counts three serial ports; the
 * pins say which: USART2 on PA2/PA3, USART3 on the debug pads PC18/PC19
 * (its remap code 1) and USART4 on PB0/PB1 (its default column) or on the
 * USB pads PC16/PC17. USART1 has no column with both its TX and its RX
 * pads bonded here - its default RX, PB11, is, and its default TX, PB10,
 * is not - so it is not offered.
 */

#pragma once

#include <stdint.h>

namespace brio::device {

/// The name this part answers to on a console banner.
inline constexpr const char* part_name = "CH32X035F8U6";
inline constexpr Package package = Package::qfn20;

// ---- the memories (datasheet model table; RM 1.2.1, table 20-1) ------------
inline constexpr uint32_t flash_bytes = 62UL * 1024UL;
inline constexpr uint32_t sram_bytes  = 20UL * 1024UL;
/// The flash's grain: 256-byte pages, the unit of the fast program and
/// erase (RM 20.2.1), write-protected in 2 KB units (table 20-1's note).
inline constexpr uint32_t flash_page_bytes    = 256UL;
inline constexpr uint32_t flash_protect_bytes = 2048UL;

// ---- the pads ---------------------------------------------------------------
/// Which pins each port bonds, as a mask (bit n = Pxn).
inline constexpr uint32_t port_pins(char port) {
    return port == 'A' ? 0x000000FFUL :   // PA0..PA7
           port == 'B' ? 0x0000180BUL :   // PB0, PB1, PB3, PB11, PB12
           port == 'C' ? 0x000FC000UL :   // PC14..PC19
           0x00000000UL;
}

inline constexpr bool has_port(char port) { return port_pins(port) != 0UL; }

/// The bonded pads that SHARE a package pin with another pad (table 2-1's
/// notes 4 to 7), which the datasheet forbids as outputs: none on this
/// package.
inline constexpr uint32_t port_twinned(char port) {
    (void)port;
    return 0x00000000UL;
}

// ---- the instances (datasheet model table) ----------------------------------
/// The USARTs this part offers, one bit per instance number, and the
/// model table's count beside it (the file header).
inline constexpr uint16_t usart_instances = (1U << 2) | (1U << 3) | (1U << 4);
inline constexpr uint8_t usart_count = 3;

inline constexpr bool has_usart(uint8_t n) {
    return n < 16U && (usart_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

inline constexpr uint8_t gpio_count        = 19;
inline constexpr uint8_t adc_channel_count = 10;   ///< external channels; one internal beside them
inline constexpr uint8_t touch_key_count   = 10;
inline constexpr uint8_t opa_count         = 2;
inline constexpr uint8_t cmp_count         = 0;
/// The full-speed USB controller: device only on this package (the model
/// table's "USB Host" column is empty for it).
inline constexpr bool has_usb_host   = false;
inline constexpr bool has_usb_device = true;
inline constexpr bool has_usbpd      = true;

} // namespace brio::device
