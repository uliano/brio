// vx03_pair.hpp - the desk pair's two links as the part's facts choose
// them: ONE rule each for the serial link and the SPI link, read by the
// peers (vx03_peer.hpp includes this file) and by the suites that drive
// them (vx03_link_dut.hpp, test_vx03_spi, bench_vx03), so the two ends of a
// link are chosen by one statement. App-level bench tooling; nothing in
// brio knows it exists. Constants only: no function here forms code.

#pragma once

#include <stdint.h>

#include "ch32vx03/device.hpp"
#include "ch32vx03/spi.hpp"
#include "ch32vx03/usart.hpp"

namespace vx03pair {

/// THE PAIR'S SERIAL LINK, one rule for the peer and the DUT
/// (vx03_link_dut.hpp): UART4 on its SECOND column (TX PB0, RX PB1, RM
/// table 10-26) where the fourth port is a plain UART - TX and RX, no clock
/// pad, so no single-board letter wants it - and this part has that column:
/// the CH32V303RC and VC, and the CH32V203RB. USART3 on its default column (TX PB10, RX PB11,
/// table 10-25) everywhere else: the CH32V203C8, whose fourth port is the
/// full USART4 its clocked letters own, and the CH32V303CB and RB, which
/// have no fourth port. A part that has not even USART3 is no end of the
/// pair (vx03_link_dut.hpp's `present`).
inline constexpr bool link_on_fourth =
    brio::device::has_usart(4) && !brio::device::usart_full(4) && brio::usart_remap_valid(4, 1);
inline constexpr uint8_t link_port = link_on_fourth ? 4 : 3;
inline constexpr uint8_t link_column = link_on_fourth ? 1 : 0;

/// THE PAIR'S SPI LINK, one rule for spi_peer and the suites that host the
/// far client: SPI3 on its SECOND column (NSS PA4, SCK PC10, MISO PC11,
/// MOSI PC12) where the part has SPI3 - the CH32V303RC and VC, whose SPI3
/// default column is SPI1's second one -, SPI2 on its one column (NSS PB12,
/// SCK PB13, MISO PB14, MOSI PB15) where it has SPI2 and no SPI3 - the
/// CH32V203C8 and RB, the CH32V303CB and RB. Zero where the part has SPI1
/// alone: no end of the pair.
inline constexpr uint8_t spi_link_instance = brio::spi_present(3) ? 3 : brio::spi_present(2) ? 2 : 0;
inline constexpr uint8_t spi_link_column = spi_link_instance == 3 ? 1 : 0;

/// Whether a pad is one of the SPI link's four (none where the part has no
/// SPI link).
inline constexpr bool on_spi_link(brio::Pad pad) {
    if (spi_link_instance == 0u) {
        return false;
    }
    const brio::SpiPins p = brio::spi_pins_for(spi_link_instance, spi_link_column);
    return pad == p.nss || pad == p.sck || pad == p.miso || pad == p.mosi;
}

/// THE SINGLE-BOARD CROSSED PAIR (USART2 against the fourth port's DEFAULT
/// column, a suite's own straps) IS RETIRED where that column lands on the
/// pair's SPI link: on the CH32V303RC and VC, UART4's PC10/PC11 (RM table
/// 10-26) are SPI3's second column's SCK and MISO. False where the part has
/// no fourth port.
inline constexpr bool fourth_default_on_spi_link =
    brio::device::has_usart(4) && (on_spi_link(brio::usart_pads_for(4, 0).tx) ||
                                   on_spi_link(brio::usart_pads_for(4, 0).rx));

}  // namespace vx03pair
