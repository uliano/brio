/*
 * dma_engine.hpp
 *
 * The "no DMA engine" default of the two optional Uart engine slots,
 * the other families' tag verbatim (samc21/sercom.hpp is where its
 * reasoning is written): `present` is the only thing a transport asks
 * of an engine, with `if constexpr`, so every engine branch disappears
 * from a Uart that names none. The real engines live in rp2040/dma.hpp
 * (twelve channels, two interrupt lines, 2.5), so that a program with a
 * serial port and no DMA never sees the block.
 *
 * The REQUEST NUMBERS live here too (table 119): on this chip a
 * request is a field any channel takes, so a transport tells its
 * engine which peripheral request paces it - and the transport must be
 * able to name that request without including the block.
 */

#pragma once

#include <stdint.h>

namespace brio {

struct NoDmaEngine {
    NoDmaEngine() = delete;
    static constexpr bool present = false;
};

/// Two engines on one transport must not name the same DMA channel.
template <typename Tx, typename Rx>
constexpr bool uart_engines_distinct() {
    if constexpr (Tx::present && Rx::present) {
        return Tx::channel != Rx::channel;
    } else {
        return true;
    }
}

/// CTRL.TREQ_SEL: what paces a channel - a peripheral's data request
/// (table 119), a pacing timer, or nothing (`permanent`: as fast as the
/// bus allows, a memory-to-memory transfer's choice).
enum class Dreq : uint8_t {
    pio0_tx0 = 0, pio0_tx1 = 1, pio0_tx2 = 2, pio0_tx3 = 3,
    pio0_rx0 = 4, pio0_rx1 = 5, pio0_rx2 = 6, pio0_rx3 = 7,
    pio1_tx0 = 8, pio1_tx1 = 9, pio1_tx2 = 10, pio1_tx3 = 11,
    pio1_rx0 = 12, pio1_rx1 = 13, pio1_rx2 = 14, pio1_rx3 = 15,
    spi0_tx = 16, spi0_rx = 17, spi1_tx = 18, spi1_rx = 19,
    uart0_tx = 20, uart0_rx = 21, uart1_tx = 22, uart1_rx = 23,
    pwm_wrap0 = 24, pwm_wrap1 = 25, pwm_wrap2 = 26, pwm_wrap3 = 27,
    pwm_wrap4 = 28, pwm_wrap5 = 29, pwm_wrap6 = 30, pwm_wrap7 = 31,
    i2c0_tx = 32, i2c0_rx = 33, i2c1_tx = 34, i2c1_rx = 35,
    adc = 36, xip_stream = 37, xip_ssitx = 38, xip_ssirx = 39,
    timer0 = 59, timer1 = 60, timer2 = 61, timer3 = 62,
    permanent = 63,
};

} // namespace brio
