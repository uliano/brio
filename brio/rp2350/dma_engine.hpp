/*
 * dma_engine.hpp
 *
 * The "no DMA engine" default of the two optional Uart engine slots, the
 * other families' tag verbatim (samc21/sercom.hpp is where its reasoning
 * is written): `present` is the only thing a transport asks of an engine,
 * with `if constexpr`, so every engine branch disappears from a Uart that
 * names none. The real engines live in rp2350/dma.hpp (sixteen channels,
 * four interrupt lines, 12.6), so that a program with a serial port and
 * no DMA never sees the block.
 *
 * The REQUEST NUMBERS live here too (datasheet 12.6.4.1, the system DREQ
 * table): on this chip a request is a field any channel takes, so a
 * transport tells its engine which peripheral request paces it - and the
 * transport must be able to name that request without including the
 * controller.
 *
 * THE TABLE IS NOT THE RP2040'S, and not one row of it is. This chip has
 * a third PIO and twelve PWM slices against eight, so every number above
 * the first PIO's has moved: the UARTs sit at 28..31 where they sat at
 * 20..23, the I2Cs at 44..47 where they sat at 32..35, and the entries
 * for the HSTX, the CoreSight trace and the SHA-256 accelerator have no
 * twin on the other chip at all. The SSI's two rows are the QMI's here,
 * under that name. Only the four pacing timers and the "always" row keep
 * their places at 59..63, at the top of the six-bit field.
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
/// (12.6.4.1), a pacing timer, or nothing (`permanent`: as fast as the
/// bus allows, a memory-to-memory transfer's choice).
enum class Dreq : uint8_t {
    pio0_tx0 = 0, pio0_tx1 = 1, pio0_tx2 = 2, pio0_tx3 = 3,
    pio0_rx0 = 4, pio0_rx1 = 5, pio0_rx2 = 6, pio0_rx3 = 7,
    pio1_tx0 = 8, pio1_tx1 = 9, pio1_tx2 = 10, pio1_tx3 = 11,
    pio1_rx0 = 12, pio1_rx1 = 13, pio1_rx2 = 14, pio1_rx3 = 15,
    pio2_tx0 = 16, pio2_tx1 = 17, pio2_tx2 = 18, pio2_tx3 = 19,
    pio2_rx0 = 20, pio2_rx1 = 21, pio2_rx2 = 22, pio2_rx3 = 23,
    spi0_tx = 24, spi0_rx = 25, spi1_tx = 26, spi1_rx = 27,
    uart0_tx = 28, uart0_rx = 29, uart1_tx = 30, uart1_rx = 31,
    pwm_wrap0 = 32, pwm_wrap1 = 33, pwm_wrap2 = 34, pwm_wrap3 = 35,
    pwm_wrap4 = 36, pwm_wrap5 = 37, pwm_wrap6 = 38, pwm_wrap7 = 39,
    pwm_wrap8 = 40, pwm_wrap9 = 41, pwm_wrap10 = 42, pwm_wrap11 = 43,
    i2c0_tx = 44, i2c0_rx = 45, i2c1_tx = 46, i2c1_rx = 47,
    adc = 48, xip_stream = 49, xip_qmitx = 50, xip_qmirx = 51,
    hstx = 52, coresight = 53, sha256 = 54,
    timer0 = 59, timer1 = 60, timer2 = 61, timer3 = 62,
    permanent = 63,
};

} // namespace brio
