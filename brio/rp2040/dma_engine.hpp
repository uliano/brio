/*
 * dma_engine.hpp
 *
 * The "no DMA engine" default of the two optional Uart engine slots,
 * the other families' tag verbatim (samc21/sercom.hpp is where its
 * reasoning is written): `present` is the only thing a transport asks
 * of an engine, with `if constexpr`, so every engine branch disappears
 * from a Uart that names none. The real engines arrive with this
 * stratum's dma.hpp (twelve channels, two interrupt lines, 2.5), and
 * live THERE so that a program with a serial port and no DMA never
 * sees the block.
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

} // namespace brio
