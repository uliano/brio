/*
 * dma_engine.hpp
 *
 * The "no DMA engine" tag of every optional engine slot in this stratum
 * (the USART's, the SPI host's and the I2C host's) and nothing else: a
 * driver with an engine slot must not include dma.hpp, or every program
 * with a console would carry the controller; an application that wants
 * an engine includes both headers and names the channel, one that does
 * not never sees the controller at all. `present` is the only thing a
 * task asks about, with `if constexpr`, so every engine branch
 * disappears from a driver that does not name one - and an engine
 * reaches the driver only through its own published names (start,
 * service, block_flags, the flag names), never the controller's.
 */

#pragma once

#include <stdint.h>

namespace brio {

struct NoDmaEngine {
    NoDmaEngine() = delete;
    static constexpr bool present = false;
};

/// The channel an engine sits on - 0 for the empty slot, which has
/// none - so a transport can check table 8-2 against a named engine
/// without asking the tag for a member it has not got.
template <typename E>
constexpr uint8_t dma_engine_channel() {
    if constexpr (E::present) {
        return E::channel;
    } else {
        return 0;
    }
}

/// Two engines on one transport must not name the same channel: a
/// channel moves data ONE way. Generic over any engine that says
/// `present` and `channel`.
template <typename Tx, typename Rx>
constexpr bool dma_engines_distinct() {
    if constexpr (Tx::present && Rx::present) {
        return Tx::channel != Rx::channel;
    } else {
        return true;
    }
}

} // namespace brio
