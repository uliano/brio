/*
 * dma_engine.hpp
 *
 * The "no DMA engine" tag of every optional engine slot in this stratum
 * (the USART's, the SPI host's and the I2C host's) and the priority the
 * engines in those slots arm at, and nothing else: a
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

/**
 * CFGR.PL: the software half of the arbitration, the channel index
 * deciding between equal levels - the lower number wins (RM 8.2.1). It
 * lives here, beside the tag, because a transport names the level of the
 * engines in its slots and must not include the controller to do it.
 *
 * THE LEVEL IS THE PERIPHERAL'S (docs/design/dma.md). An engine whose
 * peripheral OVERRUNS when its channel is starved - a receive ring, an SPI
 * host's receive - arms at `very_high`; one whose peripheral only waits or
 * holds its last value - a transmit, a paced output, an I2C host's receive
 * under its stretched clock - at `high`; a memory-to-memory copy at `low`,
 * because no request paces it and it asks for the bus without a pause, so
 * a channel it outranks moves nothing until its block is over. The
 * engines' defaults are those three by direction (ch32v00x/dma.hpp), and
 * every driver arms its engines at its own level, with its chapter's
 * reason beside the call.
 */
enum class DmaPriority : uint8_t { low = 0, medium = 1, high = 2, very_high = 3 };

} // namespace brio
