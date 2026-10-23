/*
 * dma_engine.hpp
 *
 * The vocabulary a transport needs to have an optional DMA engine slot,
 * and nothing else: the `NoDmaEngine` tag that fills an empty slot, the
 * two compile-time checks a transport makes on the engines it is given,
 * and `DmaPriority`, the level an engine arms at. The slots are the
 * USART's, the SPI host's and the I2C host's (an engine, in brio, is the
 * driver piece that runs one DMA channel for a transport; a task is a
 * driver layer with state of its own, served from its peripheral's
 * interrupt, such as `Uart` or `SpiHost`).
 *
 * A program meets this header only through a transport's template
 * arguments. Leaving a slot at its default is the smallest use:
 *
 *     using Serial = brio::Uart<1, P>;               // both slots NoDmaEngine
 *
 * and naming an engine means including ch32v00x/dma.hpp as well and
 * naming the channel RM table 8-2 gives the request
 * (docs/ch32v00x/dma.md, "How to use it"):
 *
 *     using Serial = brio::Uart<1, P, 64, 128, brio::DmaTxEngine<4>, brio::DmaRxEngine<5>>;
 *
 * Nothing here runs at run time: every name is a type, a constant or a
 * constexpr function, so there is no context to respect.
 *
 * Why a header of its own: a driver with an engine slot must not include
 * dma.hpp, or every program with a console would carry the controller;
 * an application that wants an engine includes both headers and names
 * the channel, one that does not never sees the controller at all.
 * `present` is the only thing a task asks about, with `if constexpr`, so
 * every engine branch disappears from a driver that does not name one -
 * and an engine reaches the driver only through its own published names
 * (start, service, block_flags, the flag names), never the controller's.
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
/// channel moves data one way. Generic over any engine that says
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
 * The level is the peripheral's (docs/design/dma.md). An engine whose
 * peripheral overruns when its channel is starved - a receive ring, an SPI
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
