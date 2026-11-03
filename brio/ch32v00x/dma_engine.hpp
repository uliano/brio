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
 * Who owns a channel. One owner a channel for the whole image
 * (docs/design/dma.md, "The ownership rule"), checked when the program is
 * compiled by util/dma_owner.hpp, the channel's key `DmaSlotId<1, ch>`
 * (this family's one controller is the manual's DMA1). The claim is made
 * by whatever programs the channel, at the moment it does: an engine's
 * `arm()` (or `arm_ring()`) claims the channel for the owner it is handed,
 * and a `DmaChannel<ch>` verb that programs the channel claims it for that
 * channel's owner. A transport hands its engines its request,
 * `DmaOwner<DmaPeriph::usart, 2, DmaSignal::tx>` for USART2's transmit; a
 * program that arms an engine itself, or programs a channel by hand,
 * claims it for the tags below unless it names the request it serves. So a
 * channel two of them would program - the console's transmit engine and a
 * copy on channel 4, USART2's and the I2C host's engines on channel 6 - is
 * a compile error naming the channel and both owners. On this family each
 * request is wired to one channel (table 8-2) and every transport refuses an
 * engine on any other, so the claim's other half, one channel a request,
 * fires only on a program that names one request on two channels by hand.
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

#include "util/dma_owner.hpp"

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

/// The owners of a channel no request is named for (the file header), one
/// tag a channel and a kind of user: `DmaChannel<ch>`'s verbs that program
/// it, a `DmaCopyEngine` on it, a transmit or a receive engine a program
/// arms itself. Two of them on one channel are two owners, refused.
template <uint8_t ch> struct DmaHandOwner {};
template <uint8_t ch> struct DmaCopyOwner {};
template <uint8_t ch> struct DmaTxOwner {};
template <uint8_t ch> struct DmaRxOwner {};

} // namespace brio
