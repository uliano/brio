/*
 * spi_bus.hpp
 *
 * The SPI names of the shared bus arbiter: what a program names when
 * several active objects share one SPI bus. SpiBus is util/
 * bus_master.hpp's BusMaster, the reply is a BusDone and spi_ok and
 * spi_rejected are the arbiter's codes, all zero-cost aliases, kept so
 * that a display client reads "SpiDone" and not "BusDone": the names
 * say which wire the bytes took. A program names one arbiter per bus
 * over the family's SpiHost and posts the host's Request to it with a
 * reply channel inside (kernel/post.hpp's reply_to):
 *
 *     using Host = brio::SpiHost<1, host_pins>;   // the family's engine (stm32g0/spi.hpp's spelling)
 *     using Bus = brio::SpiBus<Host, P>;
 *     brio::post<Bus>(Host::Request{..., .reply = brio::reply_to<MyAo, brio::SpiDone>()});
 *
 * (A designated initializer follows its own host's declaration: the
 * fields are the same on every stratum for an 8-bit request, their order
 * is not, so a program written for several strata assigns them by name.)
 *
 * Everything about arbitration, the pending FIFO, replies, who owns the
 * request's buffers until when and the engine contract is documented
 * once, in bus_master.hpp; the SPI engine descriptor and its two ways to
 * run a request live in each target's SpiHost and in
 * docs/design/spi-bus.md. Polled is an explicit loop at any length: no
 * DMA and no interrupt vector, and start() returns when it is done.
 * Asynchronous is DMA: the host's engines carry the request and their
 * completion is the TransferDone, a host with no engines refuses it
 * (spi_rejected at once, nothing on the wire; the host type's `has_dma`
 * says so at compile time), and a request too short to repay the engines'
 * fixed cost runs the same loop inside start().
 *
 * SPI has no wire-level failure the engine can detect (no acknowledge,
 * no arbitration): what a host can report is its own (a wait it gave up
 * on, a DMA block it could not finish), so the codes past the arbiter's
 * are the host's: spi_stalled is spelled here once, spi_dma_fault with
 * each host that defines it, and each host's header says which it
 * answers.
 */

#pragma once

#include <stdint.h>

#include "kernel/platform.hpp"
#include "util/bus_master.hpp"

namespace brio {

using SpiDone = BusDone;

inline constexpr uint8_t spi_ok = bus_ok;
/// The request never reached the wire: the arbiter's pending FIFO was full,
/// or the host refused it - an asynchronous request on a host with no DMA
/// engines (docs/design/spi-bus.md, "Two ways to run a request").
inline constexpr uint8_t spi_rejected = bus_rejected;
/// The transaction never answered inside the arbiter's per-bus timeout:
/// the engine was recover()ed and the requester is told here. On SPI
/// the plausible wedge is not a wire (the host clocks itself) but a
/// dead engine - a host demoted mid-transfer, a DMA channel stopped by
/// a controller erratum - and the interrupt's completion that therefore
/// never posts.
inline constexpr uint8_t spi_timeout = bus_timeout;

// ---- the host's own codes, in the range bus_master.hpp leaves to engines ----
//
// bus_engine_status + 0 is `spi_dma_fault`, defined by every stratum whose
// host has DMA engine slots and by none that has not: the engines reported
// a block they could not finish. Only the engines answer it, so only an
// asynchronous request that rode them can carry it; a block that simply
// never completes is the arbiter's spi_timeout, on a bus with a timeout.
// The code below has one spelling and one value on every family, distinct
// from that one.

/// A polled wait ran out: a flag the loop waited for never rose within
/// the transaction's one spin budget - a block whose clock does not run
/// (a gate closed, the enable down, a mode fault that demoted it). Every
/// host whose loop has such a budget answers it, whether the request
/// asked for the loop or was short enough to run it; the AVR's has none,
/// a transfer written there always completing. The select is released
/// and the receive side drained; what else the host does to the block (a
/// reset and a reconfiguration, or nothing) is the family's. On the
/// PL022 hosts it does nothing else: the way back is recover(), the
/// program's to call with the bus idle.
inline constexpr uint8_t spi_stalled = bus_engine_status + 2u;

/// The arbiter under SPI's name. `pending_depth` is how many requests
/// may wait while one is in flight (a further one is answered
/// spi_rejected at once); `timeout_ticks` is per bus and in kernel
/// ticks: size it to the longest legal transaction (a request that
/// completes inside start() never arms it; only the asynchronous ones'
/// completions are on this clock), 0 for no timeout at all.
/// ticks_from_ms<P>() (kernel/time.hpp) converts.
template <typename Bus, Platform P, uint8_t pending_depth = 4,
          typename Policy = BusPassThrough, uint32_t timeout_ticks = 0>
using SpiBus = BusMaster<Bus, P, pending_depth, Policy, timeout_ticks>;

} // namespace brio
