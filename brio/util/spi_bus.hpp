/*
 * spi_bus.hpp
 *
 * The SPI vocabulary over util/bus_master.hpp: SpiBus IS BusMaster, the
 * SPI reply is a BusDone, spi_ok/spi_rejected are the arbiter's codes.
 * Zero-cost aliases, kept so that a display client reads "SpiDone" and
 * not "BusDone" - the names say which wire the bytes took. Everything
 * about arbitration, the pending FIFO, replies and the engine contract
 * is documented once, in bus_master.hpp; the SPI engine descriptor and
 * its two completion styles live in each target's SpiHost and in
 * docs/design/spi-bus.md.
 *
 * SPI has no wire-level failure the engine can detect (no ACK, no
 * arbitration): what a host can report is its own - a frame it lost,
 * a wait it gave up on, a DMA block it could not finish - so the codes
 * below the arbiter's are the host's, and the two every host can
 * answer are spelled here once (spi_overrun, spi_stalled) while the
 * engines' spi_dma_fault stays with the host that has engines.
 */

#pragma once

#include <stdint.h>

#include "kernel/platform.hpp"
#include "util/bus_master.hpp"

namespace brio {

using SpiDone = BusDone;

inline constexpr uint8_t spi_ok = bus_ok;
inline constexpr uint8_t spi_rejected = bus_rejected;
/// The transaction never answered inside the arbiter's per-bus timeout:
/// the engine was recover()ed and the requester is told here. On SPI
/// the plausible wedge is not a wire (the host clocks itself) but a
/// dead engine - a host demoted mid-transfer, a DMA channel stopped by
/// a controller erratum - and the ISR-style completion that therefore
/// never posts.
inline constexpr uint8_t spi_timeout = bus_timeout;

// ---- the host's own codes, in the range bus_master.hpp leaves to engines ----
//
// bus_engine_status + 0 is `spi_dma_fault`, defined by every host that
// has DMA engine slots and by none that has not: a block the engines
// could not finish (a transfer error, a block that never completed).
// The two below are every host's, one spelling and one value on every
// family, so a client written over the vocabulary alone (a panel driver
// over any family's host) can tell a lost frame from a dead block
// without naming a family.

/// The host lost a frame: the receive side was overrun - a frame landed
/// while the previous one still stood unread (one register deep on the
/// F1 lineage's blocks, a FIFO's depth elsewhere), which with frames
/// written ahead means the handler, or the polled loop under a long
/// handler, came later than the receive side could hold. Every frame
/// went out; one that came back is missing, so the transaction ends
/// with this code instead of handing back a run with a hole in it.
inline constexpr uint8_t spi_overrun = bus_engine_status + 1u;
/// A polled wait ran out: a flag the loop waited for never rose within
/// the transaction's one spin budget - a block whose clock does not run
/// (a gate closed, the enable down, a mode fault that demoted it). The
/// select is released and the receive side drained; what else the host
/// does to the block (a reset and a reconfiguration, or nothing but
/// the arbiter's recover() as the way back) is the family's.
inline constexpr uint8_t spi_stalled = bus_engine_status + 2u;

/// `timeout_ticks` is PER BUS; size it to the longest legal transaction
/// (a polled request completes inside start() and never arms it - only
/// ISR-style completions are on this clock). ticks_from_ms<P>() converts.
template <typename Bus, Platform P, uint8_t pending_depth = 4,
          typename Policy = BusPassThrough, uint32_t timeout_ticks = 0>
using SpiBus = BusMaster<Bus, P, pending_depth, Policy, timeout_ticks>;

} // namespace brio
