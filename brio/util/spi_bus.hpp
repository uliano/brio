/*
 * spi_bus.hpp
 *
 * The SPI vocabulary over util/bus_master.hpp: SpiBus IS BusMaster, the
 * SPI reply is a BusDone, spi_ok/spi_rejected are the arbiter's codes.
 * Zero-cost aliases, kept so that a display client reads "SpiDone" and
 * not "BusDone" - the names say which wire the bytes took. Everything
 * about arbitration, the pending FIFO, replies and the engine contract
 * is documented once, in bus_master.hpp; the SPI engine descriptor and
 * its two completion styles live in avrdx/spi.hpp and
 * docs/design/spi-bus.md.
 *
 * SPI has no wire-level failure the engine can detect (no ACK, no
 * arbitration), so its status vocabulary is exactly the arbiter's.
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
/// dead engine - a demoted AVR host mid-transfer, a DMA channel the
/// 1.10.4 class of death stopped - and the ISR-style completion that
/// therefore never posts.
inline constexpr uint8_t spi_timeout = bus_timeout;

/// `timeout_ticks` is PER BUS; size it to the longest legal transaction
/// (a polled request completes inside start() and never arms it - only
/// ISR-style completions are on this clock). ticks_from_ms<P>() converts.
template <typename Bus, Platform P, uint8_t pending_depth = 4,
          typename Policy = BusPassThrough, uint32_t timeout_ticks = 0>
using SpiBus = BusMaster<Bus, P, pending_depth, Policy, timeout_ticks>;

} // namespace brio
