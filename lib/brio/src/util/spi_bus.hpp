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

template <typename Bus, Platform P, uint8_t pending_depth = 4>
using SpiBus = BusMaster<Bus, P, pending_depth>;

} // namespace brio
