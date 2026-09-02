/*
 * i2c_bus.hpp
 *
 * The I2C vocabulary over util/bus_master.hpp: I2cBus IS BusMaster, the
 * reply is a BusDone, and the status codes are the arbiter's two plus
 * the wire-level outcomes an I2C master can observe - a NACK on the
 * address (nobody home: the probe result an address scanner reads), a
 * NACK on a data byte (device refused/finished early), arbitration
 * lost against another master, and a bus error (illegal START/STOP,
 * SDA stuck). The engine (avrdx/twi.hpp) reports them through
 * TransferDone{status}; the arbiter forwards them untouched.
 *
 * The I2C transaction shape lives in the engine's Request (see
 * avrdx/twi.hpp): {addr, tx span, rx span} in ONE bus tenure - write,
 * read, or write-then-read with a repeated START (the register-access
 * idiom), and the empty request as an address probe. Same rule as SPI:
 * the request is the complete script of one tenure, so nothing another
 * client could interleave into is left to the caller.
 */

#pragma once

#include <stdint.h>

#include "kernel/platform.hpp"
#include "util/bus_master.hpp"

namespace brio {

using I2cDone = BusDone;

inline constexpr uint8_t i2c_ok = bus_ok;
inline constexpr uint8_t i2c_rejected = bus_rejected;
inline constexpr uint8_t i2c_nack_addr = bus_engine_status + 0;  ///< no ACK on the address
inline constexpr uint8_t i2c_nack_data = bus_engine_status + 1;  ///< no ACK on a written byte
inline constexpr uint8_t i2c_arb_lost = bus_engine_status + 2;   ///< lost to another master
inline constexpr uint8_t i2c_bus_error = bus_engine_status + 3;  ///< protocol violation on the wire
/// The tenure never answered inside the arbiter's per-bus timeout: the
/// engine was recover()ed, and the WIRE is now the application's to
/// judge (unstick(), re-probe - the recovery ladder). Arbiter's code,
/// not the engine's: see bus_master.hpp on why a client-wedged wire has
/// no silicon timeout on any engine here (docs/samc/i2c.md, measured).
inline constexpr uint8_t i2c_timeout = bus_timeout;

/// `timeout_ticks` is PER BUS (one wedged client starves every other
/// device of the wire) and must sit ABOVE legal clock stretching, which
/// is flow control, not a fault: a whole worst-case tenure at the
/// slowest device on the bus, not a byte. ticks_from_ms<P>() converts.
template <typename Bus, Platform P, uint8_t pending_depth = 4,
          typename Policy = BusPassThrough, uint32_t timeout_ticks = 0>
using I2cBus = BusMaster<Bus, P, pending_depth, Policy, timeout_ticks>;

} // namespace brio
