/*
 * i2c_bus.hpp
 *
 * The I2C names of the shared bus arbiter: what a program names when
 * several active objects share one I2C bus. I2cBus is util/
 * bus_master.hpp's BusMaster, the reply is a BusDone, and the status
 * codes are the arbiter's two plus the wire-level outcomes an I2C master
 * can observe: a NACK (no acknowledge) on the address (nobody home: the
 * probe result an address scanner reads), a NACK on a data byte (the
 * device refused or finished early), arbitration lost against another
 * master, and a bus error (an illegal START or STOP, SDA stuck). The
 * engine (each target's I2cHost) reports them through
 * TransferDone{status}; the arbiter forwards them untouched. A program
 * names one arbiter per bus over the family's I2cHost and posts the
 * host's Request to it with a reply channel inside (kernel/post.hpp's
 * reply_to):
 *
 *     using Host = brio::I2cHost<1, host_pins>;   // the family's engine (stm32g0/i2c.hpp's spelling)
 *     using Bus = brio::I2cBus<Host, P>;
 *     brio::post<Bus>(Host::Request{..., .reply = brio::reply_to<MyAo, brio::I2cDone>()});
 *
 * Arbitration, the pending FIFO, replies and who owns the request's
 * buffers until when are documented once, in bus_master.hpp; the stack
 * and its realizations per family are docs/design/i2c-bus.md.
 *
 * The I2C transaction shape lives in the engine's Request (each
 * target's I2cHost): {addr, tx span, rx span} in one bus tenure (the
 * time the master holds the bus, from START to STOP): a write, a read,
 * or a write then a read with a repeated START (the register-access
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
/// engine was recover()ed, and the wire is now the application's to
/// judge (unstick(), re-probe: the recovery ladder). The arbiter's code,
/// not the engine's: see bus_master.hpp on why a client-wedged wire has
/// no silicon timeout on any engine here (measured; the reasoning is in
/// docs/design/i2c-bus.md, "The per-bus timeout").
inline constexpr uint8_t i2c_timeout = bus_timeout;

/// The arbiter under I2C's name. `pending_depth` is how many requests
/// may wait while one is in flight (a further one is answered
/// i2c_rejected at once). `timeout_ticks` is per bus and in kernel ticks
/// (one wedged client starves every other device of the wire) and must
/// sit above legal clock stretching, which is flow control, not a
/// fault: a whole worst-case tenure at the slowest device on the bus,
/// not a byte; 0 for no timeout at all. ticks_from_ms<P>()
/// (kernel/time.hpp) converts.
template <typename Bus, Platform P, uint8_t pending_depth = 4,
          typename Policy = BusPassThrough, uint32_t timeout_ticks = 0>
using I2cBus = BusMaster<Bus, P, pending_depth, Policy, timeout_ticks>;

} // namespace brio
