/*
 * i2c_bus.hpp
 *
 * The I2C names of the shared bus arbiter: what a program names when
 * several active objects share one I2C bus. I2cBus is util/
 * bus_master.hpp's BusMaster, the reply is a BusDone, and the status
 * codes are the arbiter's two plus the wire-level outcomes an I2C host
 * can observe: a NACK (no acknowledge) on the address (no device answered:
 * the probe result an address scanner reads), a NACK on a data byte (the
 * device would not take it, or had finished early), arbitration lost
 * against another host, and a bus error (any other fault that ends the
 * tenure; which faults do differs by host, docs/design/i2c-bus.md, "What a
 * host does on the wire"). The host (each target's I2cHost) reports them
 * through TransferDone{status}; the arbiter forwards them untouched. A program
 * names one arbiter per bus over the family's I2cHost and posts the
 * host's Request to it with a reply channel inside (kernel/post.hpp's
 * reply_to):
 *
 *     using Host = brio::I2cHost<1, host_pins>;   // the family's host (stm32g0/i2c.hpp's spelling)
 *     using Bus = brio::I2cBus<Host, P>;
 *     brio::post<Bus>(Host::Request{..., .reply = brio::reply_to<MyAo, brio::I2cDone>()});
 *
 * Arbitration, the pending FIFO, replies and who owns the request's
 * buffers until when are documented once, in bus_master.hpp; the stack
 * and its realizations per family are docs/design/i2c-bus.md.
 *
 * The I2C transaction shape lives in the host's Request (each
 * target's I2cHost): {addr, tx span, rx span} in one bus tenure (one
 * START to its STOP, the bus held for one request): a write, a read,
 * or a write then a read with a repeated START (the register-access
 * idiom), and the empty request as an address probe. Same rule as SPI:
 * the request is the complete script of one tenure, so nothing another
 * requester could interleave into is left to the caller.
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
inline constexpr uint8_t i2c_arb_lost = bus_engine_status + 2;   ///< lost to another host
inline constexpr uint8_t i2c_bus_error = bus_engine_status + 3;  ///< any other fault that ends the tenure (the bus error above)
/// The tenure never answered inside the arbiter's per-bus timeout: the
/// host was recover()ed, and the wire is now the application's to
/// judge (unstick(), re-probe: the recovery ladder). On a host whose start()
/// has a bound on the last STOP, also that bound running out, with nothing of
/// the request on the wire and no recover(). Otherwise the arbiter's code,
/// not the host's: no silicon time-out staged here sees a held SDA, and of
/// those staged only the SAM C21's, armed, bound a client's hold of SCL (the
/// measurements are in docs/design/i2c-bus.md, "The per-bus timeout").
inline constexpr uint8_t i2c_timeout = bus_timeout;

/// The 7-bit addresses a Request may name: 0x08 to 0x77, the range the
/// I2C specification leaves to devices (NXP UM10204 3.1.12). The sixteen
/// outside it are reserved: 0x00 is the general call and, with the read
/// bit, the START byte; 0x01 is CBUS, 0x02 other bus formats, 0x03 a future
/// purpose and 0x04 to 0x07 the Hs-mode codes; 0x78 to 0x7B are the first byte of a 10-bit address, and
/// 0x7C to 0x7F the device ID and a future purpose. Every host's start()
/// asks this one function and answers an address outside the range
/// i2c_rejected at once, nothing on the wire - the path a speed the
/// clock cannot produce already takes. The reason is the wire, not
/// tidiness: the STM32F1's event machine, which several families carry,
/// sends an address of 0x78 to 0x7B as a 10-bit header, so the same
/// Request would mean two different things on two blocks. The general
/// call is refused with the rest until it is a feature of its own, and
/// 10-bit addressing is not offered by any host.
inline constexpr uint8_t i2c_address_first = 0x08;
inline constexpr uint8_t i2c_address_last = 0x77;

/// Is `addr` a 7-bit address a Request may name (i2c_address_first to
/// i2c_address_last)? One subtraction and one unsigned compare.
constexpr bool i2c_address_in_range(uint8_t addr) {
    return static_cast<uint8_t>(addr - i2c_address_first) <=
           static_cast<uint8_t>(i2c_address_last - i2c_address_first);
}

/// Which mechanism moves a request's data bytes, named by the program in
/// the Request's `engine` field on a host that takes it. The address, the
/// repeated START and the end of a tenure are the host's interrupt in
/// every case; only the data halves - the bytes written, the bytes read -
/// move one way or the other:
///
///   interrupt  the default, what a Request that does not name the field
///              gets: the host's interrupt, one entry per byte or per FIFO
///              level, at any length
///   dma        the DMA channels bound to the host (its engines) carry
///              every half they can, at any length; a half they cannot
///              carry (the host's header names which) stays on the
///              interrupt and the request completes, not refused: the
///              interrupt path is asynchronous too, so the fallback costs
///              core cycles and changes nothing of the request's shape
///              (an SPI host refuses instead, its fallback being a loop
///              that would hold the arbiter's dispatch)
///
/// The host never chooses. A wrong choice costs core cycles, a sleep, or
/// wire time when the interrupt stretches the clock, and in one case
/// correctness: a half in a memory no DMA master reaches should end
/// i2c_dma_fault under `dma` where `interrupt` would succeed
/// (docs/design/i2c-bus.md). The length from which `dma` costs the core less
/// is in each family's document. A host with no engines bound (its
/// `has_dma` is false) answers `dma` i2c_rejected from start(), nothing
/// armed and nothing on the wire, as it answers an address outside the
/// range. A host that does not carry the field has no such member,
/// and a Request that names it does not compile there.
enum class I2cEngine : uint8_t {
    interrupt = 0,
    dma = 1,
};

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
