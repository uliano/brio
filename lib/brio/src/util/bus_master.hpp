/*
 * bus_master.hpp
 *
 * BusMaster: the bus-owner active object for any master-side serial
 * bus (SPI, I2C/TWI, ...). Generic over the Bus engine (the target-side
 * driver that moves the bytes under interrupts): this layer only owns
 * ARBITRATION and the REPLY channel, which is exactly what makes a
 * shared bus safe - clients post requests, the AO serializes them, the
 * requester gets its BusDone back through the ReplyTo capsule inside
 * the request. Born as SpiBus (2026-08-13); generalized on the second
 * specimen, I2C (2026-08-17): the arbiter never looked at a byte, only
 * its name was SPI. util/spi_bus.hpp and util/i2c_bus.hpp keep the
 * per-bus vocabulary (SpiDone, I2cDone, status codes) as zero-cost
 * aliases so client code reads as what it is.
 *
 * Why the event queue alone is not the arbiter: a transaction OUTLIVES
 * the dispatch that starts it (it runs on interrupts and completes
 * later), so while busy the kernel may well deliver the next request -
 * it needs a place to wait. That place is a small internal pending FIFO
 * (main-context only: no critical sections needed). Full FIFO = the
 * request is answered IMMEDIATELY with bus_rejected: never silent,
 * never blocking; an undersized FIFO shows up in the requester's error
 * handling, not as a lost transfer.
 *
 * Contract with the Bus engine (avrdx/spi.hpp, avrdx/twi.hpp on AVR; a
 * fake in host tests):
 *  - Bus::Request: the transaction descriptor. Must be trivially
 *    copyable and carry a `ReplyTo<BusDone> reply` member. Buffer
 *    ownership travels with it: the requester must not touch the spans
 *    until its BusDone arrives (RTC makes this race-free).
 *  - Bus::start(req) -> bool: begin the transfer. FALSE = the engine
 *    runs on its ISR and the app's ISR glue posts TransferDone{status}
 *    to this AO when it ends (same pattern as the uart RxActivity
 *    edge). TRUE = the transaction COMPLETED SYNCHRONOUSLY inside
 *    start() (polled bulk transfers, degenerate empty requests): the
 *    reply is sent right away with bus_ok and no TransferDone must
 *    follow for it. Both styles interleave freely on one bus.
 *  - Status codes: bus_ok and bus_rejected are the arbiter's; every
 *    value >= bus_engine_status belongs to the engine's vocabulary
 *    (see i2c_bus.hpp) and travels untouched from TransferDone to the
 *    requester's BusDone.
 *
 * The request event exceeds the 8-byte envelope guideline (a SPI
 * descriptor is ~16 bytes, an I2C one 9): a recorded, legal deviation -
 * the request IS the arbitration token, splitting it into a reference
 * would add an ownership protocol for zero RAM at queue depths this
 * small.
 *
 * Validated on: AVR DA/DB (SpiHost<n>, TwiHost<n>) and the host fake.
 * The contract assumes a transaction that runs on interrupts and
 * completes later with a status only (buffers travel in the request);
 * a DMA engine or a peripheral with hardware chip-select/queues may
 * change it (docs/design/overview.md, "Authority of util/").
 */

#pragma once

#include <stdint.h>
#include <variant>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"

namespace brio {

/// Reply payload of every bus transaction.
struct BusDone {
    uint8_t status;   ///< bus_ok, bus_rejected, or an engine code
};

inline constexpr uint8_t bus_ok = 0;
inline constexpr uint8_t bus_rejected = 1;      ///< pending FIFO was full
inline constexpr uint8_t bus_engine_status = 2; ///< first engine-defined code

/// Posted by the app's ISR glue when the engine finishes a transfer.
struct TransferDone {
    uint8_t status;
};

template <typename Bus, Platform P, uint8_t pending_depth = 4>
class BusMaster : public Fsm<BusMaster<Bus, P, pending_depth>,
                             typename Bus::Request, TransferDone> {
    using Base = Fsm<BusMaster<Bus, P, pending_depth>,
                     typename Bus::Request, TransferDone>;
    using Request = typename Bus::Request;

public:
    using Event = typename Base::Event;
    using Status = typename Base::Status;

    // pending_depth requests can wait + one in flight + one TransferDone.
    static inline EventQueue<Event, pending_depth + 2, P> queue;

    static void init() { Base::start(&idle); }
    static void dispatch(const Event& e) { Base::dispatch(e); }

    /// Requests answered with bus_rejected because the FIFO was full.
    static uint8_t rejected_count() { return rejected_; }

private:
    static Status idle(const Event& e) {
        return std::visit(overloaded{
            [](const Request& r) {
                if (begin_chain(r)) {
                    return Base::transition(&busy);
                }
                return Base::handled();     // completed synchronously
            },
            [](auto) { return Base::unhandled(); },
        }, e);
    }

    static Status busy(const Event& e) {
        return std::visit(overloaded{
            [](const Request& r) {
                if (!pending_push(r)) {
                    if (rejected_ != UINT8_MAX) {
                        ++rejected_;
                    }
                    r.reply.send(BusDone{bus_rejected});
                }
                return Base::handled();
            },
            [](TransferDone d) {
                active_reply_.send(BusDone{d.status});
                if (pending_count_ > 0 && begin_chain(pending_pop())) {
                    return Base::handled();     // stay busy on the next one
                }
                return Base::transition(&idle);
            },
            [](auto) { return Base::unhandled(); },
        }, e);
    }

    /// Start r and keep draining the pending FIFO through synchronous
    /// completions. Returns true when a transfer went asynchronous
    /// (its TransferDone will arrive), false when everything finished.
    static bool begin_chain(Request r) {
        for (;;) {
            active_reply_ = r.reply;
            if (!Bus::start(r)) {
                return true;
            }
            active_reply_.send(BusDone{bus_ok});
            if (pending_count_ == 0) {
                return false;
            }
            r = pending_pop();
        }
    }

    // ---- pending FIFO: main-context only, no critical sections ----------
    static bool pending_push(const Request& r) {
        if (pending_count_ == pending_depth) {
            return false;
        }
        uint8_t slot = static_cast<uint8_t>(pending_head_ + pending_count_);
        if (slot >= pending_depth) {
            slot = static_cast<uint8_t>(slot - pending_depth);
        }
        pending_[slot] = r;
        ++pending_count_;
        return true;
    }

    static Request pending_pop() {
        const Request r = pending_[pending_head_];
        if (++pending_head_ == pending_depth) {
            pending_head_ = 0;
        }
        --pending_count_;
        return r;
    }

    template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };

    static inline Request pending_[pending_depth]{};
    static inline uint8_t pending_head_ = 0;
    static inline uint8_t pending_count_ = 0;
    static inline uint8_t rejected_ = 0;
    static inline ReplyTo<BusDone> active_reply_{};
};

} // namespace brio
