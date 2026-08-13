/*
 * spi_bus.hpp
 *
 * SpiBus: the bus-owner active object for a SPI master. Generic over the
 * Bus engine (the target-side driver that moves the bytes under
 * interrupts): this layer only owns ARBITRATION and the REPLY channel,
 * which is exactly what makes a shared bus safe - clients post requests,
 * the AO serializes them, the requester gets its SpiDone back through
 * the ReplyTo capsule inside the request.
 *
 * Why the event queue alone is not the arbiter: a SPI transaction
 * OUTLIVES the dispatch that starts it (it runs on interrupts and
 * completes later), so while busy the kernel may well deliver the next
 * request - it needs a place to wait. That place is a small internal
 * pending FIFO (main-context only: no critical sections needed). Full
 * FIFO = the request is answered IMMEDIATELY with spi_rejected: never
 * silent, never blocking; an undersized FIFO shows up in the requester's
 * error handling, not as a lost transfer.
 *
 * Contract with the Bus engine (avrdx/spi.hpp on AVR, a fake in host
 * tests):
 *  - Bus::Request: the transaction descriptor. Must be trivially
 *    copyable and carry a `ReplyTo<SpiDone> reply` member. Buffer
 *    ownership travels with it: the requester must not touch the spans
 *    until its SpiDone arrives (RTC makes this race-free).
 *  - Bus::start(req): begin the transfer (assert CS, first byte). The
 *    engine runs on its ISR; when the transfer ends the app's ISR glue
 *    posts TransferDone{status} to this AO (same pattern as the uart
 *    RxActivity edge).
 *
 * The request event exceeds the 8-byte envelope guideline (a SPI
 * descriptor is ~14 bytes): a recorded, legal deviation - the request
 * IS the arbitration token, splitting it into a reference would add an
 * ownership protocol for zero RAM at queue depths this small.
 */

#pragma once

#include <stdint.h>
#include <variant>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"

namespace brio {

/// Reply payload of every SPI transaction.
struct SpiDone {
    uint8_t status;   ///< spi_ok, or why not
};

inline constexpr uint8_t spi_ok = 0;
inline constexpr uint8_t spi_rejected = 1;   ///< pending FIFO was full

/// Posted by the app's ISR glue when the engine finishes a transfer.
struct TransferDone {
    uint8_t status;
};

template <typename Bus, Platform P, uint8_t pending_depth = 4>
class SpiBus : public Fsm<SpiBus<Bus, P, pending_depth>,
                         typename Bus::Request, TransferDone> {
    using Base =
        Fsm<SpiBus<Bus, P, pending_depth>, typename Bus::Request, TransferDone>;
    using Request = typename Bus::Request;

public:
    using Event = typename Base::Event;
    using Status = typename Base::Status;

    // pending_depth requests can wait + one in flight + one TransferDone.
    static inline EventQueue<Event, pending_depth + 2, P> queue;

    static void init() { Base::start(&idle); }
    static void dispatch(const Event& e) { Base::dispatch(e); }

    /// Requests answered with spi_rejected because the FIFO was full.
    static uint8_t rejected_count() { return rejected_; }

private:
    static Status idle(const Event& e) {
        return std::visit(overloaded{
            [](const Request& r) {
                begin(r);
                return Base::transition(&busy);
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
                    r.reply.send(SpiDone{spi_rejected});
                }
                return Base::handled();
            },
            [](TransferDone d) {
                active_reply_.send(SpiDone{d.status});
                if (pending_count_ > 0) {
                    begin(pending_pop());
                    return Base::handled();     // stay busy on the next one
                }
                return Base::transition(&idle);
            },
            [](auto) { return Base::unhandled(); },
        }, e);
    }

    static void begin(const Request& r) {
        active_reply_ = r.reply;
        Bus::start(r);
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
    static inline ReplyTo<SpiDone> active_reply_{};
};

} // namespace brio
