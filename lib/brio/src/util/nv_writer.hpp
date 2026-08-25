/*
 * nv_writer.hpp
 *
 * NvWriter: the active object that writes nonvolatile memory WITHOUT
 * ever stalling the system.
 *
 * The problem it exists for. A byte of EEPROM takes about ten
 * milliseconds to erase-and-write, and the silicon halts the CPU the
 * moment a second store reaches the array while the first is still in
 * flight. Written naively - a loop with a busy wait - saving a
 * thirty-byte record freezes every other active object for a third of a
 * second. That is not a slow write, it is a dead system.
 *
 * The shape. One byte per interrupt: the store announces "ready" with
 * its own interrupt, the app's ISR glue posts NvReady, and the writer's
 * dispatch writes exactly ONE byte and returns. Between two bytes the
 * kernel runs everything else. The whole transfer is therefore made of
 * ten-microsecond dispatches spread over as long as the memory needs,
 * and no code anywhere waits.
 *
 * The interface is BusMaster's, for the same reason (util/bus_master.hpp):
 * a request that outlives its dispatch needs a queue to wait in and a
 * return channel to answer on. Requests that arrive while one is in
 * flight wait in a small FIFO; a full FIFO answers immediately with
 * nv_rejected rather than blocking or dropping.
 *
 * OWNERSHIP. The bytes travel as a pointer inside the request and are
 * read one at a time as the writer advances: the requester must keep
 * them alive and unchanged until its NvDone arrives. A static buffer or
 * a member of the requesting AO is the normal answer.
 *
 * ISR GLUE the app must provide (the vector name never appears in
 * portable code):
 *
 *     ISR(NVMCTRL_EE_vect) {
 *         Store::arm_ready_interrupt(false);   // level flag: mandatory
 *         brio::post<Writer>(brio::NvReady{});
 *     }
 */

#pragma once

#include <stdint.h>
#include <variant>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"
#include "util/nv_record.hpp"

namespace brio {

/// Reply payload of an NvWrite.
struct NvDone {
    uint8_t status;    ///< nv_ok, nv_rejected, nv_bad_range, nv_refused
    uint16_t written;  ///< bytes actually committed (0 .. len)
};

inline constexpr uint8_t nv_ok = 0;
inline constexpr uint8_t nv_rejected = 1;   ///< the pending FIFO was full
inline constexpr uint8_t nv_bad_range = 2;  ///< the request leaves the store
inline constexpr uint8_t nv_refused = 3;    ///< the store refused a byte

/// Write `len` bytes at `addr`. Bytes already equal to what is stored
/// are SKIPPED (endurance, exactly as in NvRecord::store), so `written`
/// in the reply is normally smaller than `len`.
struct NvWrite {
    uint16_t addr;
    const uint8_t* data;
    uint16_t len;
    ReplyTo<NvDone> reply;
};

/// Posted by the app's ISR glue when the store's ready interrupt fires.
struct NvReady {};

template <NvPacedStore Store, Platform P, uint8_t pending_depth = 2>
class NvWriter : public Fsm<NvWriter<Store, P, pending_depth>, NvWrite, NvReady> {
    using Base = Fsm<NvWriter<Store, P, pending_depth>, NvWrite, NvReady>;

public:
    using Event = typename Base::Event;
    using Status = typename Base::Status;

    // pending_depth requests waiting + one in flight + one NvReady.
    static inline EventQueue<Event, pending_depth + 2, P> queue;

    static void init() {
        Store::arm_ready_interrupt(false);
        pending_head_ = pending_count_ = 0;
        rejected_ = 0;
        Base::start(&idle);
    }
    static void dispatch(const Event& e) { Base::dispatch(e); }

    /// Requests answered with nv_rejected because the FIFO was full.
    static uint8_t rejected_count() { return rejected_; }

private:
    /// What one advance of the active request found: a byte is now in
    /// flight, the request is complete, or the store refused a byte.
    enum class Step : uint8_t { in_flight, done, refused };

    static Status idle(const Event& e) {
        return std::visit(overloaded{
            [](const NvWrite& w) {
                if (begin_chain(w)) {
                    return Base::transition(&busy);
                }
                return Base::handled();
            },
            [](auto) { return Base::unhandled(); },
        }, e);
    }

    static Status busy(const Event& e) {
        return std::visit(overloaded{
            [](const NvWrite& w) {
                if (!pending_push(w)) {
                    if (rejected_ != UINT8_MAX) {
                        ++rejected_;
                    }
                    w.reply.send(NvDone{nv_rejected, 0});
                }
                return Base::handled();
            },
            [](NvReady) {
                const Step s = step();
                if (s == Step::in_flight) {
                    return Base::handled();     // another byte is in flight
                }
                finish(s == Step::refused ? nv_refused : nv_ok);
                if (pending_count_ > 0 && begin_chain(pending_pop())) {
                    return Base::handled();
                }
                return Base::transition(&idle);
            },
            [](auto) { return Base::unhandled(); },
        }, e);
    }

    /// Start w and keep draining the FIFO through requests that finish
    /// without ever needing an interrupt (nothing to change, empty, or
    /// out of range). True = a byte is in flight and NvReady will come.
    static bool begin_chain(NvWrite w) {
        for (;;) {
            active_ = w;
            next_ = 0;
            written_ = 0;
            if (static_cast<uint32_t>(w.addr) + w.len >
                static_cast<uint32_t>(Store::size())) {
                w.reply.send(NvDone{nv_bad_range, 0});
            } else {
                const Step s = step();
                if (s == Step::in_flight) {
                    return true;
                }
                finish(s == Step::refused ? nv_refused : nv_ok);
            }
            if (pending_count_ == 0) {
                return false;
            }
            w = pending_pop();
        }
    }

    /// Skip every byte already equal to its target and start the first
    /// one that differs. The reply is NOT sent here - the caller decides,
    /// because it is also the one that chains the next request.
    static Step step() {
        while (next_ < active_.len) {
            const uint16_t addr = static_cast<uint16_t>(active_.addr + next_);
            const uint8_t want = active_.data[next_];
            ++next_;
            if (Store::read(addr) == want) {
                continue;
            }
            if (!Store::write(addr, want)) {
                return Step::refused;
            }
            ++written_;
            Store::clear_ready_flag();
            Store::arm_ready_interrupt(true);
            return Step::in_flight;
        }
        return Step::done;
    }

    static void finish(uint8_t status) {
        Store::arm_ready_interrupt(false);
        Store::finish();
        active_.reply.send(NvDone{status, written_});
    }

    // ---- pending FIFO: main-context only, no critical sections ----------
    static bool pending_push(const NvWrite& w) {
        if (pending_count_ == pending_depth) {
            return false;
        }
        uint8_t slot = static_cast<uint8_t>(pending_head_ + pending_count_);
        if (slot >= pending_depth) {
            slot = static_cast<uint8_t>(slot - pending_depth);
        }
        pending_[slot] = w;
        ++pending_count_;
        return true;
    }

    static NvWrite pending_pop() {
        const NvWrite w = pending_[pending_head_];
        if (++pending_head_ == pending_depth) {
            pending_head_ = 0;
        }
        --pending_count_;
        return w;
    }

    template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };

    static inline NvWrite active_{};
    static inline uint16_t next_ = 0;
    static inline uint16_t written_ = 0;
    static inline NvWrite pending_[pending_depth]{};
    static inline uint8_t pending_head_ = 0;
    static inline uint8_t pending_count_ = 0;
    static inline uint8_t rejected_ = 0;
};

} // namespace brio
