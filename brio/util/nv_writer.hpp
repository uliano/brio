/*
 * nv_writer.hpp
 *
 * NvWriter: the active object that writes nonvolatile memory without
 * ever stalling the system. A caller posts one request (an address, the
 * bytes, their length, where to reply) and gets one NvDone back when
 * the bytes are down, the rest of the program running meanwhile. An
 * active object is a component with its own event queue whose dispatch
 * the kernel loop runs, one event at a time (docs/design/kernel.md,
 * section 1, "The model in one page").
 *
 * The smallest use names the writer over a store that has a ready
 * interrupt (util/nv_record.hpp's NvPacedStore: the AVR's EepromStore,
 * avrxt/nvm.hpp), lends it the bytes, and handles the reply in the
 * requesting active object:
 *
 *     using Writer = brio::NvWriter<brio::EepromStore, P, 2>;
 *
 *     static const uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
 *     brio::post<Writer>(brio::NvWrite{
 *         96, brio::lend<brio::Lease::reply>(payload), sizeof payload,
 *         brio::reply_to<Requester, brio::NvDone>()});
 *     // Requester's dispatch receives NvDone{status, written}
 *
 * plus the interrupt glue below, which posts NvReady.
 *
 * Who calls what, and from where:
 *  - the requester posts NvWrite from its own dispatch (post() itself
 *    is legal from any context, kernel/post.hpp); the reply comes later,
 *    as an NvDone event in the requester's queue;
 *  - the store's ready interrupt handler, the program's glue, disables
 *    the interrupt and posts NvReady;
 *  - NvWriter's dispatch, in the kernel loop like every active object,
 *    starts at most one operation per event and returns. No dispatch
 *    waits: none calls the store's wait_ready(), and the only reads of
 *    the store are made with no operation of the writer's in flight.
 *    Nothing here takes a critical section beyond the one each post()
 *    takes for the copy of its event (kernel/post.hpp); the pending
 *    FIFO is touched by the dispatch alone.
 *
 * The problem it exists for. A byte of EEPROM takes about ten
 * milliseconds to erase-and-write, and the silicon halts the CPU the
 * moment a second store reaches the array while the first is still in
 * flight. Written naively - a loop with a busy wait - saving a
 * thirty-byte record freezes every other active object for a third of a
 * second. That is not a slow write, it is a dead system.
 *
 * The shape. One operation per interrupt: the store announces "ready"
 * with its own interrupt, the program's interrupt glue posts NvReady,
 * and the writer's dispatch starts exactly one operation and returns -
 * one byte on a byte store, one run (the differing bytes of one write
 * unit, util/nv_record.hpp's NvRunStore) on a store that takes a run.
 * Between two operations the kernel runs everything else. The whole
 * transfer is therefore made of dispatches of some tens of
 * microseconds (what one run costs on the AVR EA: docs/avrxt/nvm.md)
 * spread over as long as the memory needs, and no code anywhere waits.
 *
 * The interface is BusMaster's, for the same reason (util/bus_master.hpp):
 * a request that outlives its dispatch needs a queue to wait in and a
 * return channel to answer on. Requests that arrive while one is in
 * flight wait in a small FIFO of pending_depth entries; a full FIFO
 * answers immediately with nv_rejected rather than blocking or
 * dropping. The writer's event queue holds pending_depth + 2 events
 * (the code's own count: the waiting requests, the one in flight and
 * one NvReady); a post past that is dropped by the queue and counted in
 * queue.overflows() (kernel/event_queue.hpp), with no reply. The writer
 * never sees a dropped request, so its bytes are the requester's again
 * at once, and that counter is the only trace of it.
 *
 * Who owns the bytes, and until when. The bytes travel as a
 * Borrowed<..., Lease::reply> inside the request (kernel/borrowed.hpp)
 * and are read as the writer advances: the requester must keep them
 * alive and unchanged from the post until its NvDone arrives, whatever
 * its status. A static buffer or a member of the requesting active
 * object is the normal answer. Nothing checks it.
 *
 * Interrupt glue the program must provide (the vector name is the
 * target's, and never appears in portable code; on the AVR the
 * driver's handler body Nvm::eeready() does the disabling,
 * avrxt/nvm.hpp):
 *
 *     ISR(<the store's ready vector>) {
 *         Store::arm_ready_interrupt(false);   // level flag: mandatory
 *         brio::post<Writer>(brio::NvReady{});
 *     }
 *
 * The disable is mandatory because the ready flag is a level: left
 * enabled, the vector would run again as soon as it returned, for as
 * long as the store is idle. The writer re-arms it when it starts the
 * next operation.
 */

#pragma once

#include <stdint.h>
#include <variant>

#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"
#include "util/nv_record.hpp"

namespace brio {

/// Reply payload of an NvWrite: sent once per request the writer
/// received, whatever its fate (a request the event queue dropped never
/// reached it, and gets none). After it the lent bytes are the
/// requester's again.
struct NvDone {
    uint8_t status;    ///< nv_ok, nv_rejected, nv_bad_range, nv_refused
    uint16_t written;  ///< bytes actually committed (0 .. len)
};

inline constexpr uint8_t nv_ok = 0;
inline constexpr uint8_t nv_rejected = 1;   ///< the pending FIFO was full
inline constexpr uint8_t nv_bad_range = 2;  ///< the request leaves the store
inline constexpr uint8_t nv_refused = 3;    ///< the store refused a byte or a run

/// Write `len` bytes at `addr`. Bytes already equal to what is stored
/// are skipped (endurance, exactly as in NvRecord::store), so `written`
/// in the reply can be smaller than `len`: zero when the store already
/// held them all. A request that leaves the store is answered with
/// nv_bad_range and writes nothing.
struct NvWrite {
    uint16_t addr;
    /// The source bytes, read as the writer advances: lent until the
    /// reply lands.
    Borrowed<const uint8_t, Lease::reply> data;
    uint16_t len;
    ReplyTo<NvDone> reply;
};

/// Posted by the program's interrupt glue when the store's ready
/// interrupt fires.
struct NvReady {};

template <NvPacedStore Store, Platform P, uint8_t pending_depth = 2>
class NvWriter : public Fsm<NvWriter<Store, P, pending_depth>, NvWrite, NvReady> {
    using Base = Fsm<NvWriter<Store, P, pending_depth>, NvWrite, NvReady>;

public:
    using Event = typename Base::Event;
    using Status = typename Base::Status;

    // pending_depth requests waiting + one in flight + one NvReady.
    static inline EventQueue<Event, pending_depth + 2, P> queue;

    /// The kernel's call before the first dispatch (Tenuto calls every
    /// active object's init()): the ready interrupt off, the FIFO empty.
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
    /// What one advance of the active request found: an operation is now
    /// in flight, the request is complete, or the store refused it.
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
    /// one that differs - on a store that takes a run, the run from it to
    /// the last differing byte of its write unit. The reply is NOT sent
    /// here - the caller decides, because it is also the one that chains
    /// the next request.
    static Step step() {
        if constexpr (NvRunStore<Store>) {
            const NvRunScan r =
                nv_scan_run<Store>(active_.addr, active_.data.get(), next_, active_.len);
            next_ = r.resume;
            if (r.len == 0) {
                return Step::done;
            }
            if (!Store::write_run(static_cast<uint16_t>(active_.addr + r.from),
                                  active_.data.get() + r.from, r.len)) {
                return Step::refused;
            }
            written_ = static_cast<uint16_t>(written_ + r.changed);
            Store::clear_ready_flag();
            Store::arm_ready_interrupt(true);
            return Step::in_flight;
        }
        while (next_ < active_.len) {
            const uint16_t addr = static_cast<uint16_t>(active_.addr + next_);
            const uint8_t want = active_.data.get()[next_];
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
