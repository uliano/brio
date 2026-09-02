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
 * WHAT HAPPENS ON A FAILURE is a POLICY, and it is the `Policy`
 * template argument's - see BusPassThrough below. The arbiter is the
 * only object that knows a transfer failed AND still holds the request
 * that failed, so it is the only place a retry can be decided without
 * every client writing one; but WHICH failures are worth retrying, and
 * how often, is knowledge about the devices on the wire that no generic
 * arbiter has. The default answers "none", at compile time and for
 * free: the images of every existing bus are byte-identical with the
 * hook in place.
 *
 * WHAT HAPPENS ON A TRANSFER THAT NEVER ANSWERS is the `timeout_ticks`
 * template argument's, and the arbiter is again the only honest home:
 * it is the one object that knows a completion is OWED, and it lives in
 * a kernel that has TimeEvents - the engine is interrupt-driven and
 * owns no clock, and the silicon's own time-outs (where they exist at
 * all: SMBus mode on the SAM SERCOM) police the HOST'S OWN clock hold,
 * not a wire a client wedged (measured, docs/samc/i2c.md). With
 * timeout_ticks != 0 every transfer that goes asynchronous arms a
 * one-shot TimeEvent; if it matures first, the engine is declared dead:
 * Bus::recover() puts the PERIPHERAL back where start() is legal, the
 * requester is answered bus_timeout IN ITS PLACE like every reply, and
 * the queue moves on. THE WIRE STAYS THE APPLICATION'S: whether to
 * unstick(), power-cycle a client, or re-probe the whole bus is a
 * recovery ladder no arbiter can own (ruling 2026-09-02) - what the
 * timeout guarantees is only that the bus AO and its queue survive to
 * be asked. A timeout is NOT a completion: the retry policy is not
 * consulted (the engine never spoke), and stretching below the limit
 * is legal flow control, so the value must be generous - a whole
 * transaction's worth at the slowest device, not a byte's.
 *
 * THE RACE WITH THE REAL COMPLETION is closed by construction, both
 * ways. A TransferDone already QUEUED when the timeout event is served
 * is detected by a per-transfer sequence number the timeout carries
 * (stale = dropped, counted); a TransferDone posted DURING the timeout
 * dispatch (the engine finishing in the very window recover() closes)
 * necessarily enters the queue BEFORE the BusFlushed marker the
 * handler posts after recover() returns - recover() silences the
 * engine, so nothing can post later - and is therefore drained in the
 * one state that expects it, before the next request starts. With
 * timeout_ticks == 0 (the default) none of this exists: no extra
 * variant alternatives, no timer, no states - byte-identical images,
 * the never_retries discipline again.
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
#include <concepts>
#include <type_traits>
#include <variant>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"
#include "kernel/time_event.hpp"
#include "util/power.hpp"

namespace brio {

/// Reply payload of every bus transaction.
struct BusDone {
    uint8_t status;   ///< bus_ok, bus_rejected, bus_timeout, or an engine code
};

inline constexpr uint8_t bus_ok = 0;
inline constexpr uint8_t bus_rejected = 1;      ///< pending FIFO was full
inline constexpr uint8_t bus_engine_status = 2; ///< first engine-defined code
/// The transfer never answered inside the arbiter's timeout_ticks: the
/// engine was recover()ed and this reply stands in for the completion
/// that never came. TOP OF THE RANGE, deliberately: engine vocabularies
/// grow UP from bus_engine_status (i2c_bus.hpp holds four already) and
/// must never collide with an arbiter code.
inline constexpr uint8_t bus_timeout = 255;

/// Posted by the app's ISR glue when the engine finishes a transfer.
struct TransferDone {
    uint8_t status;
};

/// Posted to a TIMED arbiter by its own TimeEvent: the transfer whose
/// sequence number this carries never completed. Exists in a bus AO's
/// event set only when timeout_ticks != 0.
struct BusTimeout {
    uint8_t seq;
};

/// Self-posted by a timed arbiter right after a timeout recovery: when
/// it arrives, everything the dead transfer could still have queued (a
/// TransferDone posted in the window recover() was closing) has been
/// seen and drained - the FIFO order of the AO queue is the guarantee.
struct BusFlushed {};

namespace detail {
/// The arbiter's Fsm base: the two timeout alternatives exist only in a
/// timed instantiation, so an untimed bus keeps today's exact variant
/// (and its exact code - the byte-identity discipline).
template <bool timed, typename Self, typename Request>
using BusMasterFsm = std::conditional_t<
    timed,
    Fsm<Self, Request, TransferDone, PrepareSleep, BusTimeout, BusFlushed>,
    Fsm<Self, Request, TransferDone, PrepareSleep>>;
} // namespace detail

// ---- the completion policy ---------------------------------------------------

/**
 * What the arbiter does with a finished transfer. `pass` = answer the
 * requester with the status the engine reported (what a bus has always
 * done); `retry` = start the SAME request again and say nothing yet.
 */
enum class BusAction : uint8_t {
    pass,
    retry,
};

/**
 * The default: every completion goes straight back to the requester.
 *
 * `never_retries` is the opt-out that makes this hook FREE. A policy
 * that declares it is answering `pass` at compile time, so the arbiter
 * neither calls on_done() nor keeps the copy of the in-flight request a
 * retry would need - and the generated code is, byte for byte, the code
 * that existed before the hook. Any other policy is assumed to retry
 * and pays for the copy.
 *
 * A policy that DOES retry writes on_done(status, attempt) -> BusAction:
 * `status` is the engine's, `attempt` counts the retries already spent
 * on this request (0 at its first completion) and resets with each new
 * request. It is called in the arbiter's dispatch, main context, and
 * must be a pure decision - the place for a recovery ladder's ACTIONS
 * (a bus reset, a clock pulse train) is the engine, not here.
 *
 * Note that the hook sees ASYNCHRONOUS completions only: a transfer the
 * engine finishes inside start() reported bus_ok by that very fact, and
 * there is nothing for a policy to judge.
 */
struct BusPassThrough {
    static constexpr bool never_retries = true;
    static constexpr BusAction on_done(uint8_t, uint8_t) { return BusAction::pass; }
};

/// True unless the policy declares itself retry-free. Absent = assume it
/// may retry: the safe half of the guess costs a copy, the other half
/// would lose a request.
template <typename Policy>
constexpr bool bus_policy_may_retry() {
    if constexpr (requires { { Policy::never_retries } -> std::convertible_to<bool>; }) {
        return !Policy::never_retries;
    } else {
        return true;
    }
}

/**
 * The arbiter, and - because it is the one object that knows whether the
 * wire is quiet - a power-management VOTER (util/power.hpp): it answers
 * a PrepareSleep with ok only when nothing is in flight and nothing is
 * waiting. A transfer runs on interrupts the deep modes may gate, and it
 * outlives the dispatch that started it, so it is precisely the kind of
 * fact the requester of a sleep cannot have. The vote costs a variant
 * alternative three bytes wide, which no bus request comes close to, and
 * two lambdas.
 *
 * A RETRYING MASTER IS BUSY. When the policy asks for a retry the
 * arbiter stays in its busy state, which is also the state that votes
 * NOT-OK on a PrepareSleep: a request whose completion is still owed is
 * exactly the fact a sleep must not be taken against, and it stays true
 * across as many attempts as the policy spends. A DRAINING master (the
 * dispatch after a timeout recovery) is busy in the same sense.
 *
 * `timeout_ticks` is PER BUS and in KERNEL TICKS (ticks_from_ms<P>()
 * converts): one wedged device on a shared bus starves every other
 * client of that bus, so the limit is a property of the wire, not of a
 * request (ruling 2026-09-02). 0 = no timeout, and no timeout code.
 */
template <typename Bus, Platform P, uint8_t pending_depth = 4,
          typename Policy = BusPassThrough, uint32_t timeout_ticks = 0>
class BusMaster
    : public detail::BusMasterFsm<timeout_ticks != 0,
                                  BusMaster<Bus, P, pending_depth, Policy, timeout_ticks>,
                                  typename Bus::Request> {
    /// Whether the timeout machinery exists at all in this instantiation.
    static constexpr bool timed = timeout_ticks != 0;

    using Base = detail::BusMasterFsm<timed, BusMaster, typename Bus::Request>;
    using Request = typename Bus::Request;

    static_assert(!timed || requires { Bus::recover(); },
                  "a timed BusMaster needs Bus::recover(): the verb that puts a dead "
                  "engine back where start() is legal again (avrdx/twi.hpp's is the "
                  "model). The WIRE is not its job - unstick() and the recovery "
                  "ladder stay the application's");

    /// Whether the retry machinery exists at all in this instantiation.
    static constexpr bool may_retry = bus_policy_may_retry<Policy>();

    /// The held copy of the request in flight - present only where a
    /// retry could ask for it back.
    struct NoHold {};
    using Held = std::conditional_t<may_retry, Request, NoHold>;

    /// The timeout state, instantiated only when timed (the Held
    /// discipline again): the one-shot timer - a raw TimeEvents node
    /// with its own firing glue, because the posted payload must carry
    /// the sequence number AT FIRE TIME, which a TimeEvent's
    /// construction-time payload cannot - plus the per-transfer sequence
    /// and the stale-event tally.
    struct TimedState {
        struct Node : TimeEvents<P>::Base {
            constexpr Node() : TimeEvents<P>::Base(&fire) {}

        private:
            static void fire(typename TimeEvents<P>::Base&) {
                // Main context (TimeEvents<P>::process), so seq is
                // exactly the armed transfer's: the timer is disarmed
                // at every completion BEFORE seq moves on.
                post<BusMaster>(BusTimeout{seq});
            }
        };
        static inline Node timer{};
        static inline uint8_t seq = 0;    ///< counts wire transfers, not requests
        static inline uint8_t stale = 0;  ///< dropped stale events (saturating)
    };
    struct NoTimedState {};
    using TState = std::conditional_t<timed, TimedState, NoTimedState>;

public:
    using Event = typename Base::Event;
    using Status = typename Base::Status;

    // pending_depth requests can wait + one in flight + one TransferDone
    // + one PrepareSleep. The vote's slot is not optional: a dropped
    // PrepareSleep is a vote that never comes back, and the manager waits
    // for unanimity rather than timing out. A timed bus adds headroom
    // for its own events: a BusTimeout or two (a stale one can coexist
    // with the next transfer's) and the BusFlushed marker.
    static inline EventQueue<Event, pending_depth + (timed ? 6 : 3), P> queue;

    /// Full reset, like every other AO's init(): the pending FIFO, the
    /// rejection tally, the retry counter and the active reply all go
    /// back to power-on state, so a re-init cannot replay a stale
    /// request (found by the host suite; PowerManager and AnalogSampler
    /// already followed this rule).
    static void init() {
        pending_head_ = 0;
        pending_count_ = 0;
        rejected_ = 0;
        attempt_ = 0;
        active_reply_ = {};
        if constexpr (timed) {
            TimeEvents<P>::disarm(TState::timer);
            TState::seq = 0;
            TState::stale = 0;
        }
        Base::start(&idle);
    }
    static void dispatch(const Event& e) { Base::dispatch(e); }

    /// Requests answered with bus_rejected because the FIFO was full.
    static uint8_t rejected_count() { return rejected_; }

    /// Retries the policy has spent on the request in flight (0 when it
    /// has not asked for any, and after every completion that passed).
    static uint8_t attempt() { return attempt_; }

    /// Timed buses only: events dropped as stale - a BusTimeout that
    /// lost the race with its own transfer's completion, or the dead
    /// transfer's TransferDone drained after a recovery. A diagnostic,
    /// not an error: each one is a race CLOSED correctly.
    static uint8_t stale_events() {
        if constexpr (timed) {
            return TState::stale;
        } else {
            return 0;
        }
    }

private:
    static Status idle(const Event& e) {
        return std::visit(overloaded{
            [](const Request& r) {
                if (begin_chain(r)) {
                    return Base::transition(&busy);
                }
                return Base::handled();     // completed synchronously
            },
            [](const PrepareSleep& p) {
                // Idle means idle: nothing in flight, and the FIFO is
                // empty by construction (this state is only reached once
                // it has drained). Checked anyway - a vote is a claim
                // about the machine, not about the state chart.
                p.reply.send(SleepVote{pending_count_ == 0});
                return Base::handled();
            },
            [](BusTimeout) {
                // Stale by definition: idle owes nobody a completion (a
                // timeout that lost the race with a synchronous drain).
                count_stale();
                return Base::handled();
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
                // Every TransferDone seen here is the CURRENT transfer's:
                // a dead transfer's straggler can only exist between a
                // timeout recovery and its BusFlushed marker, and that
                // window is the draining state, not this one.
                if constexpr (timed) {
                    TimeEvents<P>::disarm(TState::timer);
                }
                if constexpr (may_retry) {
                    if (Policy::on_done(d.status, attempt_) == BusAction::retry) {
                        ++attempt_;
                        if (!Bus::start(held_)) {
                            arm_timeout();              // a new wire transfer
                            return Base::handled();     // the retry is in flight
                        }
                        // The retry finished inside start(): bus_ok by
                        // that fact, and the request is answered.
                        attempt_ = 0;
                        active_reply_.send(BusDone{bus_ok});
                        if (pending_count_ > 0 && begin_chain(pending_pop())) {
                            return Base::handled();
                        }
                        return Base::transition(&idle);
                    }
                    attempt_ = 0;
                }
                active_reply_.send(BusDone{d.status});
                if (pending_count_ > 0 && begin_chain(pending_pop())) {
                    return Base::handled();     // stay busy on the next one
                }
                return Base::transition(&idle);
            },
            [](const PrepareSleep& p) {
                // A transfer is in flight: its completion interrupt is
                // exactly what a gated clock domain would swallow.
                p.reply.send(SleepVote{false});
                return Base::handled();
            },
            [](BusTimeout t) {
                if constexpr (timed) {
                    if (t.seq != TState::seq) {
                        // An earlier transfer's, overtaken by its own
                        // completion while both sat in the queue.
                        count_stale();
                        return Base::handled();
                    }
                    // The transfer in flight never answered. The ENGINE
                    // is declared dead and put back where start() is
                    // legal; the WIRE's health is the application's to
                    // judge from this very reply. Not a completion: the
                    // retry policy is not consulted.
                    Bus::recover();
                    if constexpr (may_retry) {
                        attempt_ = 0;
                    }
                    active_reply_.send(BusDone{bus_timeout});
                    // Anything the dying transfer still posted entered
                    // the queue before this marker will (recover()
                    // silenced the engine): drain it before the next
                    // request can be confused with it.
                    post<BusMaster>(BusFlushed{});
                    return Base::transition(&draining);
                } else {
                    return Base::handled();   // unreachable: never posted
                }
            },
            [](auto) { return Base::unhandled(); },
        }, e);
    }

    /// The dispatch after a timeout recovery (timed instantiations
    /// only): between the recovery and its BusFlushed marker, so the one
    /// place a dead transfer's straggler is EXPECTED. Busy in every
    /// other respect: requests wait or are rejected, sleep is refused.
    static Status draining(const Event& e) {
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
            [](TransferDone) {
                // The dead transfer's own completion, posted in the
                // window recover() was closing: its requester was
                // already answered bus_timeout - drop it, count it.
                count_stale();
                return Base::handled();
            },
            [](BusTimeout) {
                count_stale();      // stale by definition here
                return Base::handled();
            },
            [](BusFlushed) {
                if (pending_count_ > 0 && begin_chain(pending_pop())) {
                    return Base::transition(&busy);
                }
                return Base::transition(&idle);
            },
            [](const PrepareSleep& p) {
                // A recovery is in progress and requests may be waiting.
                p.reply.send(SleepVote{false});
                return Base::handled();
            },
            [](auto) { return Base::unhandled(); },
        }, e);
    }

    /// Start r and keep draining the pending FIFO through synchronous
    /// completions. Returns true when a transfer went asynchronous
    /// (its TransferDone will arrive), false when everything finished.
    static bool begin_chain(Request r) {
        for (;;) {
            if constexpr (may_retry) {
                held_ = r;              // the copy a retry would start again
                attempt_ = 0;           // attempts are counted per request
            }
            active_reply_ = r.reply;
            if (!Bus::start(r)) {
                arm_timeout();
                return true;
            }
            active_reply_.send(BusDone{bus_ok});
            if (pending_count_ == 0) {
                return false;
            }
            r = pending_pop();
        }
    }

    /// A transfer just went asynchronous: give it its sequence number
    /// and start the clock. Folded away entirely on an untimed bus.
    static void arm_timeout() {
        if constexpr (timed) {
            ++TState::seq;
            TimeEvents<P>::arm(TState::timer, timeout_ticks, 0);
        }
    }

    static void count_stale() {
        if constexpr (timed) {
            if (TState::stale != UINT8_MAX) {
                ++TState::stale;
            }
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

    // The retry state. With the default policy Held is an empty struct
    // and attempt_ is never read or written, so both fold away.
    static inline Held held_{};
    static inline uint8_t attempt_ = 0;
};

} // namespace brio
