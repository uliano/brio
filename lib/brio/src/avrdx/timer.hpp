/**
 * @file timer.hpp
 * @author uliano
 * @brief Software timers on the Ticker time base, one list per time unit
 * @date Created on March 5, 2025; devirtualized rewrite 07/2026
 *
 * Rewrite of the AVR-Multislope timer system following the framework
 * decisions - same behavior, modern mechanics:
 *
 *  - NO virtual functions: the old TimerBase had a vtable, a pure-virtual
 *    enforceAbstract() and a virtual destructor whose only effect was
 *    requiring operator delete stubs. Timer<Unit> is now a standalone class;
 *  - member callbacks use a `template <auto Method>` compile-time trampoline
 *    (brio::bind<&Class::method>(obj)) instead of the memcpy'd
 *    pointer-to-member union;
 *  - time units are enforced with a concept instead of a static_assert on
 *    hand-rolled is_same.
 *
 * Behavior kept from the original:
 *  - one intrusive linked list per unit (Timer<Millis>, Timer<Secs>,
 *    Timer<Ticks> are independent);
 *  - overflow-safe expiration compare via signed difference;
 *  - periodic timers reschedule from the missed deadline, or from `now`
 *    when the deadline slipped a whole period (no catch-up bursts);
 *  - check_all() early-outs when the time value has not changed.
 *
 * Usage:
 * ```cpp
 * // free function (or capture-less lambda), periodic:
 * brio::Timer<brio::Millis> heartbeat(500, true, +[] { Led::toggle(); });
 * heartbeat.start();
 *
 * // member function via compile-time trampoline:
 * struct Logger { void flush(); };
 * Logger logger;
 * brio::Timer<brio::Secs> flusher(10, true, brio::bind<&Logger::flush>(&logger));
 * flusher.start();
 *
 * // main loop:
 * for (;;) {
 *     brio::Timer<brio::Millis>::check_all();
 *     brio::Timer<brio::Secs>::check_all();
 * }
 * ```
 */

#pragma once

#include <stdint.h>
#include <concepts>
#include "avrdx/ticker.hpp"

namespace brio {

/// Time-unit tags: which Ticker counter a Timer list runs on.
struct Ticks {};
struct Millis {};
struct Secs {};

template <typename U>
concept time_unit =
    std::same_as<U, Ticks> || std::same_as<U, Millis> || std::same_as<U, Secs>;

/// A member-function callback bound to an object, built at compile time.
struct BoundCallback {
    void (*invoke)(void *);
    void *context;
};

namespace impl {
template <typename>
struct member_fn_class;
template <typename C>
struct member_fn_class<void (C::*)()> {
    using type = C;
};
} // namespace impl

/**
 * @brief Bind a member function to an object: brio::bind<&Class::method>(&obj)
 *
 * The trampoline is a capture-less lambda generated per <Method>, converted
 * to a plain function pointer: no memcpy of pointers-to-member, no virtual
 * dispatch, and the member call is a direct (inlinable) call.
 */
template <auto Method>
inline BoundCallback bind(typename impl::member_fn_class<decltype(Method)>::type *object) {
    using Obj = typename impl::member_fn_class<decltype(Method)>::type;
    return {[](void *p) { (static_cast<Obj *>(p)->*Method)(); }, object};
}

/**
 * @class Timer
 * @brief One-shot or periodic software timer, polled via check_all()
 * @tparam Unit brio::Ticks, brio::Millis or brio::Secs
 *
 * Timers self-register in a per-unit intrusive list at construction and
 * unlink at destruction. They are neither copyable nor movable (the list
 * holds their address).
 */
template <time_unit Unit>
class Timer {
private:
    static inline Timer *s_head = nullptr;

    Timer *m_next;
    uint32_t m_period;
    uint32_t m_expiration = 0;
    void (*m_function)() = nullptr;      ///< free-function callback (or null)
    BoundCallback m_bound{nullptr, nullptr};  ///< member callback (or null)
    bool m_running = false;
    bool m_expired = false;
    bool m_periodic;

    /// Current time from the Ticker, in this unit.
    static uint32_t time_now() {
        if constexpr (std::same_as<Unit, Millis>) {
            return Ticker::millis();
        } else if constexpr (std::same_as<Unit, Secs>) {
            return Ticker::secs();
        } else {
            return Ticker::ticks();
        }
    }

    void link() {
        m_next = s_head;
        s_head = this;
    }

    /// Fire the callback if expired; reschedule (periodic) or stop (one-shot).
    void check(uint32_t now) {
        if (!m_running) {
            return;
        }
        if (static_cast<int32_t>(now - m_expiration) < 0) {
            return;
        }
        if (m_bound.invoke) {
            m_bound.invoke(m_bound.context);
        } else if (m_function) {
            m_function();
        }
        if (m_periodic) {
            m_expiration += m_period;
            if (static_cast<int32_t>(now - m_expiration) >= 0) {
                m_expiration = now + m_period;  // slipped a whole period
            }
        } else {
            m_running = false;
            m_expired = true;
        }
    }

public:
    /// Timer with a free-function callback (nullptr = poll expired() yourself).
    Timer(uint32_t period, bool periodic, void (*function)() = nullptr)
        : m_period(period), m_function(function), m_periodic(periodic) {
        link();
    }

    /// Timer with a bound member callback (see brio::bind).
    Timer(uint32_t period, bool periodic, BoundCallback callback)
        : m_period(period), m_bound(callback), m_periodic(periodic) {
        link();
    }

    /// Unlink from the per-unit list.
    ~Timer() {
        if (s_head == this) {
            s_head = m_next;
        } else {
            Timer *prev = s_head;
            while (prev && prev->m_next != this) {
                prev = prev->m_next;
            }
            if (prev) {
                prev->m_next = m_next;
            }
        }
    }

    Timer(const Timer &) = delete;
    Timer &operator=(const Timer &) = delete;

    /// Start (or restart) the timer from the current time.
    void start() {
        m_running = true;
        m_expired = false;
        m_expiration = time_now() + m_period;
    }

    void stop() { m_running = false; }

    /// New period; takes effect at the next start()/reschedule.
    void set_period(uint32_t period) { m_period = period; }

    void set_periodic(bool periodic) { m_periodic = periodic; }

    bool running() const { return m_running; }

    /// One-shot timers only: has the timer fired since start()?
    bool expired() const { return m_expired; }

    /**
     * @brief Walk every timer of this unit, firing the expired ones.
     *
     * Call regularly from the main loop, once per unit in use. Early-outs
     * when the unit's time value has not advanced since the last call.
     */
    static void check_all() {
        static uint32_t last_check = 0;
        const uint32_t now = time_now();
        if (now == last_check) {
            return;
        }
        last_check = now;
        for (Timer *t = s_head; t; t = t->m_next) {
            t->check(now);
        }
    }
};

} // namespace brio
