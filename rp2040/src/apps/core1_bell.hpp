// core1_bell.hpp - THE RP2040'S SECOND CORE AS A PRESSURE SOURCE, for the
// suites under Rubato that run a pass with their Pressure object fed from
// core 1 instead of from a local timer (test_rp2040_rubato_link,
// test_rp2040_rubato_i2c): core 1 launched into a Tenuto program of one
// object, Bell, which alarm 1 of the system timer posts to at a fixed
// period - re-armed at absolute counts in the alarm's handler, so the
// period does not drift with the handler's entry - and whose dispatch sends
// a Tick to Pressure on core 0 through its inbox (util/inbox.hpp). Each
// preemption on core 0 then arrives as road 1 from the doorbell's handler,
// and core 1, fetching its program from the flash, shares the XIP cache
// with what core 0 runs from it. The system timer's microsecond and clk_sys
// come from one crystal, so a period of p us is p x clk_sys MHz cycles and
// a pass fed from here is judged by the same arithmetic as the local
// timer's (rubato_passes.hpp's judge_pressure()).
//
// APP-LEVEL BENCH TOOLING, included by the suite inside its own anonymous
// namespace with `using namespace brio` in force, after its pack is complete
// (the pack's Pressure, with Tick among its events) and after kernel/
// tenuto.hpp, util/inbox.hpp, rp2040/multicore.hpp, rp2040/timer.hpp,
// rp2040/ticker.hpp and rp2040/nvic.hpp at file scope. It asks of the suite:
// `clock`, `Ruler`, `core_hz` and `spin()`. Its glue binds isr_timer_1 to
// bell_alarm_isr() and isr_sio_proc0 to bell_doorbell_isr() with the pack's
// own landing record, and main() calls Drain0::enable() before the kernel
// runs.

#pragma once

using P1T = Rp2040Platform<1>;

struct Fire {};

/// Core 1's one object: every Fire, posted by alarm 1's handler, answered
/// with a Tick sent to Pressure on core 0.
struct Bell {
    using Event = Fire;
    static inline EventQueue<Event, 4, P1T> queue;
    static inline volatile uint32_t runs = 0;
    static void init() {}
    static void dispatch(const Event&) {
        runs = runs + 1u;
        send<Pressure>(Tick{});
    }
};
using K1T = Tenuto<P1T, Bell>;

/// The system timer's alarm core 1 ticks on, in its NVIC alone.
constexpr uint8_t bell_alarm = 1;
volatile bool bell_running = false;
volatile uint32_t bell_next = 0;
volatile uint32_t bell_period_us = 97;
/// Which program core 1 runs: 0 none (the bootrom), 1 the Bell.
volatile uint32_t core1_stage = 0;

[[noreturn]] void core1_entry() {
    CoreTicker<1>::init(clock);   // core 1's own SysTick, Tenuto's timebase there
    Timer::clear(bell_alarm);
    Timer::interrupt(bell_alarm, true);
    Nvic::clear_pending(Timer::irq(bell_alarm));
    Nvic::enable(Timer::irq(bell_alarm));   // core 1's NVIC
    core1_stage = 1;
    enable_interrupts();
    K1T::run();
}

/// What core 0's doorbell handler drains: Pressure alone receives across.
using Drain0 = Inboxes<Pressure>;

/// Core 1 out of its program and into the bootrom, with core 0's doorbell
/// off around it (the reset's words come through the FIFO), the inbox
/// emptied while nobody sends (test_rp2040_rubato's core1_park()).
bool core1_park() {
    SioDoorbell<0>::disable();
    const bool ok = Core1::reset();
    core1_stage = 0;
    Inbox<Pressure>::clear();
    SioDoorbell<0>::pop_all();
    Nvic::clear_pending(SioDoorbell<0>::irq());
    SioDoorbell<0>::enable();
    return ok;
}

/// Core 1 reset, the inbox emptied, then launched; true when its entry
/// reported within 10 ms.
bool core1_launch() {
    SioDoorbell<0>::disable();
    bool ok = Core1::reset();
    core1_stage = 0;
    Inbox<Pressure>::clear();
    ok = ok && Core1::protocol(&core1_entry);
    SioDoorbell<0>::pop_all();
    Nvic::clear_pending(SioDoorbell<0>::irq());
    SioDoorbell<0>::enable();
    const uint32_t t0 = Ruler::now();
    while (ok && core1_stage != 1u) {
        if (Ruler::now() - t0 >= 10u * (core_hz / 1000u)) {
            return false;
        }
    }
    return ok;
}

bool core1_up = false;

// ---- the four pressure verbs' core-1 halves ------------------------------------------

/// Armed: the alarm off, the period kept, core 1 launched into the Bell.
[[gnu::always_inline]] inline void bell_arm(uint16_t period_us) {
    bell_running = false;
    Timer::disarm(bell_alarm);
    bell_period_us = period_us;
    core1_up = core1_launch();
}

/// Started: the first alarm one period from now, when core 1 came up.
[[gnu::always_inline]] inline void bell_go() {
    if (core1_up) {
        bell_next = Timer::now_low() + bell_period_us;
        bell_running = true;
        Timer::alarm(bell_alarm, bell_next);
    }
}

/// Stopped: the flag first, then the alarm.
[[gnu::always_inline]] inline void bell_halt() {
    bell_running = false;
    Timer::disarm(bell_alarm);
}

/// Quiet: a send already made drained and dispatched, core 1 parked, the
/// alarm disarmed, off and clear.
[[gnu::always_inline]] inline void bell_quiet() {
    spin(20'000u);   // a send already made, drained and dispatched
    (void)core1_park();
    Timer::disarm(bell_alarm);
    Timer::interrupt(bell_alarm, false);
    Timer::clear(bell_alarm);
    core1_up = false;
}

/// Core 1 parked, alarm 1 disarmed, the inbox empty: letter i's half.
[[gnu::always_inline]] inline bool bell_silent() {
    return core1_stage == 0u && !Timer::armed(bell_alarm) && Drain0::idle();
}

// ---- the two vectors' bodies -----------------------------------------------------------

/// isr_timer_1, core 1's alarm: re-armed at absolute counts while the pass
/// runs, the post to its Bell.
[[gnu::always_inline]] inline void bell_alarm_isr() {
    Timer::clear(bell_alarm);
    if (bell_running) {
        bell_next = bell_next + bell_period_us;
        Timer::alarm(bell_alarm, bell_next);
    }
    post<Bell>(Fire{});
}

/// isr_sio_proc0, core 0's doorbell: Inboxes<Pressure>::isr() written out -
/// the bells popped, then the inbox drained into local posts - with each
/// event that crossed recorded where it landed by `note`, the pack's
/// landing record without its post (the drain made the post).
template <void (*note)()>
[[gnu::always_inline]] inline void bell_doorbell_isr() {
    Drain0::Bell::pop_all();
    std::atomic_thread_fence(std::memory_order_acquire);
    const uint8_t n = Inbox<Pressure>::drain();
    for (uint8_t k = 0; k < n; ++k) {
        note();
    }
}
