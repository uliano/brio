// Timer family smoke TU: both system timers, every alarm of each with
// its own line, the counter's three reads, pause, the source select and
// the one-way lock - every verb instantiated once, and the interrupt
// numbering the two instances share checked against the device header.
#include "rp2350/timer.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000>;

static_assert(Timer<0>::alarm_count == 4u);
static_assert(Timer<1>::alarm_count == 4u);

// The tick generator each instance owns in the TICKS block (8.5), and
// the reset line each sits behind - both facts of the chip, not of this
// file.
static_assert(Timer<0>::tick_consumer == TickConsumer::timer0);
static_assert(Timer<1>::tick_consumer == TickConsumer::timer1);
static_assert(Timer<0>::reset_block == ResetBlock::timer0);
static_assert(Timer<1>::reset_block == ResetBlock::timer1);

// TIMER0's four alarms are lines 0..3 and TIMER1's are 4..7: one
// numbering for both architectures (3.8.4.2).
static_assert(Timer<0>::irq<0>() == TIMER0_IRQ_0_IRQn);
static_assert(Timer<0>::irq<3>() == TIMER0_IRQ_3_IRQn);
static_assert(Timer<1>::irq<0>() == TIMER1_IRQ_0_IRQn);
static_assert(Timer<1>::irq<3>() == TIMER1_IRQ_3_IRQn);

template <uint8_t n>
void timer_verbs() {
    (void)Timer<n>::init(SysClock{});

    (void)Timer<n>::now();
    (void)Timer<n>::now_low();
    (void)Timer<n>::now_latched();

    Timer<n>::template alarm<0>(1234u);
    Timer<n>::template alarm_in<1>(2000u);
    (void)Timer<n>::template alarm_at<1>();
    (void)Timer<n>::template armed<2>();
    Timer<n>::template disarm<3>();
    Timer<n>::template interrupt<0>(true);
    (void)Timer<n>::template raised<1>();
    (void)Timer<n>::template pending<2>();
    Timer<n>::template clear<3>();
    Timer<n>::template force<0>(true);
    Timer<n>::template force<0>(false);
    (void)Timer<n>::template irq<2>();

    Timer<n>::pause(true);
    (void)Timer<n>::paused();
    Timer<n>::pause(false);
    Timer<n>::debug_pause(true, true);
    (void)Timer<n>::debug_paused(0);
    (void)Timer<n>::debug_paused(1);
    Timer<n>::debug_pause(false, false);

    Timer<n>::source(TimerSource::sysclk);
    (void)(Timer<n>::source() == TimerSource::tick);
    Timer<n>::source(TimerSource::tick);

    (void)Timer<n>::locked();

    // The tick generator behind this instance, reached by the alias the
    // timer publishes rather than spelled again.
    (void)Timer<n>::Tick::running();
    (void)Timer<n>::Tick::cycles();
    (void)Timer<n>::Tick::count();
}

void timers() {
    timer_verbs<0>();
    timer_verbs<1>();

    // The lock is a separate function: it is the one verb with no way
    // back but a reset of the block, so nothing calls it by accident.
    Timer<1>::lock();
    (void)Resets::cycle(Timer<1>::reset_block);
}
