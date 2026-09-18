// Watchdog family smoke TU: the block's own tick generator, the
// countdown and its selectors, the scratch registers brio may use, and
// the power-on state machine beside them - every verb instantiated once,
// and the reach of the counter checked against the chapter's own number.
#include <type_traits>

#include "rp2350/watchdog.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000>;

// 12.9's LOAD: 0xffffff microseconds, about 16.8 s, with no doubling -
// the RP2040's halving was that chip's erratum E1 and is not this one's.
static_assert(Watchdog::max_timeout_us == 0x00FFFFFFu);
static_assert(Watchdog::max_timeout_us > 16'000'000u);

// The generator this block counts is the TICKS block's, not its own.
static_assert(std::is_same_v<Watchdog::Tick, TickGenerator<TickConsumer::watchdog>>);

// A reboot keeps the oscillators and restarts every other stage.
static_assert((PsmStage::reboot & PsmStage::rosc) == 0u);
static_assert((PsmStage::reboot & PsmStage::xosc) == 0u);
static_assert((PsmStage::reboot | PsmStage::oscillators) == PsmStage::all);
static_assert(PsmStage::proc_cold != PsmStage::proc0);

void watchdog_verbs() {
    (void)Watchdog::init(SysClock{});
    (void)Watchdog::Tick::running();

    Watchdog::start(500'000u);
    Watchdog::start(500'000u, false);
    Watchdog::kick();
    (void)Watchdog::running();
    (void)Watchdog::remaining_us();
    (void)Watchdog::timeout_us();
    Watchdog::system_resets(PsmStage::reboot);
    (void)Watchdog::system_resets();
    (void)Watchdog::reason();
    (void)(Watchdog::reason() == WatchdogReason::timer);
    (void)(Watchdog::reason() == WatchdogReason::force);
    Watchdog::stop();
}

void scratch_verbs() {
    Scratch<0>::write(1u);
    Scratch<1>::write(Scratch<0>::read());
    Scratch<2>::write(0u);
    hw_set(Scratch<3>::reg(), 0xFFu);
    hw_clear(Scratch<3>::reg(), 0xFFu);
    hw_xor(Scratch<3>::reg(), 0xFFu);
    hw_write_masked(Scratch<3>::reg(), 0x1234u, 0xFFFFu);
}

void psm_verbs() {
    Psm::watchdog_resets(PsmStage::reboot);
    (void)Psm::watchdog_resets();
    (void)Psm::done();
    Psm::hold(PsmStage::proc1);
    (void)Psm::held();
    Psm::release(PsmStage::all & ~PsmStage::proc1);
    Resets::watchdog_resets(ResetBlock::all);
    (void)Resets::watchdog_resets();
}
