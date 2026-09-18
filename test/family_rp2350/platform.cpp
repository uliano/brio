// Platform family smoke TU: the concept, the core's names, the ticker,
// the microsecond counter, the resets and the chip id - every verb
// instantiated once, for both architectures and both packages.
#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/resets.hpp"
#include "rp2350/sysinfo.hpp"
#include "rp2350/ticker.hpp"

using namespace brio;

using P = Rp2350Platform<>;
static_assert(Platform<P>);
static_assert(Platform<Rp2350Platform<1>>);
static_assert(P::ticks_per_second == 1000u);
static_assert(P::atomic_width == 4u);
static_assert(P::core == 0u);
static_assert(irq_priority_levels == 16u);

using SysClock = Clock<ClockSource::pll, 150'000'000>;

void platform_verbs() {
    constexpr SysClock clock;
    { P::CriticalSection cs; }
    P::idle();
    (void)P::interrupts_enabled();
    (void)P::on_own_core();
    (void)P::now();
    (void)P::core_id();
    (void)P::panic_record();
    P::sleep_hook = nullptr;

    (void)Ticker::init(clock);
    Ticker::tick();
    (void)Ticker::ticks();
    (void)Ticker::millis();
    (void)Ticker::secs();
    TimeStamp stamp{};
    Ticker::now(stamp);
    Ticker::advance(3u);
    Ticker::rebase(12'000'000u);
    Ticker::pause();
    Ticker::resume();
    (void)CoreTicker<1>::ticks();

    (void)Mtime::start(clock);
    (void)Mtime::running();
    (void)Mtime::now();
    (void)Mtime::micros();
    Mtime::set_compare(Mtime::now() + 1000u);
    (void)Mtime::compare();
    Mtime::disarm();

    Irq::enable(UART0_IRQ_IRQn);
    Irq::set_pending(SPARE_IRQ_0_IRQn);
    (void)Irq::enabled(TIMER0_IRQ_0_IRQn);
    (void)Irq::pending(SPARE_IRQ_0_IRQn);
    Irq::clear_pending(SPARE_IRQ_0_IRQn);
    Irq::disable(UART0_IRQ_IRQn);
    enable_interrupts();
    disable_interrupts();
    (void)interrupts_enabled();
    wait_for_interrupt();
    static_assert(core_kind == CoreKind::cortex_m33 || core_kind == CoreKind::hazard3);

    (void)Resets::release(ResetBlock::uart0 | ResetBlock::pads_bank0);
    Resets::hold(ResetBlock::pio2);
    (void)Resets::cycle(ResetBlock::adc);
    (void)Resets::released(ResetBlock::all);
    (void)Resets::held(ResetBlock::dma);
    Resets::watchdog_resets(ResetBlock::uart0 | ResetBlock::timer0 | ResetBlock::timer1 |
                            ResetBlock::trng | ResetBlock::sha256 | ResetBlock::hstx);
    (void)Resets::watchdog_resets();

    (void)ChipId::read();
    (void)ChipId::package_sel();
    (void)ChipId::asic();
    (void)ChipId::gitref();

    hw_set(RESETS->WDSEL, 1u);
    hw_clear(RESETS->WDSEL, 1u);
    hw_xor(RESETS->WDSEL, 1u);
    hw_write_masked(RESETS->WDSEL, 0u, 1u);
}
