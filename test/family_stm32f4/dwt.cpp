// DWT family smoke TU: the core stratum's CycleCounter over a Cortex-M4
// header - the include-order contract (the device header first, then
// cortexm/dwt.hpp), the verbs, and the counter as the benchmark's ruler:
// util/bench.hpp's CycleRuler, its Stopwatch, and BenchIdle over this
// family's platform still a Platform - on every header of the pack.
#include <stdint.h>

#include "kernel/platform.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/dwt.hpp"
#include "stm32f4/platform.hpp"
#include "util/bench.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::hsi, 16'000'000>;

struct Ruler {
    static uint32_t now() { return CycleCounter::now(); }
    static uint32_t hz() { return SysClock::hz; }
};
static_assert(CycleRuler<Ruler>);

using Idle = BenchIdle<Stm32f4Platform<>, Ruler>;
static_assert(Platform<Idle>);
static_assert(IdleWindow<Idle>);

IsrMeter<Ruler, Idle> meter;

void dwt_verbs() {
    (void)CycleCounter::init();
    (void)CycleCounter::running();
    (void)CycleCounter::now();
    Stopwatch<Ruler> sw;
    sw.start();
    (void)sw.elapsed();
    meter.enter();
    meter.leave();
    (void)bench_counters<Idle>(meter);
}
