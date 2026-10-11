// LseTracker family smoke TU (RM0444 5.2's frequency measurement with
// TIM16, figure 13): TIM16's TI1 on the LSE, every eighth edge captured
// and moved by a DMA channel into a circular ring; the rate over any
// recent stretch read back as a walk. Every part of the family has TIM16
// and a DMA1 channel 2 (five channels at least), so the default binding
// compiles on all twelve headers.
#include <optional>

#include "stm32g0/clock.hpp"
#include "stm32g0/lse_tracker.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 64'000'000>;
constexpr SysClock clock{};

using Tracker = LseTracker<1, 2>;
using Short = LseTracker<1, 4, 64>;   // the smallest ring: 16 ms of the crystal

static_assert(Tracker::periods == 8u && Tracker::lse_hz == 32'768u);
static_assert(Tracker::interval_us == 244u);
static_assert(Tracker::most == 1024u - Tracker::guard - 1u);
static_assert(Short::most == 55u);
static_assert(Tracker::entries_ms == 250u && Short::entries_ms == 15u);
static_assert(Tracker::Meter::has_tisel, "TIM16's TI1 is selectable (TIM16_TISEL)");
static_assert(Tracker::Meter::dma_compare_request(0) == 44u, "TIM16_CH1's DMAMUX request (table 55)");
// An interval at the fastest clock this family runs stays under the
// 16-bit counter's wrap: 8 x 64 MHz / 32768 = 15625 counts.
static_assert(Tracker::periods * 64'000'000ull / Tracker::lse_hz < 65'536u);

void lse_tracker_verbs() {
    (void)Tracker::start(clock);
    (void)Tracker::running();
    (void)Tracker::filled();
    const std::optional<uint32_t> a = Tracker::hz(Tracker::most);
    const std::optional<uint32_t> b = Tracker::hz_over(64'000u);
    const std::optional<int32_t> c = Tracker::ppm_over(640'000u);
    (void)a;
    (void)b;
    (void)c;
    (void)Tracker::nominal_hz();
    Tracker::stop();
    (void)Short::start(clock);
    Short::stop();
}
