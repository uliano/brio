// Clock family smoke TU: the one root and the HPRE ladder at compile time
// (every rate 48 MHz divided by a divider exactly, the reset rate among
// them), the static Clock and the DynamicClock over the ticker and a
// console, the Rcc resource's verbs and the clock output - which ANSWERS
// FALSE on a package without PB9 and compiles everywhere.
#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/ticker.hpp"
#include "ch32x035/usart.hpp"

using namespace brio;

using P = Ch32x035Platform<>;

// ---- the ladder --------------------------------------------------------------
static_assert(hpre_divider(0) == 1 && hpre_divider(5) == 6 && hpre_divider(7) == 8);
static_assert(hpre_divider(8) == 2 && hpre_divider(11) == 16 && hpre_divider(15) == 256);
static_assert(hpre_for(hsi_hz, 48'000'000) == 0);
static_assert(hpre_for(hsi_hz, 24'000'000) == 1);
static_assert(hpre_for(hsi_hz, 16'000'000) == 2);
static_assert(hpre_for(hsi_hz, 9'600'000) == 4);
static_assert(hpre_for(hsi_hz, 8'000'000) == 5);   // the reset value, 0101
static_assert(hpre_for(hsi_hz, 3'000'000) == 11);
static_assert(hpre_for(hsi_hz, 187'500) == 15);
static_assert(hpre_for(hsi_hz, 7'000'000) == 0xFF);   // 48/7 is not whole

using Top = Clock<ClockSource::internal, 48'000'000>;
using Reset = Clock<ClockSource::internal, 8'000'000>;
using Slow = Clock<ClockSource::internal, 187'500>;
static_assert(Top::hz == 48'000'000 && Top::pclk_hz == Top::hz && Top::is_static);
static_assert(Top::hpre_code == 0 && Reset::hpre_code == 5 && Slow::hpre_code == 15);
static_assert(Top::sysclk_hz == hsi_hz);

using Serial = Uart<2, P>;
using SysClock = DynamicClock<Top, Ticker, Serial>;
static_assert(!SysClock::is_static);
static_assert(SysClock::rate_count == 16);
static_assert(SysClock::rate_hz(0) == 48'000'000 && SysClock::rate_hz(5) == 8'000'000);
static_assert(SysClock::can_run_at(12'000'000) && !SysClock::can_run_at(7'000'000));
static_assert(SysClock::rebases<Ticker> && SysClock::rebases<Serial>);

void static_verbs() {
    (void)Top::init();
    (void)Top::restore();
    (void)Reset::init();
}

void dynamic_verbs() {
    constexpr SysClock clock;
    (void)SysClock::init();
    (void)Serial::init(clock, 115200);
    (void)Ticker::init(clock);
    SysClock::set<6'000'000>();
    (void)SysClock::set(24'000'000);
    (void)SysClock::hz();
    (void)SysClock::pclk_hz();
    (void)SysClock::rate_index();
    (void)SysClock::restore();
    (void)delay_us(clock, 50);
}

void rcc_verbs() {
    Rcc::enable(Bus::pb2, rcc_pb2_afio);
    (void)Rcc::enabled(Bus::pb2, rcc_pb2_afio);
    Rcc::disable(Bus::pb2, rcc_pb2_afio);
    Rcc::enable(Bus::pb1, rcc_pb1_pwr);
    Rcc::reset(Bus::pb1, rcc_pb1_pwr);
    Rcc::enable(Bus::hb, rcc_hb_dma1);
    Rcc::reset(Bus::hb, rcc_hbrst_pioc);
    (void)Rcc::hsi_on();
    (void)Rcc::hsi_ready();
    (void)Rcc::hsi_start();
    Rcc::hsi_trim(static_cast<uint8_t>(Rcc::hsi_trim() + 1u));
    (void)Rcc::hsi_calibration();
    (void)Rcc::hpre_code();
    Rcc::hpre(5);
    (void)Rcc::hclk_hz();
    Rcc::mco(McoSource::sysclk);
    (void)Rcc::mco();
    (void)Rcc::reset_flags();
    Rcc::clear_reset_flags();
}

void mco_verbs() {
    (void)Mco::has_pad;
    (void)Mco::init(McoSource::hsi);
    Mco::off();
    (void)Mco::source();
}
