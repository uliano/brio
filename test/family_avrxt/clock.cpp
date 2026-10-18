// CLKCTRL family smoke TU. The CFD block is the DB's and the EA's
// (compiled out on the DA by its header symbol); the DA takes an
// external CLOCK on PA0 directly (no XOSCHF block); the HF crystal is the
// DB's and the EA's. OSCHF's rate is a register field on the DA/DB
// (FRQSEL) and the OSCCFG fuse's on the EA - there the Clock type names it
// as a claim (16 or 20 MHz) that init() checks against the fuse.
#include "avrxt/clock.hpp"

using namespace brio;

// The PDIV codes are written in clock.hpp as the chapter's numbers; here
// they are held against the header's own names, whichever set this part's
// header uses (CLKCTRL_PDIV_2X_gc on the DA/DB, CLKCTRL_PDIV_DIV2_gc on
// the EA).
template <typename E>
concept PdivNamedX = requires { E::CLKCTRL_PDIV_2X_gc; };

template <typename E>
constexpr bool pdiv_codes_hold() {
    auto v = [](auto e) { return static_cast<uint8_t>(static_cast<uint8_t>(e) | CLKCTRL_PEN_bm); };
    if constexpr (PdivNamedX<E>) {
        return v(E::CLKCTRL_PDIV_2X_gc) == static_cast<uint8_t>(ClockDiv::div2) &&
               v(E::CLKCTRL_PDIV_4X_gc) == static_cast<uint8_t>(ClockDiv::div4) &&
               v(E::CLKCTRL_PDIV_6X_gc) == static_cast<uint8_t>(ClockDiv::div6) &&
               v(E::CLKCTRL_PDIV_8X_gc) == static_cast<uint8_t>(ClockDiv::div8) &&
               v(E::CLKCTRL_PDIV_10X_gc) == static_cast<uint8_t>(ClockDiv::div10) &&
               v(E::CLKCTRL_PDIV_12X_gc) == static_cast<uint8_t>(ClockDiv::div12) &&
               v(E::CLKCTRL_PDIV_16X_gc) == static_cast<uint8_t>(ClockDiv::div16) &&
               v(E::CLKCTRL_PDIV_24X_gc) == static_cast<uint8_t>(ClockDiv::div24) &&
               v(E::CLKCTRL_PDIV_32X_gc) == static_cast<uint8_t>(ClockDiv::div32) &&
               v(E::CLKCTRL_PDIV_48X_gc) == static_cast<uint8_t>(ClockDiv::div48) &&
               v(E::CLKCTRL_PDIV_64X_gc) == static_cast<uint8_t>(ClockDiv::div64);
    } else {
        return v(E::CLKCTRL_PDIV_DIV2_gc) == static_cast<uint8_t>(ClockDiv::div2) &&
               v(E::CLKCTRL_PDIV_DIV4_gc) == static_cast<uint8_t>(ClockDiv::div4) &&
               v(E::CLKCTRL_PDIV_DIV6_gc) == static_cast<uint8_t>(ClockDiv::div6) &&
               v(E::CLKCTRL_PDIV_DIV8_gc) == static_cast<uint8_t>(ClockDiv::div8) &&
               v(E::CLKCTRL_PDIV_DIV10_gc) == static_cast<uint8_t>(ClockDiv::div10) &&
               v(E::CLKCTRL_PDIV_DIV12_gc) == static_cast<uint8_t>(ClockDiv::div12) &&
               v(E::CLKCTRL_PDIV_DIV16_gc) == static_cast<uint8_t>(ClockDiv::div16) &&
               v(E::CLKCTRL_PDIV_DIV24_gc) == static_cast<uint8_t>(ClockDiv::div24) &&
               v(E::CLKCTRL_PDIV_DIV32_gc) == static_cast<uint8_t>(ClockDiv::div32) &&
               v(E::CLKCTRL_PDIV_DIV48_gc) == static_cast<uint8_t>(ClockDiv::div48) &&
               v(E::CLKCTRL_PDIV_DIV64_gc) == static_cast<uint8_t>(ClockDiv::div64);
    }
}
static_assert(pdiv_codes_hold<CLKCTRL_PDIV_t>(), "ClockDiv's codes disagree with this header's PDIV");

// The time base rounds UP: the data sheet's own example (DS40002443A
// 12.5.7) and the edges.
static_assert(clock_timebase_for(3'333'333) == 4);
static_assert(clock_timebase_for(20'000'000) == 20);
static_assert(clock_timebase_for(1'000'000) == 1);
static_assert(clock_timebase_for(32'768) == 1);

// The common surface, at a rate every part reaches.
void clock_common() {
    (void)Clock<ClockSource::osc32k, 32'768>::init();
    (void)Clock<ClockSource::xosc32k, 32'768>::init();
    Xosc32k::start_crystal();                  // stops first if enabled: the config always lands
    Xosc32k::start_external();
    Oschf::tune(-3);
    Oschf::autotune(true);
    Oschf::autotune(false);
}

#if defined(CLKCTRL_FRQSEL_gm)
void clock_frqsel() {
    (void)Clock<ClockSource::internal, 24'000'000>::init();
    (void)Clock<ClockSource::external, 16'000'000>::init();   // DB: XOSCHF EXTCLK; DA: direct
    (void)Oschf::set_hz(24'000'000);
    (void)Oschf::set_hz(5'000'000);            // not a rate OSCHF produces: false, nothing written
    (void)DynamicClock<Clock<ClockSource::internal, 24'000'000>>::init();
}
#else
// The EA: the fuse's rate is the claim; a crystal at another rate names
// the fuse's as the fourth argument and falls back to OSCHF over the
// prescaler that gives the same CLK_PER.
void clock_fuse() {
    using Internal20 = Clock<ClockSource::internal, 20'000'000>;
    (void)Internal20::init();
    (void)Internal20::matches_fuse();
    (void)Clock<ClockSource::internal, 16'000'000, ClockDiv::div4>::init();
    (void)Clock<ClockSource::crystal, 20'000'000>::init();
    (void)Clock<ClockSource::crystal, 8'000'000, ClockDiv::div1, 16'000'000>::init();   // OSCHF/2 fallback
    (void)Clock<ClockSource::crystal, 10'000'000, ClockDiv::div1, 20'000'000>::init();  // OSCHF/2 fallback
    (void)Clock<ClockSource::external, 20'000'000, ClockDiv::div2>::init();
    using Dyn = DynamicClock<Internal20>;
    (void)Dyn::init();
    Dyn::set<5'000'000>();
    (void)Dyn::set(10'000'000u);
    ClockTimebase::write(clock_timebase_for(Internal20::hz));
    (void)ClockTimebase::read();
}
#endif

#ifdef CLKCTRL_XOSCHFCTRLA
void clock_xoschf() {
#if defined(CLKCTRL_FRQSEL_gm)
    (void)Clock<ClockSource::crystal, 24'000'000>::init();
#endif
    Xoschf::start_crystal(16'000'000);
    Xoschf::start_external(16'000'000);
    ClockFailure::watch(CfdSource::main);
    ClockFailure::watch(CfdSource::xosc32k);
    ClockFailure::test(true);
    ClockFailure::stop();
}
#endif
