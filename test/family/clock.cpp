// CLKCTRL family smoke TU. The CFD block is DB-only (compiled out on
// the DA by its header symbol); the DA takes an external CLOCK on PA0
// directly (no XOSCHF block), datasheet-trusted until a DA reaches
// the bench; the HF crystal stays DB-only.
#include "avrdx/clock.hpp"

using namespace brio;

void clock_common() {
    (void)Clock<ClockSource::internal, 24'000'000>::init();
    (void)Clock<ClockSource::external, 16'000'000>::init();   // DB: XOSCHF EXTCLK; DA: direct
    (void)Clock<ClockSource::osc32k, 32'768>::init();
    (void)Oschf::set_hz(24'000'000);
    (void)Oschf::set_hz(5'000'000);            // not a rate OSCHF produces: false, nothing written
    Xosc32k::start_crystal();                  // stops first if enabled: the config always lands
    Xosc32k::start_external();
    (void)DynamicClock<Clock<ClockSource::internal, 24'000'000>>::init();
}

#ifdef CLKCTRL_XOSCHFCTRLA
void clock_db_only() {
    (void)Clock<ClockSource::crystal, 24'000'000>::init();
    Xoschf::start_crystal(24'000'000);
    Xoschf::start_external(16'000'000);
    ClockFailure::watch(CfdSource::main);
    ClockFailure::stop();
}
#endif
