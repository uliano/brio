// Must FAIL: a Request's cs_setup_us is spent on the family's own
// microsecond busy-wait, so a chip traits type that names a DelayRate no
// delay_us takes is not enough - Pl022ChipDelay is the half of the
// contract the host checks, and it names the verb.
#include "host/sim_pl022.hpp"
#include "pl022/spi.hpp"

namespace {
struct NoBusyWait : brio::SimPl022 {
    struct DelayRate {
        uint32_t cycles_per_us = 0;
    };
    static constexpr DelayRate delay_rate(uint32_t) { return {}; }
};

constexpr brio::SimPl022Pins pins{.sck = 0, .tx = 1, .rx = 2};
using Host = brio::Pl022Host<NoBusyWait, 0, pins, brio::SimPl022NoEngine, brio::SimPl022NoEngine>;
}   // namespace

void f() { (void)Host::status(); }
