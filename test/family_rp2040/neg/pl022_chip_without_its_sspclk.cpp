// Must FAIL: a chip traits type that does not say which rate of a clock
// type is SSPCLK cannot bring a host up - Pl022ChipClock is the half of
// the contract init() checks, and it names the member.
#include "host/sim_pl022.hpp"
#include "pl022/spi.hpp"

namespace {
struct NoSspClk : brio::SimPl022 {
    template <typename Clock>
    static constexpr uint32_t sspclk_hz(Clock) = delete;
};

constexpr brio::SimPl022Pins pins{.sck = 0, .tx = 1, .rx = 2};
using Host = brio::Pl022Host<NoSspClk, 0, pins, brio::SimPl022NoEngine, brio::SimPl022NoEngine>;
}   // namespace

void f() { (void)Host::init(brio::SimPl022Clock<48'000'000>{}); }
