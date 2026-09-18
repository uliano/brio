// Must FAIL: a chip traits type that does not say which rate of a clock
// type is ic_clk cannot bring a host up - DwApbI2cChipClock is the half
// of the contract init() checks, and it names the member.
#include "dw_apb_i2c/i2c.hpp"
#include "host/sim_dw_apb_i2c.hpp"

namespace {
struct NoIcClk : brio::SimDwApbI2c {
    template <typename Clock>
    static constexpr uint32_t ic_clk_hz(Clock) = delete;
};

constexpr brio::SimDwApbI2cPins pins{.scl = 1, .sda = 0};
using Bus = brio::DwApbI2cHost<NoIcClk, 0, pins, brio::SimI2cNoEngine, brio::SimI2cNoEngine>;
}   // namespace

void f() { (void)Bus::init(brio::SimDwApbI2cClock<48'000'000>{}); }
