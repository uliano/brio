// Must FAIL: a chip traits type that does not say which rate of a clock
// type is UARTCLK cannot bring a transport up - Pl011ChipClock is the
// half of the contract init() checks, and it names the member.
#include "host/sim_pl011.hpp"
#include "pl011/uart.hpp"

namespace {
struct NoUartClk : brio::SimPl011 {
    template <typename Clock>
    static constexpr uint32_t uartclk_hz(Clock) = delete;
};

constexpr brio::SimPl011Pins pins{.tx = 0, .rx = 1};
using Port = brio::Pl011Transport<NoUartClk, 0, pins, 64, 64,
                                  brio::SimNoEngine, brio::SimNoEngine>;
}   // namespace

void f() { (void)Port::init(brio::SimPl011Clock<48'000'000>{}, 115200); }
