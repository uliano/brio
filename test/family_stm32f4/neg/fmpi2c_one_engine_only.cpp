// A write-then-read tenure carries bytes both ways inside ONE bus tenure,
// so a transmit engine with no receive one would leave its read phase
// without a mover and force a hand-over mid-transfer.
// mcu: stm32f446xx
#include "stm32f4/dma.hpp"
#include "stm32f4/fmpi2c.hpp"
using namespace brio;
constexpr FmpI2cPins pins{.scl = {'C', 6, PinFunction::af4},
                          .sda = {'C', 7, PinFunction::af4}};
using Bus = FmpI2cHost<1, pins, DmaTxEngine<1, 5, 2>>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }
