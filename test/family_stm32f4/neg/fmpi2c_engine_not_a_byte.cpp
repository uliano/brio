// TXDR and RXDR are one byte wide, so an engine whose element is a
// half-word would move two bus accesses per data register write.
// mcu: stm32f446xx
#include "stm32f4/dma.hpp"
#include "stm32f4/fmpi2c.hpp"
using namespace brio;
constexpr FmpI2cPins pins{.scl = {'C', 6, PinFunction::af4},
                          .sda = {'C', 7, PinFunction::af4}};
using Bus = FmpI2cHost<1, pins, DmaTxEngine<1, 5, 2, uint16_t>, DmaRxEngine<1, 2, 2, uint16_t>>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }
