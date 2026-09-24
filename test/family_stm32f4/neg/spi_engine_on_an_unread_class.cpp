// No device header of this pack carries a request mapping, so the
// reserve keys the SPI slice on the part class - and a class whose
// reference manual was not read has no table. An engine there is
// REFUSED: this is the same cell that is correct on the F429, the F446,
// the F411 and the F469.
// mcu: stm32f412zx stm32f401xe
#include "stm32f4/dma.hpp"
#include "stm32f4/spi.hpp"
using namespace brio;
constexpr SpiPins pins{.sck = {'A', 5, PinFunction::af5},
                       .miso = {'A', 6, PinFunction::af5},
                       .mosi = {'A', 7, PinFunction::af5}};
using Bus = SpiHost<1, pins, DmaTxEngine<2, 3, 3>, DmaRxEngine<2, 0, 3>>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }
