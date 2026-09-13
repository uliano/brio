// The data phase is full duplex and its completion is the RECEIVE
// block's: a transmit engine with no receive one would leave the
// transaction without an end.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/dma.hpp"
#include "stm32f4/spi.hpp"
using namespace brio;
constexpr SpiPins pins{.sck = {'A', 5, PinFunction::af5},
                       .miso = {'A', 6, PinFunction::af5},
                       .mosi = {'A', 7, PinFunction::af5}};
using Bus = SpiHost<1, pins, DmaTxEngine<2, 3, 3>>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }
