// A stream carries one direction and has one FIFO, so two engines of one
// bus pointed at the same stream would each re-program the other's
// block - and stream 3 of DMA2 on channel 3 is a legal cell for SPI1's
// transmit, so the placement check alone would let this through.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/dma.hpp"
#include "stm32f4/spi.hpp"
using namespace brio;
constexpr SpiPins pins{.sck = {'A', 5, PinFunction::af5},
                       .miso = {'A', 6, PinFunction::af5},
                       .mosi = {'A', 7, PinFunction::af5}};
using Bus = SpiHost<1, pins, DmaTxEngine<2, 3, 3>, DmaRxEngine<2, 3, 3>>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }
