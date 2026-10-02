// The SPI's DR is sixteen bits wide: the engines carry bytes or
// half-words, and an engine whose element is a word would pour two frames
// into one register access.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include <stdint.h>
#include "stm32f4/dma.hpp"
#include "stm32f4/spi.hpp"
using namespace brio;
constexpr SpiPins pins{.sck = {'A', 5, PinFunction::af5},
                       .miso = {'A', 6, PinFunction::af5},
                       .mosi = {'A', 7, PinFunction::af5}};
using Bus = SpiHost<1, pins, DmaTxEngine<2, 3, 3, uint32_t>, DmaRxEngine<2, 0, 3, uint32_t>>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }
