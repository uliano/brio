// RM0390 table 28 gives this block ONE cell per direction - DMA1 stream 5
// channel 2 for the transmit request, DMA1 stream 2 channel 2 for the
// receive one - and a stream that does not carry the request is a stream
// that will never be triggered. Here the receive engine sits on DMA1
// stream 3 of the same channel, which is TIM4_CH2's cell and nobody
// else's.
// mcu: stm32f446xx
#include "stm32f4/dma.hpp"
#include "stm32f4/fmpi2c.hpp"
using namespace brio;
constexpr FmpI2cPins pins{.scl = {'C', 6, PinFunction::af4},
                          .sda = {'C', 7, PinFunction::af4}};
using Bus = FmpI2cHost<1, pins, DmaTxEngine<1, 5, 2>, DmaRxEngine<1, 3, 2>>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }
