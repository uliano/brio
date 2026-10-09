// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// UNDER A RECEIVE ENGINE THE RECEIVE FIFO THRESHOLD IS THE TASK'S: seven
// eighths, the FIFO wedge's watch (docs/stm32g0/usart.md). rx_threshold is
// the interrupt receiver's pace, and a value named beside an engine would
// be read by nothing - so it is REFUSED rather than ignored.
#include "stm32g0/clock.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/usart.hpp"
constexpr brio::UartPins p{.tx = {'A', 2, brio::PinFunction::af1},
                           .rx = {'A', 3, brio::PinFunction::af1}};
constexpr brio::UartOptions quarter{.rx_threshold = brio::UartFifoThreshold::quarter};
using Paced = brio::Uart<2, p, 64, 256, brio::NoDmaEngine, brio::DmaRxEngine<1, 1>, quarter>;
void f() { (void)Paced::init(brio::Clock<brio::ClockSource::pll, 64'000'000>{}, 115200); }
