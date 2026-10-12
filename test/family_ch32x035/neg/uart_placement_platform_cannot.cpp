// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// InRam on a platform whose family does not place .fast_text - this
// family's platform with the trait taken back, which is what every other
// family's platform says by saying nothing: refused at compile time, never
// an orphan section the linker would put wherever its defaults say.
#include <stdint.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/usart.hpp"

struct CannotPlace : brio::Ch32x035Platform<> {
    static constexpr bool places_fast_text = false;
};
using Port = brio::Uart<2, CannotPlace, 64, 64, brio::UartFormat{}, brio::DmaTxEngine<7>, brio::NoDmaEngine, 0,
                        brio::UartOptions{}, brio::InRam>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Port::init(clock, 115200);
    (void)Port::dma_isr();
}
