// mcu: ch32v006k8 ch32v003f4
// InRam on a platform whose family does not place .fast_text - this
// family's platform with the trait taken back, which is what every family
// that collects no .fast_text says by saying nothing: refused at compile
// time, never an orphan section the linker would put wherever its defaults
// say.
#include "ch32v00x/clock.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/usart.hpp"

struct CannotPlace : brio::Ch32v00xPlatform<> {
    static constexpr bool places_fast_text = false;
};
using Port = brio::Uart<1, CannotPlace, 64, 64, brio::DmaTxEngine<4>, brio::NoDmaEngine, 0, brio::UartOptions{},
                        brio::InRam>;
void f() {
    (void)Port::init(brio::Clock<brio::ClockSource::pll, 48'000'000>{}, 115200);
    (void)Port::dma_isr();
}
