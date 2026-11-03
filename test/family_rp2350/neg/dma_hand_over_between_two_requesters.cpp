// Must FAIL: a channel is not handed from one peripheral to another as a program
// runs (docs/design/dma.md, "The ownership rule"). The shape the rule forbids: a
// scratch channel engine armed in turn on UART1's transmit and on a PWM slice's
// wrap. The DREQ is handed at arm(), so the program's two claims are what say it.
#include "rp2350/dma.hpp"
#include "rp2350/pwm.hpp"
#include "rp2350/uart.hpp"
using namespace brio;
using Scratch = DmaTxEngine<10, uint32_t>;
static_assert(dma_claim<DmaSlotId<0, 10>, DmaOwner<DmaPeriph::uart, 1, DmaSignal::tx>>);
static_assert(dma_claim<DmaSlotId<0, 10>, DmaOwner<DmaPeriph::pwm, 3, DmaSignal::update>>);
void f() {
    Scratch::arm(&Pl011<1>::regs().UARTDR, Dreq::uart1_tx);
    Scratch::stop();
    Scratch::arm(PwmSlice<3>::cc_address(), PwmSlice<3>::dreq);
}
